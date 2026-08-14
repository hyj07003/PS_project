from __future__ import annotations

import math
import os
import threading
import time
from typing import Any

from ..types import BatteryData, ImuData, LidarData, UltrasonicData
from .base import RobotBackend
from . import ros2_runtime

EMOTIONS = (
    "hello",
    "basic",
    "angry",
    "bored",
    "fun",
    "happy",
    "interest",
    "sad",
)


class Ros2Backend(RobotBackend):
    """
    pinky_pro ROS2 토픽/서비스 래퍼.

    Topics (subscribe):
      /battery/percent, /battery/voltage, /scan, /imu_raw,
      /us_sensor/range, /ir_sensor/range
    Topics (publish):
      /cmd_vel
    Services:
      /set_led, /set_brightness, /set_emotion
    """

    name = "ros2"

    def __init__(self, node_name: str = "pinky_bridge") -> None:
        self._node_name = node_name
        self._started = False
        self._lock = __import__("threading").Lock()

        self._battery_percent: float | None = None
        self._battery_voltage: float | None = None
        self._scan: LidarData | None = None
        self._imu: ImuData | None = None
        self._us: UltrasonicData | None = None
        self._battery_source = "ros2"
        self._imu_source = "ros2"
        self._us_source = "ros2"

        self._node = None
        self._cmd_vel_pub = None
        self._set_led_cli = None
        self._set_brightness_cli = None
        self._set_emotion_cli = None
        self._hw = None

        # Navigation (Nav2 bridge)
        self._nav_pose: tuple[float, float, float] | None = None
        # 마지막 map→base TF (홈 제외 우선). ensure 홈 fallback 차단에 사용
        self._last_good_map_pose: tuple[float, float, float] | None = None
        self._pose_hold_until = 0.0
        self._pose_hold_target: tuple[float, float, float] | None = None
        self._localization_idle_frozen = False
        self._amcl_active: bool | None = None
        self._amcl_change_cli = None
        self._amcl_get_cli = None
        self._is_navigating = False
        # NavigateToPose 세션: status 토픽의 옛 SUCCEEDED만으로 idle freeze 하는 레이스 방지
        self._nav_session_id = 0
        self._nav_session_active = False
        self._nav_session_saw_active = False
        self._nav_plan: dict[str, Any] | None = None
        # ArUco visual dock: pin pose at dock site; block idle-freeze / home reseed
        self._visual_dock_active = False
        self._visual_dock_pose: tuple[float, float, float] | None = None
        # 부트 홈 initialpose 루프 / 대기장소 강제 시드 차단
        self._boot_home_cancel = threading.Event()
        self._home_seed_locked_out = False
        self._nav_client = None
        self._plan_client = None
        self._initial_pose_pub = None
        self._cancel_client = None
        self._tf_buffer = None
        self._tf_listener = None
        self._nav_enabled = True
        self._Time = None
        self._GoalStatus = None

    def start(self) -> None:
        if self._started:
            return
        try:
            from geometry_msgs.msg import Twist
            from rclpy.node import Node
            from sensor_msgs.msg import Imu, LaserScan, Range
            from std_msgs.msg import Float32, UInt16MultiArray
        except ImportError as exc:
            raise RuntimeError(
                "ROS2(rclpy)가 없습니다. PINKY_BACKEND=mock 으로 실행하거나 "
                "로봇에서 ROS2 Jazzy 환경을 source 하세요."
            ) from exc

        try:
            from controllers.hardware import HardwareSensors

            self._hw = HardwareSensors()
        except Exception:
            self._hw = None

        ros2_runtime.ensure_runtime()
        self._node = Node(self._node_name)

        self._node.create_subscription(
            Float32, "battery/percent", self._on_battery_percent, 10
        )
        self._node.create_subscription(
            Float32, "battery/voltage", self._on_battery_voltage, 10
        )
        self._node.create_subscription(LaserScan, "scan", self._on_scan, 10)
        self._node.create_subscription(Imu, "imu_raw", self._on_imu, 10)
        self._node.create_subscription(Range, "us_sensor/range", self._on_us, 10)
        self._node.create_subscription(
            UInt16MultiArray, "ir_sensor/range", self._on_ir, 10
        )

        self._cmd_vel_pub = self._node.create_publisher(Twist, "cmd_vel", 10)

        try:
            from pinky_interfaces.srv import Emotion, SetBrightness, SetLed

            self._set_led_cli = self._node.create_client(SetLed, "set_led")
            self._set_brightness_cli = self._node.create_client(
                SetBrightness, "set_brightness"
            )
            self._set_emotion_cli = self._node.create_client(Emotion, "set_emotion")
        except ImportError:
            self._node.get_logger().warn(
                "pinky_interfaces not found; LED/LCD services disabled"
            )

        self._setup_navigation()

        ros2_runtime.add_node(self._node)
        self._started = True

    def _setup_navigation(self) -> None:
        import os

        flag = os.environ.get("PINKY_NAV", "1").lower().strip()
        if flag in ("0", "false", "off", "no"):
            self._nav_enabled = False
            self._seed_home_pose_for_monitor()
            return
        self._nav_enabled = True
        try:
            from geometry_msgs.msg import PoseWithCovarianceStamped
            from nav2_msgs.action import NavigateToPose
            from action_msgs.msg import GoalStatus, GoalStatusArray
            from action_msgs.srv import CancelGoal
            from rclpy.action import ActionClient
            from rclpy.qos import (
                QoSDurabilityPolicy,
                QoSHistoryPolicy,
                QoSProfile,
                QoSReliabilityPolicy,
            )
            from rclpy.time import Time
            from tf2_ros import Buffer, TransformListener
        except ImportError as exc:
            if self._node:
                self._node.get_logger().warn(
                    f"Nav2/tf2 not available ({exc}); navigation disabled"
                )
            self._nav_enabled = False
            self._seed_home_pose_for_monitor()
            return

        self._GoalStatus = GoalStatus
        self._Time = Time

        self._nav_client = ActionClient(self._node, NavigateToPose, "/navigate_to_pose")
        # ComputePathToPose 는 선택: import/서버 실패해도 NavigateToPose 는 유지
        self._plan_client = None
        try:
            from nav2_msgs.action import ComputePathToPose

            self._plan_client = ActionClient(
                self._node, ComputePathToPose, "/compute_path_to_pose"
            )
            if self._node:
                self._node.get_logger().info(
                    "ComputePathToPose client ready (POST /nav/plan)"
                )
        except Exception as exc:
            if self._node:
                self._node.get_logger().warn(
                    f"ComputePathToPose unavailable ({exc}); POST /nav/plan disabled"
                )
        # VOLATILE: TRANSIENT_LOCAL 이면 부트 홈 initialpose 가 래치되어
        # AMCL deactivate→activate 때 대기장소로 다시 점프함
        initialpose_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._initial_pose_pub = self._node.create_publisher(
            PoseWithCovarianceStamped, "initialpose", initialpose_qos
        )
        self._cancel_client = self._node.create_client(
            CancelGoal, "/navigate_to_pose/_action/cancel_goal"
        )

        status_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._node.create_subscription(
            GoalStatusArray,
            "navigate_to_pose/_action/status",
            self._on_nav_status,
            status_qos,
        )

        try:
            from nav_msgs.msg import Path
        except ImportError:
            Path = None  # type: ignore
        if Path is not None:
            plan_qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
            )
            self._node.create_subscription(
                Path, "plan", self._on_global_plan, plan_qos
            )
            self._node.get_logger().info("Subscribed to /plan (global path)")

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self._node, spin_thread=False
        )
        self._setup_amcl_lifecycle_clients()
        self._node.create_timer(0.1, self._update_pose_from_tf)
        self._node.get_logger().info("Navigation bridge (Nav2 + TF map→base_footprint) ready")
        # 모니터용: TF 전에 홈(S1/S2)을 바로 /nav/state 에 반영
        code = (os.environ.get("PINKY_DEVICE_CODE") or "cart-1").strip()
        self._node.get_logger().info(
            f"home pose deviceCode={code} "
            "(cart-2 로봇이면 PINKY_DEVICE_CODE=cart-2 + S2 INITIAL_POSE 필요)"
        )
        self._seed_home_pose_for_monitor()
        delay = float(os.environ.get("PINKY_INITIAL_POSE_DELAY_SEC", "3.0"))
        threading.Timer(max(0.5, delay), self._boot_home_pose_loop).start()

    def _amcl_idle_freeze_enabled(self) -> bool:
        flag = os.environ.get("PINKY_AMCL_IDLE_FREEZE", "1").lower().strip()
        return flag not in ("0", "false", "off", "no")

    def _amcl_node_name(self) -> str:
        return (os.environ.get("PINKY_AMCL_NODE") or "amcl").strip().lstrip("/")

    def _setup_amcl_lifecycle_clients(self) -> None:
        if not self._amcl_idle_freeze_enabled() or self._node is None:
            return
        try:
            from lifecycle_msgs.srv import ChangeState, GetState
        except ImportError:
            self._node.get_logger().warn(
                "lifecycle_msgs missing — AMCL idle freeze disabled"
            )
            return
        node = self._amcl_node_name()
        self._amcl_change_cli = self._node.create_client(
            ChangeState, f"/{node}/change_state"
        )
        self._amcl_get_cli = self._node.create_client(GetState, f"/{node}/get_state")
        self._node.get_logger().info(
            f"AMCL idle freeze enabled (node=/{node})"
        )

    def _amcl_get_state_label(self) -> str | None:
        if self._amcl_get_cli is None:
            return None
        try:
            if not self._amcl_get_cli.wait_for_service(timeout_sec=0.5):
                return None
            from lifecycle_msgs.srv import GetState

            fut = self._amcl_get_cli.call_async(GetState.Request())
            deadline = time.time() + 2.0
            while not fut.done() and time.time() < deadline:
                time.sleep(0.02)
            if not fut.done():
                return None
            res = fut.result()
            if res is None:
                return None
            return str(res.current_state.label)
        except Exception:
            return None

    def _amcl_change_state(self, transition_id: int, label: str) -> bool:
        if self._amcl_change_cli is None:
            return False
        try:
            from lifecycle_msgs.msg import Transition
            from lifecycle_msgs.srv import ChangeState

            if not self._amcl_change_cli.wait_for_service(timeout_sec=1.0):
                if self._node:
                    self._node.get_logger().warn(
                        f"AMCL change_state unavailable (/{self._amcl_node_name()})"
                    )
                return False
            req = ChangeState.Request()
            req.transition = Transition()
            req.transition.id = int(transition_id)
            fut = self._amcl_change_cli.call_async(req)
            deadline = time.time() + 3.0
            while not fut.done() and time.time() < deadline:
                time.sleep(0.02)
            if not fut.done():
                if self._node:
                    self._node.get_logger().warn(f"AMCL {label} timeout")
                return False
            res = fut.result()
            ok = bool(res and res.success)
            if self._node:
                self._node.get_logger().info(
                    f"AMCL {label} → ok={ok} state={self._amcl_get_state_label()}"
                )
            return ok
        except Exception as exc:
            if self._node:
                self._node.get_logger().warn(f"AMCL {label} failed: {exc}")
            return False

    def _amcl_activate(self) -> bool:
        if not self._amcl_idle_freeze_enabled():
            self._amcl_active = True
            return True
        state = self._amcl_get_state_label()
        if state == "active":
            self._amcl_active = True
            return True
        # inactive → activate (TRANSITION_ACTIVATE = 3)
        # unconfigured → configure(1) then activate(3)
        from lifecycle_msgs.msg import Transition

        if state == "unconfigured":
            self._amcl_change_state(Transition.TRANSITION_CONFIGURE, "configure")
        ok = self._amcl_change_state(Transition.TRANSITION_ACTIVATE, "activate")
        self._amcl_active = ok or self._amcl_get_state_label() == "active"
        return bool(self._amcl_active)

    def _amcl_deactivate(self) -> bool:
        if not self._amcl_idle_freeze_enabled():
            self._amcl_active = False
            return True
        state = self._amcl_get_state_label()
        if state == "inactive":
            self._amcl_active = False
            return True
        if state != "active":
            # already off or unknown
            self._amcl_active = False
            return state == "inactive"
        from lifecycle_msgs.msg import Transition

        ok = self._amcl_change_state(Transition.TRANSITION_DEACTIVATE, "deactivate")
        self._amcl_active = not ok and self._amcl_get_state_label() == "active"
        return not self._amcl_active

    def _pose_is_home(self, pose: tuple[float, float, float]) -> bool:
        from ..home_poses import home_pose_for_device

        hx, hy, _ = home_pose_for_device()
        near_m = float(os.environ.get("PINKY_HOME_POSE_NEAR_M", "0.35"))
        dx = float(pose[0]) - hx
        dy = float(pose[1]) - hy
        return (dx * dx + dy * dy) ** 0.5 <= near_m

    def _should_block_home_seed(self) -> bool:
        """투어/작업/홈이 아닌 마지막 pose 가 있으면 S1/S2 강제 시드 금지."""
        with self._lock:
            if self._home_seed_locked_out:
                return True
            if (
                self._visual_dock_active
                or self._nav_session_active
                or self._is_navigating
                or self._visual_dock_pose is not None
            ):
                return True
            lg = self._last_good_map_pose
            frozen = self._nav_pose
            idle = self._localization_idle_frozen
        if lg is not None and not self._pose_is_home(lg):
            return True
        # idle freeze 홈 캐시는 허용; 그 외 비홈 _nav_pose 는 차단
        if frozen is not None and not self._pose_is_home(frozen) and not idle:
            return True
        if self._boot_home_cancel.is_set() and lg is not None:
            return True
        return False

    def _lock_out_home_seed(self, reason: str = "") -> None:
        with self._lock:
            already = self._home_seed_locked_out
            self._home_seed_locked_out = True
        self._boot_home_cancel.set()
        if not already and self._node:
            self._node.get_logger().info(
                f"home seed locked out{(': ' + reason) if reason else ''}"
            )

    def _clear_home_seed_lockout_if_at_home(
        self, pose: tuple[float, float, float] | None
    ) -> None:
        """대기장소 idle freeze 시에만 홈 시드 잠금 해제 (다음 부트 없이 대기 가능)."""
        if pose is None or not self._pose_is_home(pose):
            return
        with self._lock:
            if (
                self._visual_dock_active
                or self._nav_session_active
                or self._is_navigating
            ):
                return
            self._home_seed_locked_out = False

    def _note_map_pose(self, pose: tuple[float, float, float] | None) -> None:
        if pose is None:
            return
        with self._lock:
            self._nav_pose = pose
            if not self._pose_is_home(pose):
                self._last_good_map_pose = pose
                self._home_seed_locked_out = True
                self._boot_home_cancel.set()

    def begin_visual_dock_hold(self) -> bool:
        """Pin monitor pose at current TF for ArUco cmd_vel dock (never fall back to S1/S2)."""
        self._invalidate_pending_freeze()
        pose = self._lookup_tf_pose()
        if pose is None:
            with self._lock:
                cur = self._nav_pose
            if cur is not None and not self._pose_is_home(cur):
                pose = cur
        if pose is None:
            if self._node:
                self._node.get_logger().warn(
                    "visual dock hold: no TF and no non-home pose — skip home fallback"
                )
            return False
        with self._lock:
            self._visual_dock_active = True
            self._visual_dock_pose = pose
            self._nav_pose = pose
            self._last_good_map_pose = pose
            self._home_seed_locked_out = True
            self._pose_hold_target = pose
            self._pose_hold_until = time.time() + 86400.0 * 365
            self._localization_idle_frozen = True
        self._boot_home_cancel.set()
        if self._amcl_idle_freeze_enabled():
            self._amcl_deactivate()
        if self._node:
            self._node.get_logger().info(
                f"visual dock hold → ({pose[0]:.4f},{pose[1]:.4f},yaw={pose[2]:.3f})"
            )
        return True

    def end_visual_dock_hold(self) -> None:
        """Release dock hold; keep last dock pose for next Nav2 ensure (not home)."""
        with self._lock:
            pose = self._visual_dock_pose or self._nav_pose
            self._visual_dock_active = False
            self._visual_dock_pose = None
            self._localization_idle_frozen = False
            self._pose_hold_until = 0.0
            self._pose_hold_target = None
            if pose is not None:
                self._nav_pose = pose
                if not self._pose_is_home(pose):
                    self._last_good_map_pose = pose
        self._invalidate_pending_freeze()
        if self._node and pose is not None:
            self._node.get_logger().info(
                f"visual dock hold released → keep pose "
                f"({pose[0]:.4f},{pose[1]:.4f},yaw={pose[2]:.3f})"
            )

    def _freeze_localization_idle(
        self, pose: tuple[float, float, float] | None = None
    ) -> None:
        """Stop AMCL updates and pin monitor pose (대기 모드)."""
        if self._visual_dock_active:
            with self._lock:
                pose = self._visual_dock_pose or self._nav_pose
        elif pose is None:
            pose = self._lookup_tf_pose()
            if pose is None:
                with self._lock:
                    pose = self._nav_pose
                # Stale monitor seed is S1/S2 — never pin home while robot is elsewhere
                if pose is not None and self._pose_is_home(pose):
                    tf_pose = self._lookup_tf_pose()
                    if tf_pose is not None and not self._pose_is_home(tf_pose):
                        pose = tf_pose
        if pose is not None:
            with self._lock:
                self._nav_pose = pose
                self._pose_hold_target = pose
                self._pose_hold_until = time.time() + 86400.0 * 365
                self._localization_idle_frozen = True
            if self._pose_is_home(pose):
                self._clear_home_seed_lockout_if_at_home(pose)
            else:
                self._lock_out_home_seed("idle freeze non-home")
        else:
            with self._lock:
                self._localization_idle_frozen = True
        if self._amcl_idle_freeze_enabled():
            self._amcl_deactivate()
        if self._node and pose is not None:
            self._node.get_logger().info(
                f"localization idle freeze → "
                f"({pose[0]:.4f},{pose[1]:.4f},yaw={pose[2]:.3f})"
            )

    def _invalidate_pending_freeze(self) -> None:
        """Bump session id so scheduled idle-freeze timers become no-ops."""
        with self._lock:
            self._nav_session_id += 1
            self._nav_session_active = False
            self._nav_session_saw_active = False
            # 새 ensure/goal 직전: 옛 status 기반 navigating 잔상 제거
            # (곧 _start_nav_session 이 True 로 다시 켠다)

    def _start_nav_session(self) -> int:
        self._boot_home_cancel.set()
        with self._lock:
            self._nav_session_id += 1
            sid = self._nav_session_id
            self._nav_session_active = True
            self._nav_session_saw_active = False
            self._is_navigating = True
            self._localization_idle_frozen = False
            if self._nav_pose is not None and not self._pose_is_home(self._nav_pose):
                self._home_seed_locked_out = True
            elif (
                self._last_good_map_pose is not None
                and not self._pose_is_home(self._last_good_map_pose)
            ):
                self._home_seed_locked_out = True
            return sid

    def _end_nav_session(self, sid: int | None = None) -> None:
        with self._lock:
            if sid is not None and sid != self._nav_session_id:
                return
            self._nav_session_active = False
            self._nav_session_saw_active = False
            self._is_navigating = False

    def _schedule_idle_freeze(self, sid: int, delay_sec: float = 0.8) -> None:
        def _run() -> None:
            with self._lock:
                if self._visual_dock_active:
                    return
                if sid != self._nav_session_id:
                    return
                if self._nav_session_active or self._is_navigating:
                    return
                if self._localization_idle_frozen:
                    return
            try:
                self._freeze_localization_idle()
            except Exception:
                pass

        threading.Timer(max(0.1, float(delay_sec)), _run).start()

    def _ensure_localization_for_drive(
        self, x: float | None = None, y: float | None = None, yaw: float | None = None
    ) -> bool:
        """주행 직전 로컬라이즈 준비. True if map→base TF is available afterward.

        - 대기(idle freeze) / AMCL off: activate + initialpose(고정 _nav_pose) + settle
        - 투어 중 AMCL 이미 active: initialpose 생략, goal 만 진행
        - 단 map→base_link TF 가 없으면 Nav2 가 goal 을 REJECT 하므로 재시드
        - 작업/투어 중에는 S1/S2 홈 fallback 금지 (대기 점프 방지)
        """
        # 부트 홈 루프가 돌고 있으면 즉시 중단
        self._boot_home_cancel.set()
        # invalidate 전에 투어/작업 플래그를 읽는다 (invalidate 가 session 을 끈다)
        with self._lock:
            frozen = self._nav_pose
            was_idle_frozen = self._localization_idle_frozen
            last_good = self._last_good_map_pose
            tour_or_work = (
                self._visual_dock_active
                or self._nav_session_active
                or self._is_navigating
                or self._visual_dock_pose is not None
                or self._home_seed_locked_out
            )
            dock_pose = self._visual_dock_pose

        self._invalidate_pending_freeze()

        # 홈이 아닌 마지막 TF/캐시가 있으면 투어로 간주 → 홈 시드 금지
        away_from_home = False
        if last_good is not None and not self._pose_is_home(last_good):
            away_from_home = True
        if frozen is not None and not self._pose_is_home(frozen) and not was_idle_frozen:
            away_from_home = True
        block_home = (
            tour_or_work or away_from_home or self._should_block_home_seed()
        )
        seed: tuple[float, float, float] | None = None
        if x is not None and y is not None:
            seed = (float(x), float(y), float(yaw if yaw is not None else 0.0))
        else:
            tf_now = self._lookup_tf_pose()
            if tf_now is not None:
                seed = (
                    tf_now[0],
                    tf_now[1],
                    float(yaw) if yaw is not None else tf_now[2],
                )
            elif dock_pose is not None:
                seed = (
                    dock_pose[0],
                    dock_pose[1],
                    float(yaw) if yaw is not None else dock_pose[2],
                )
            elif last_good is not None and not self._pose_is_home(last_good):
                seed = (
                    last_good[0],
                    last_good[1],
                    float(yaw) if yaw is not None else last_good[2],
                )
            elif frozen is not None and not (
                was_idle_frozen and self._pose_is_home(frozen)
            ):
                # idle freeze 홈 캐시는 아래에서만 (대기 출발용)
                if not self._pose_is_home(frozen):
                    seed = (
                        frozen[0],
                        frozen[1],
                        float(yaw) if yaw is not None else frozen[2],
                    )
            if seed is None and not block_home:
                # 대기장소에서 첫 출발만 홈 시드 허용
                from ..home_poses import home_pose_for_device

                hx, hy, hyaw = home_pose_for_device()
                seed = (hx, hy, float(yaw) if yaw is not None else hyaw)
                if self._node:
                    self._node.get_logger().info(
                        f"ensure localization → idle home seed "
                        f"({hx:.3f},{hy:.3f},{seed[2]:.3f})"
                    )
            elif seed is None and block_home and self._node:
                self._node.get_logger().warn(
                    "ensure localization → home fallback blocked "
                    f"(tour_or_work={tour_or_work} away={away_from_home}); "
                    "waiting for TF without S1/S2 seed"
                )

        label = self._amcl_get_state_label()
        amcl_on = label == "active"
        tf_now = self._lookup_tf_pose()
        # 투어 연속 구간: AMCL 켜져 있고 idle freeze 아님 + TF 유효 → 시드 생략
        skip_seed = amcl_on and not was_idle_frozen and tf_now is not None

        with self._lock:
            self._localization_idle_frozen = False
            self._pose_hold_until = 0.0
            self._pose_hold_target = None

        if skip_seed:
            with self._lock:
                self._nav_pose = tf_now
                self._last_good_map_pose = tf_now
            if tf_now is not None and not self._pose_is_home(tf_now):
                self._lock_out_home_seed("ensure skip_seed non-home TF")
            if self._node:
                self._node.get_logger().info(
                    "ensure localization → skip initialpose (AMCL already active, TF ok)"
                )
            self._amcl_active = True
            return True

        if amcl_on and tf_now is None and self._node:
            self._node.get_logger().warn(
                "AMCL active but no map→base_footprint/base_link TF — reseeding "
                "initialpose (Nav2 rejects goals without robot pose)"
            )

        ok = self._amcl_activate()
        if not ok:
            time.sleep(0.3)
            ok = self._amcl_activate()
        if not ok and self._node:
            self._node.get_logger().warn(
                "AMCL activate failed — NavigateToPose may not move (map→odom TF)"
            )

        if seed is None:
            # 홈 시드 없이 TF 만 대기
            settle = float(os.environ.get("PINKY_LOCALIZE_SETTLE_SEC", "1.0"))
            if was_idle_frozen or not amcl_on:
                settle = max(
                    settle,
                    float(
                        os.environ.get("PINKY_LOCALIZE_SETTLE_AFTER_FREEZE_SEC", "2.5")
                    ),
                )
            deadline = time.time() + max(0.8, settle)
            while time.time() < deadline:
                got = self._lookup_tf_pose()
                if got is not None:
                    with self._lock:
                        self._nav_pose = got
                        self._last_good_map_pose = got
                    if self._node:
                        self._node.get_logger().info(
                            "ensure localization → TF recovered without home seed "
                            f"({got[0]:.3f},{got[1]:.3f},{got[2]:.3f})"
                        )
                    return True
                time.sleep(0.05)
            if self._node:
                self._node.get_logger().error(
                    "ensure localization → no TF and home seed forbidden"
                )
            return False

        x, y, yaw = seed[0], seed[1], seed[2]
        # 이중 안전: block_home 인데 시드가 홈이면 거부
        if block_home and self._pose_is_home((x, y, yaw)):
            if self._node:
                self._node.get_logger().error(
                    f"ensure localization → refused home seed during tour/work "
                    f"({x:.3f},{y:.3f})"
                )
            return False

        allow_home = (not block_home) and self._pose_is_home((x, y, yaw))
        pub = self._publish_initial_pose_raw(
            float(x), float(y), float(yaw), tight=True, allow_home=allow_home
        )
        if pub.get("ignored") and self._pose_is_home((x, y, yaw)):
            return False
        settle = float(os.environ.get("PINKY_LOCALIZE_SETTLE_SEC", "1.0"))
        # After idle freeze / AMCL off, map→odom TF often needs longer settle.
        if was_idle_frozen or not amcl_on:
            settle = max(
                settle,
                float(os.environ.get("PINKY_LOCALIZE_SETTLE_AFTER_FREEZE_SEC", "2.5")),
            )
        deadline = time.time() + max(0.4, settle)
        tf_ok = False
        while time.time() < deadline:
            if self._lookup_tf_pose() is not None:
                tf_ok = True
                break
            time.sleep(0.05)
        if not tf_ok:
            # Extra burst of initialpose if TF still missing
            self._publish_initial_pose_raw(
                float(x), float(y), float(yaw), tight=True, allow_home=allow_home
            )
            extra = time.time() + max(0.8, settle * 0.6)
            while time.time() < extra:
                if self._lookup_tf_pose() is not None:
                    tf_ok = True
                    break
                time.sleep(0.05)
        if self._node:
            self._node.get_logger().info(
                f"ensure localization → seeded amcl_ok={ok} tf_ok={tf_ok} "
                f"pose=({float(x):.3f},{float(y):.3f},{float(yaw):.3f})"
            )
        with self._lock:
            self._nav_pose = (float(x), float(y), float(yaw))
            if not self._pose_is_home((float(x), float(y), float(yaw))):
                self._last_good_map_pose = (float(x), float(y), float(yaw))
                self._home_seed_locked_out = True
        return tf_ok

    def get_localization_mode(self) -> dict[str, Any]:
        with self._lock:
            frozen = self._localization_idle_frozen
        active = self._amcl_active
        if active is None and self._amcl_idle_freeze_enabled():
            label = self._amcl_get_state_label()
            active = label == "active" if label else None
        return {
            "amclActive": active,
            "localizationMode": "idle" if frozen else "active",
            "amclIdleFreeze": self._amcl_idle_freeze_enabled(),
        }

    def _seed_home_pose_for_monitor(self) -> None:
        """AMCL/TF 대기 중에도 모니터링 현재좌표가 S1/S2로 보이게 시드.

        이미 홈이 아닌 pose / 투어 잠금이 있으면 덮어쓰지 않음.
        """
        if self._should_block_home_seed():
            if self._node:
                self._node.get_logger().info(
                    "monitor seed home skipped (tour/work or non-home pose)"
                )
            return
        from ..home_poses import home_pose_for_device

        x, y, yaw = home_pose_for_device()
        with self._lock:
            if self._last_good_map_pose is not None and not self._pose_is_home(
                self._last_good_map_pose
            ):
                return
            if self._nav_pose is not None and not self._pose_is_home(self._nav_pose):
                return
            self._nav_pose = (x, y, yaw)
            self._pose_hold_target = (x, y, yaw)
            self._pose_hold_until = time.time() + 86400.0 * 365
            self._localization_idle_frozen = True
        if self._node:
            self._node.get_logger().info(
                f"monitor seed home pose → ({x:.4f},{y:.4f},yaw={yaw:.3f})"
            )

    def _boot_home_pose_loop(self) -> None:
        """Publish home initialpose until AMCL TF is near S1/S2, then idle-freeze AMCL.

        투어/작업이 시작되면 즉시 중단 — 중간에 홈 initialpose 를 더 이상 넣지 않음.
        """
        if not self._nav_enabled:
            return
        if self._boot_home_cancel.is_set() or self._should_block_home_seed():
            if self._node:
                self._node.get_logger().info(
                    "boot home pose skipped (already cancelled / tour active)"
                )
            return
        with self._lock:
            if self._visual_dock_active or self._nav_session_active or self._is_navigating:
                if self._node:
                    self._node.get_logger().info(
                        "boot home pose skipped (nav/dock active)"
                    )
                return
        from ..home_poses import home_pose_for_device

        x, y, yaw = home_pose_for_device()
        attempts = max(1, int(os.environ.get("PINKY_INITIAL_POSE_RETRIES", "6")))
        gap = float(os.environ.get("PINKY_INITIAL_POSE_RETRY_GAP_SEC", "2.0"))
        near_m = float(os.environ.get("PINKY_HOME_POSE_NEAR_M", "0.35"))

        with self._lock:
            self._localization_idle_frozen = False
        self._amcl_activate()

        settled = False
        for i in range(1, attempts + 1):
            if self._boot_home_cancel.is_set() or self._should_block_home_seed():
                if self._node:
                    self._node.get_logger().warn(
                        f"boot home pose aborted at try {i}/{attempts} "
                        "(tour/work — refuse further home initialpose)"
                    )
                # 이미 움직인 뒤면 홈으로 freeze 하지 않음
                tf_now = self._lookup_tf_pose()
                if tf_now is not None and not self._pose_is_home(tf_now):
                    self._note_map_pose(tf_now)
                    self._lock_out_home_seed("boot abort non-home TF")
                return
            try:
                result = self._publish_initial_pose_raw(
                    x, y, yaw, tight=True, allow_home=True
                )
                if self._node:
                    self._node.get_logger().info(
                        f"boot home pose try {i}/{attempts} → "
                        f"({x:.4f},{y:.4f},yaw={yaw:.3f}) ok={result.get('success')}"
                    )
            except Exception as exc:
                if self._node:
                    self._node.get_logger().warn(f"boot home pose try {i} failed: {exc}")
            time.sleep(max(0.3, gap * 0.5))
            tf_pose = self._lookup_tf_pose()
            if tf_pose:
                dx = tf_pose[0] - x
                dy = tf_pose[1] - y
                dist = (dx * dx + dy * dy) ** 0.5
                if dist <= near_m:
                    if self._node:
                        self._node.get_logger().info(
                            f"boot home pose settled dist={dist:.3f}m"
                        )
                    settled = True
                    self._freeze_localization_idle(tf_pose)
                    return
                # TF 가 홈이 아니면 부트가 로봇을 끌어오면 안 됨
                if dist > near_m * 2.0:
                    if self._node:
                        self._node.get_logger().warn(
                            f"boot home pose: TF already away from home "
                            f"dist={dist:.3f}m — abort home seeding"
                        )
                    self._note_map_pose(tf_pose)
                    self._lock_out_home_seed("TF away during boot")
                    return
            if i < attempts:
                time.sleep(max(0.2, gap * 0.5))
        if self._boot_home_cancel.is_set() or self._should_block_home_seed():
            if self._node:
                self._node.get_logger().warn(
                    "boot home pose finished aborted — no home idle freeze"
                )
            return
        if self._node:
            self._node.get_logger().warn(
                "boot home pose: TF not near home after retries "
                "(맵/AMCL/라이다 확인) — idle freeze with seed home"
            )
        self._freeze_localization_idle((x, y, yaw) if not settled else None)

    def _on_global_plan(self, msg) -> None:
        """Cache Nav2 /plan (nav_msgs/Path) for GET /nav/plan."""
        path = self._path_msg_to_plan_dict(msg)
        with self._lock:
            self._nav_plan = path

    def get_nav_plan(self) -> dict[str, Any]:
        with self._lock:
            if self._nav_plan is not None:
                plan = dict(self._nav_plan)
                plan["poses"] = [dict(p) for p in self._nav_plan.get("poses") or []]
                return plan
        return {
            "ok": True,
            "frameId": "map",
            "stampSec": None,
            "pointCount": 0,
            "poses": [],
            "message": "no plan",
        }

    def _path_msg_to_plan_dict(self, msg) -> dict[str, Any]:
        """Convert nav_msgs/Path to GET/POST /nav/plan payload shape."""
        poses_out: list[dict[str, float]] = []
        max_pts = max(0, int(os.environ.get("PINKY_PLAN_MAX_POINTS", "500")))
        poses = list(msg.poses or [])
        if max_pts and len(poses) > max_pts:
            step = (len(poses) - 1) / max(1, max_pts - 1)
            idxs = sorted(
                {0, len(poses) - 1}
                | {int(round(i * step)) for i in range(max_pts)}
            )
            poses = [poses[i] for i in idxs if 0 <= i < len(poses)]

        for ps in poses:
            p = ps.pose.position
            q = ps.pose.orientation
            yaw = self._quat_to_yaw(q)
            poses_out.append(
                {"x": float(p.x), "y": float(p.y), "yaw": float(yaw)}
            )

        stamp_sec: float | None = None
        try:
            stamp_sec = float(msg.header.stamp.sec) + float(
                msg.header.stamp.nanosec
            ) * 1e-9
        except Exception:
            stamp_sec = None

        return {
            "ok": True,
            "frameId": str(msg.header.frame_id or "map"),
            "stampSec": stamp_sec,
            "pointCount": len(poses_out),
            "poses": poses_out,
        }

    def compute_path_to(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        timeout_sec: float = 10.0,
        planner_id: str = "",
    ) -> dict[str, Any]:
        if not self._nav_enabled or self._plan_client is None:
            return {"success": False, "message": "navigation not enabled"}

        from action_msgs.msg import GoalStatus
        from nav2_msgs.action import ComputePathToPose

        try:
            self._ensure_localization_for_drive()
        except Exception as exc:
            return {
                "success": False,
                "message": f"localization ensure failed: {exc}",
            }

        if not self._plan_client.wait_for_server(timeout_sec=2.0):
            return {
                "success": False,
                "message": "compute_path_to_pose Action Server not available (planner_server 확인)",
            }

        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = "map"
        goal.goal.header.stamp = self._node.get_clock().now().to_msg()
        goal.goal.pose.position.x = float(x)
        goal.goal.pose.position.y = float(y)
        goal.goal.pose.orientation.z = math.sin(float(yaw) / 2.0)
        goal.goal.pose.orientation.w = math.cos(float(yaw) / 2.0)
        goal.use_start = False
        goal.planner_id = str(planner_id or "")

        deadline = time.time() + max(1.0, float(timeout_sec))
        send_future = self._plan_client.send_goal_async(goal)
        while not send_future.done():
            if time.time() > deadline:
                return {"success": False, "message": "path goal accept timeout"}
            time.sleep(0.02)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return {"success": False, "message": "path planning goal rejected"}

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            if time.time() > deadline:
                return {"success": False, "message": "path planning timeout"}
            time.sleep(0.02)

        wrapped = result_future.result()
        if wrapped is None:
            return {"success": False, "message": "no path planning result"}
        if int(wrapped.status) != GoalStatus.STATUS_SUCCEEDED:
            return {
                "success": False,
                "message": f"path planning ended with status {int(wrapped.status)}",
            }

        path = self._path_msg_to_plan_dict(wrapped.result.path)
        if not path["poses"]:
            return {"success": False, "message": "planner returned empty path"}
        with self._lock:
            self._nav_plan = path
        return {
            "success": True,
            "message": "path computed without moving robot",
            "goal": {"x": float(x), "y": float(y), "yaw": float(yaw)},
            "path": path,
        }

    def _quat_to_yaw(self, q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _on_nav_status(self, msg) -> None:
        # Include CANCELING so brief status gaps / recoveries don't clear the flag.
        active = (
            self._GoalStatus.STATUS_ACCEPTED,
            self._GoalStatus.STATUS_EXECUTING,
            self._GoalStatus.STATUS_CANCELING,
        )
        # Empty status_list is common between updates — keep previous navigating state.
        if not msg.status_list:
            return
        navigating = any(s.status in active for s in msg.status_list)
        with self._lock:
            if self._nav_session_active:
                if navigating:
                    self._nav_session_saw_active = True
                    self._is_navigating = True
                else:
                    # replan/recovery 중 status 가 잠깐 idle 이어도 세션 유지.
                    # (여기서 freeze 하면 AMCL off → 재activate 시 홈 점프)
                    self._is_navigating = True
            else:
                self._is_navigating = navigating
        # idle freeze 는 navigate_to_wait/_finish · cancel · async result 콜백만

    def _lookup_tf_pose(self) -> tuple[float, float, float] | None:
        if self._tf_buffer is None or self._Time is None:
            return None
        # AMCL/costmap 은 base_footprint, 일부 설정은 base_link — 둘 다 시도
        for frame in ("base_footprint", "base_link"):
            try:
                trans = self._tf_buffer.lookup_transform(
                    "map", frame, self._Time()
                )
                t = trans.transform
                yaw = self._quat_to_yaw(t.rotation)
                return (float(t.translation.x), float(t.translation.y), float(yaw))
            except Exception:
                continue
        return None

    def _update_pose_from_tf(self) -> None:
        with self._lock:
            if self._localization_idle_frozen:
                return
        tf_pose = self._lookup_tf_pose()
        if tf_pose is None:
            return
        x, y, yaw = tf_pose
        near_m = float(os.environ.get("PINKY_HOME_POSE_NEAR_M", "0.35"))
        now = time.time()
        with self._lock:
            hold_until = self._pose_hold_until
            hold_target = self._pose_hold_target
            if hold_until > now and hold_target is not None:
                hx, hy, _hyaw = hold_target
                dx = x - hx
                dy = y - hy
                if (dx * dx + dy * dy) ** 0.5 > near_m:
                    return
                self._pose_hold_until = 0.0
                self._pose_hold_target = None
            self._nav_pose = (x, y, yaw)
            self._last_good_map_pose = (x, y, yaw)
            if not self._pose_is_home((x, y, yaw)):
                self._home_seed_locked_out = True
                self._boot_home_cancel.set()

    def get_nav_pose(self) -> dict[str, float] | None:
        with self._lock:
            if self._nav_pose is None:
                return None
            x, y, yaw = self._nav_pose
            return {"x": x, "y": y, "yaw": yaw}

    def is_navigating(self) -> bool:
        with self._lock:
            return self._is_navigating

    def _publish_initial_pose_raw(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        *,
        tight: bool = False,
        allow_home: bool = False,
    ) -> dict[str, Any]:
        if not self._nav_enabled or self._initial_pose_pub is None:
            return {"success": False, "message": "navigation not enabled"}
        # 작업/투어 중 S1/S2 initialpose 절대 금지 (ensure/boot/raw 공통)
        if self._pose_is_home((float(x), float(y), float(yaw))):
            if not allow_home or self._should_block_home_seed():
                if self._node:
                    self._node.get_logger().warn(
                        f"refuse home initialpose ({float(x):.3f},{float(y):.3f}) "
                        f"allow_home={allow_home} "
                        f"blocked={self._should_block_home_seed()}"
                    )
                return {
                    "success": False,
                    "message": "home initialpose blocked during tour/work",
                    "ignored": True,
                    "pose": {"x": x, "y": y, "yaw": yaw},
                }
        from geometry_msgs.msg import PoseWithCovarianceStamped

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        msg.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        if tight:
            msg.pose.covariance[0] = 0.05
            msg.pose.covariance[7] = 0.05
            msg.pose.covariance[35] = 0.05
        else:
            msg.pose.covariance[0] = 0.25
            msg.pose.covariance[7] = 0.25
            msg.pose.covariance[35] = 0.06853891909122467

        repeats = max(1, int(os.environ.get("PINKY_INITIAL_POSE_PUBLISHES", "5")))
        gap = float(os.environ.get("PINKY_INITIAL_POSE_GAP_SEC", "0.15"))
        for _ in range(repeats):
            msg.header.stamp = self._node.get_clock().now().to_msg()
            self._initial_pose_pub.publish(msg)
            if gap > 0:
                time.sleep(gap)
        if not self._pose_is_home((float(x), float(y), float(yaw))):
            self._lock_out_home_seed("non-home initialpose")
        return {
            "success": True,
            "message": f"initialpose published x{repeats}",
            "pose": {"x": x, "y": y, "yaw": yaw},
        }

    def set_initial_pose(self, x: float, y: float, yaw: float = 0.0) -> dict[str, Any]:
        if not self._nav_enabled or self._initial_pose_pub is None:
            return {"success": False, "message": "navigation not enabled"}

        with self._lock:
            if self._visual_dock_active:
                return {
                    "success": True,
                    "message": "ignored initialpose during visual dock",
                    "ignored": True,
                }

        # 홈 initialpose: 투어/작업·비홈 last pose 면 무조건 무시
        if self._pose_is_home((float(x), float(y), float(yaw))):
            if self._should_block_home_seed():
                with self._lock:
                    cur = self._nav_pose or self._last_good_map_pose
                if self._node:
                    self._node.get_logger().warn(
                        f"ignore home initialpose ({float(x):.3f},{float(y):.3f}) "
                        f"— tour/work lockout"
                        + (
                            f" cur=({cur[0]:.3f},{cur[1]:.3f})"
                            if cur is not None
                            else ""
                        )
                    )
                return {
                    "success": True,
                    "message": "ignored home initialpose (tour/work)",
                    "ignored": True,
                    "pose": (
                        {"x": cur[0], "y": cur[1], "yaw": cur[2]}
                        if cur is not None
                        else {"x": x, "y": y, "yaw": yaw}
                    ),
                }
            try:
                from ..home_poses import home_pose_for_device

                hx, hy, _hyaw = home_pose_for_device()
                near_m = float(os.environ.get("PINKY_HOME_POSE_NEAR_M", "0.35"))
                with self._lock:
                    cur = self._nav_pose
                tf_now = self._lookup_tf_pose()
                ref = tf_now or cur
                if ref is not None:
                    cur_far = (ref[0] - hx) ** 2 + (ref[1] - hy) ** 2 > (
                        (near_m * 1.5) ** 2
                    )
                    if cur_far:
                        if self._node:
                            self._node.get_logger().warn(
                                f"ignore home initialpose ({float(x):.3f},{float(y):.3f}) "
                                f"— current pose far ({ref[0]:.3f},{ref[1]:.3f})"
                            )
                        self._lock_out_home_seed("reject home while far")
                        return {
                            "success": True,
                            "message": "ignored home initialpose (robot not at wait spot)",
                            "ignored": True,
                            "pose": {"x": ref[0], "y": ref[1], "yaw": ref[2]},
                        }
            except Exception:
                pass

        with self._lock:
            was_frozen = self._localization_idle_frozen
            navigating = self._is_navigating
            session_active = self._nav_session_active

        # 대기 중 수동 pose: activate → publish → settle → 포즈 hold.
        # 즉시 AMCL deactivate 하지 않음(곧 주행 시 ensure 가 살아 있는 AMCL 사용).
        # 유예 후 lifecycle freeze.
        if self._amcl_idle_freeze_enabled() and (
            was_frozen or (not navigating and not session_active)
        ):
            with self._lock:
                self._localization_idle_frozen = False
            self._amcl_activate()
            result = self._publish_initial_pose_raw(
                float(x), float(y), float(yaw), tight=True
            )
            settle = float(os.environ.get("PINKY_LOCALIZE_SETTLE_SEC", "1.0"))
            if settle > 0:
                time.sleep(min(settle, 1.5))
            delay = float(os.environ.get("PINKY_AMCL_IDLE_FREEZE_DELAY_SEC", "45.0"))
            hold = max(
                float(os.environ.get("PINKY_POSE_HOLD_SEC", "12.0")),
                delay + 1.0 if delay > 0 else 12.0,
            )
            with self._lock:
                self._nav_pose = (float(x), float(y), float(yaw))
                self._pose_hold_target = (float(x), float(y), float(yaw))
                self._pose_hold_until = time.time() + hold
                self._nav_session_id += 1
                freeze_sid = self._nav_session_id
                self._nav_session_active = False
                self._nav_session_saw_active = False
            if delay <= 0:
                self._freeze_localization_idle((float(x), float(y), float(yaw)))
            else:
                self._schedule_idle_freeze(freeze_sid, delay)
            return result

        result = self._publish_initial_pose_raw(
            float(x), float(y), float(yaw), tight=True
        )
        hold = float(os.environ.get("PINKY_POSE_HOLD_SEC", "12.0"))
        with self._lock:
            self._nav_pose = (float(x), float(y), float(yaw))
            self._pose_hold_target = (float(x), float(y), float(yaw))
            self._pose_hold_until = time.time() + max(1.0, hold)
        return result

    def navigate_to(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
    ) -> dict[str, Any]:
        if not self._nav_enabled or self._nav_client is None:
            return {"success": False, "message": "navigation not enabled"}
        from nav2_msgs.action import NavigateToPose

        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            return {
                "success": False,
                "message": "navigate_to_pose Action Server not available (Nav2 기동 확인)",
            }

        try:
            for attempt in range(1, 4):
                try:
                    if self._ensure_localization_for_drive():
                        break
                except Exception as exc:
                    if self._node:
                        self._node.get_logger().warn(
                            f"ensure localization try{attempt}: {exc}"
                        )
                if self._lookup_tf_pose() is not None:
                    break
                time.sleep(0.3)
        except Exception as exc:
            if self._node:
                self._node.get_logger().warn(f"ensure localization: {exc}")

        # 이전 goal 이 실제로 돌 때만 cancel (항상 cancel 하면 새 goal REJECT 레이스)
        if self.is_navigating():
            try:
                self._cancel_nav_sync(timeout_sec=2.0)
            except Exception:
                pass

        if self._lookup_tf_pose() is None:
            return {
                "success": False,
                "message": (
                    "no map→base_footprint/base_link TF after AMCL seed "
                    "(Nav2 would reject goal)"
                ),
            }

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        # zero stamp: Nav2 가 현재 시각 stamp 를 거절하는 경우 회피
        goal.pose.header.stamp.sec = 0
        goal.pose.header.stamp.nanosec = 0
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        goal.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        sid = self._start_nav_session()
        send_fut = self._nav_client.send_goal_async(goal)

        def _on_goal_response(fut) -> None:
            try:
                handle = fut.result()
            except Exception as exc:
                if self._node:
                    self._node.get_logger().warn(f"goal response error: {exc}")
                self._end_nav_session(sid)
                self._schedule_idle_freeze(sid, 0.2)
                return
            if handle is None or not handle.accepted:
                if self._node:
                    self._node.get_logger().warn("NavigateToPose goal rejected")
                self._end_nav_session(sid)
                self._schedule_idle_freeze(sid, 0.2)
                return

            result_fut = handle.get_result_async()

            def _on_result(_rfut) -> None:
                self._end_nav_session(sid)
                delay = float(
                    os.environ.get("PINKY_AMCL_IDLE_FREEZE_DELAY_SEC", "45.0")
                )
                with self._lock:
                    self._nav_session_id += 1
                    freeze_sid = self._nav_session_id
                    self._nav_session_active = False
                    self._nav_session_saw_active = False
                self._schedule_idle_freeze(
                    freeze_sid, delay if delay > 0 else 0.2
                )

            result_fut.add_done_callback(_on_result)

        send_fut.add_done_callback(_on_goal_response)
        return {
            "success": True,
            "message": "goal sent",
            "goal": {"x": x, "y": y, "yaw": yaw},
        }

    def navigate_to_wait(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        timeout_sec: float = 180.0,
    ) -> dict[str, Any]:
        if not self._nav_enabled or self._nav_client is None:
            return {"success": False, "status": "UNAVAILABLE", "message": "navigation not enabled"}
        from action_msgs.msg import GoalStatus
        from nav2_msgs.action import NavigateToPose

        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            return {
                "success": False,
                "status": "UNAVAILABLE",
                "message": "navigate_to_pose Action Server not available (Nav2 기동 확인)",
            }

        try:
            tf_ready = False
            for attempt in range(1, 4):
                try:
                    tf_ready = bool(self._ensure_localization_for_drive())
                except Exception as exc:
                    if self._node:
                        self._node.get_logger().warn(
                            f"ensure localization try{attempt}: {exc}"
                        )
                    tf_ready = False
                if tf_ready or self._lookup_tf_pose() is not None:
                    tf_ready = True
                    break
                if self._node:
                    self._node.get_logger().warn(
                        f"no map→base TF after ensure try{attempt}/3 — retry seed"
                    )
                time.sleep(0.4)
        except Exception as exc:
            if self._node:
                self._node.get_logger().warn(f"ensure localization: {exc}")
            tf_ready = self._lookup_tf_pose() is not None

        deadline = time.time() + max(1.0, float(timeout_sec))

        def _build_goal() -> Any:
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp.sec = 0
            goal.pose.header.stamp.nanosec = 0
            goal.pose.pose.position.x = float(x)
            goal.pose.pose.position.y = float(y)
            goal.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
            goal.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
            return goal

        # 잔여 goal 이 있을 때만 cancel (무조건 cancel→send 는 REJECT 레이스)
        if self.is_navigating():
            try:
                self._cancel_nav_sync(timeout_sec=2.0)
            except Exception:
                pass

        if self._lookup_tf_pose() is None:
            return {
                "success": False,
                "status": "NO_TF",
                "message": (
                    "no map→base_footprint/base_link TF after AMCL activate+initialpose — "
                    "check AMCL/lidar/map (Nav2 rejects goals without robot pose)"
                ),
            }

        sid = self._start_nav_session()
        send_future = self._nav_client.send_goal_async(_build_goal())

        def _finish(payload: dict[str, Any]) -> dict[str, Any]:
            self._end_nav_session(sid)
            # 투어 연속 goal 사이 AMCL 유지: 유예를 길게 (다음 ensure 가 오면 취소)
            # dwell(기본 3s) + 다음 goal 준비보다 충분히 커야 중간단위 freeze 가 안 남
            delay = float(os.environ.get("PINKY_AMCL_IDLE_FREEZE_DELAY_SEC", "45.0"))
            with self._lock:
                self._nav_session_id += 1
                freeze_sid = self._nav_session_id
                self._nav_session_active = False
                self._nav_session_saw_active = False
            if self._amcl_idle_freeze_enabled():
                if delay <= 0:
                    try:
                        self._freeze_localization_idle()
                    except Exception:
                        pass
                else:
                    self._schedule_idle_freeze(freeze_sid, delay)
            if self._node and not payload.get("success"):
                self._node.get_logger().warn(
                    f"goal_wait failed: {payload.get('status')} "
                    f"{payload.get('message')} goal=({x:.3f},{y:.3f},{yaw:.3f})"
                )
            return payload

        def _wait_accept(fut) -> Any:
            while not fut.done():
                if time.time() > deadline:
                    return None
                time.sleep(0.05)
            try:
                return fut.result()
            except Exception as exc:
                if self._node:
                    self._node.get_logger().warn(f"goal accept error: {exc}")
                return None

        goal_handle = _wait_accept(send_future)
        if goal_handle is None and not send_future.done():
            return _finish(
                {
                    "success": False,
                    "status": "TIMEOUT",
                    "message": "goal accept timeout",
                }
            )

        if goal_handle is None or not getattr(goal_handle, "accepted", False):
            # 한 번 더: TF 재확인 + (필요 시) cancel 후 재전송
            if self._node:
                self._node.get_logger().warn(
                    "NavigateToPose rejected — re-ensure TF and retry once"
                )
            try:
                self._ensure_localization_for_drive()
            except Exception:
                pass
            if self.is_navigating():
                try:
                    self._cancel_nav_sync(timeout_sec=2.0)
                except Exception:
                    pass
            if self._lookup_tf_pose() is None:
                return _finish(
                    {
                        "success": False,
                        "status": "NO_TF",
                        "message": (
                            "goal rejected and still no map→base_footprint TF"
                        ),
                    }
                )
            if time.time() > deadline:
                return _finish(
                    {
                        "success": False,
                        "status": "REJECTED",
                        "message": (
                            "goal rejected (Nav2). Check: map→base_footprint TF, "
                            "bt_navigator robot_base_frame, single Nav2 instance"
                        ),
                    }
                )
            send_future = self._nav_client.send_goal_async(_build_goal())
            goal_handle = _wait_accept(send_future)
            if goal_handle is None or not getattr(goal_handle, "accepted", False):
                return _finish(
                    {
                        "success": False,
                        "status": "REJECTED",
                        "message": (
                            "goal rejected (Nav2). Check: map→base_footprint TF, "
                            "bt_navigator robot_base_frame=base_footprint, "
                            "single Nav2 instance (no duplicate launch)"
                        ),
                    }
                )

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            if time.time() > deadline:
                try:
                    self.cancel_navigation(freeze=False)
                except Exception:
                    pass
                return _finish(
                    {
                        "success": False,
                        "status": "TIMEOUT",
                        "message": "nav result timeout",
                    }
                )
            time.sleep(0.05)

        wrapped = result_future.result()
        if wrapped is None:
            return _finish(
                {
                    "success": False,
                    "status": "FAILED",
                    "message": "no result",
                }
            )
        status = int(wrapped.status)
        ok = status == GoalStatus.STATUS_SUCCEEDED
        return _finish(
            {
                "success": ok,
                "status": "SUCCEEDED" if ok else f"STATUS_{status}",
                "message": "arrived" if ok else f"nav ended with status {status}",
                "goal": {"x": x, "y": y, "yaw": yaw},
            }
        )

    def _cancel_nav_sync(self, timeout_sec: float = 2.0) -> dict[str, Any]:
        """Cancel NavigateToPose and wait briefly for the cancel service response."""
        if not self._nav_enabled or self._cancel_client is None:
            self._end_nav_session()
            return {"success": False, "message": "navigation not enabled"}
        from action_msgs.srv import CancelGoal

        if not self._cancel_client.wait_for_service(timeout_sec=1.0):
            self._end_nav_session()
            return {"success": False, "message": "cancel service not available"}
        req = CancelGoal.Request()
        fut = self._cancel_client.call_async(req)
        deadline = time.time() + max(0.2, float(timeout_sec))
        while not fut.done() and time.time() < deadline:
            time.sleep(0.05)
        self._end_nav_session()
        # cancel 직후 서버가 새 goal 을 받을 틈
        time.sleep(0.15)
        return {"success": True, "message": "cancel requested"}

    def cancel_navigation(self, *, freeze: bool = True) -> dict[str, Any]:
        if not self._nav_enabled or self._cancel_client is None:
            self._end_nav_session()
            if freeze:
                try:
                    self._freeze_localization_idle()
                except Exception:
                    pass
            return {"success": False, "message": "navigation not enabled"}
        result = self._cancel_nav_sync(timeout_sec=2.0)
        if freeze:
            self._invalidate_pending_freeze()
            try:
                self._freeze_localization_idle()
            except Exception:
                pass
        return result

    def stop(self) -> None:
        if self._node is not None:
            ros2_runtime.remove_node(self._node)
            self._node = None
        self._started = False

    def is_online(self) -> bool:
        return self._started

    def _on_battery_percent(self, msg) -> None:
        with self._lock:
            self._battery_percent = float(msg.data)
            self._battery_source = "ros2"

    def _on_battery_voltage(self, msg) -> None:
        with self._lock:
            self._battery_voltage = float(msg.data)
            self._battery_source = "ros2"

    def _on_scan(self, msg) -> None:
        data = LidarData(
            ranges=[float(x) if x == x else 0.0 for x in msg.ranges],
            angle_min=float(msg.angle_min),
            angle_max=float(msg.angle_max),
            angle_increment=float(msg.angle_increment),
            range_min=float(msg.range_min),
            range_max=float(msg.range_max),
            frame_id=msg.header.frame_id or "rplidar_link",
            stamp=time.time(),
            raw_pairs=[],
        )
        with self._lock:
            self._scan = data

    def _on_imu(self, msg) -> None:
        data = ImuData(
            orientation={
                "x": float(msg.orientation.x),
                "y": float(msg.orientation.y),
                "z": float(msg.orientation.z),
                "w": float(msg.orientation.w),
            },
            angular_velocity={
                "x": float(msg.angular_velocity.x),
                "y": float(msg.angular_velocity.y),
                "z": float(msg.angular_velocity.z),
            },
            linear_acceleration={
                "x": float(msg.linear_acceleration.x),
                "y": float(msg.linear_acceleration.y),
                "z": float(msg.linear_acceleration.z),
            },
            frame_id=msg.header.frame_id or "imu_link",
            stamp=time.time(),
        )
        with self._lock:
            self._imu = data

    def _on_us(self, msg) -> None:
        with self._lock:
            if self._us is None:
                self._us = UltrasonicData()
            self._us.range_m = float(msg.range)
            self._us.min_range = float(msg.min_range)
            self._us.max_range = float(msg.max_range)
            self._us.frame_id = msg.header.frame_id or "ultrasonic_link"

    def _on_ir(self, msg) -> None:
        with self._lock:
            if self._us is None:
                self._us = UltrasonicData()
            self._us.ir_raw = [int(x) for x in msg.data]

    def get_battery(self) -> BatteryData:
        with self._lock:
            pct, volt = self._battery_percent, self._battery_voltage
        if pct is not None or volt is not None:
            return BatteryData(percent=pct, voltage=volt, source=self._battery_source)
        if self._hw is not None:
            reading = self._hw.read_battery()
            if reading.percent is not None or reading.voltage is not None:
                return BatteryData(
                    percent=reading.percent,
                    voltage=reading.voltage,
                    source=reading.source,
                )
        return BatteryData(percent=None, voltage=None, source="unavailable")

    def get_lidar(self) -> LidarData:
        # LidarReader 원시 포인트(raw_pairs) 우선 — ROS /scan 만 쓰면 밀도 손실
        try:
            from controllers.lidar import LidarReader

            scan = LidarReader.shared().read()
            raw = list(getattr(scan, "raw_pairs", []) or []) if scan else []
            if scan is not None and (raw or scan.ranges):
                return LidarData(
                    ranges=list(scan.ranges),
                    angle_min=scan.angle_min,
                    angle_max=scan.angle_max,
                    angle_increment=scan.angle_increment,
                    range_min=scan.range_min,
                    range_max=scan.range_max,
                    frame_id=scan.frame_id,
                    stamp=scan.stamp,
                    raw_pairs=raw,
                    source=getattr(scan, "source", None) or "lidar",
                )
        except Exception:
            pass
        with self._lock:
            if self._scan is not None and self._scan.ranges:
                # ROS LaserScan → 유효 거리만 raw_pairs 로 복원 (맵 밀도)
                pairs: list[tuple[float, float]] = []
                inc = self._scan.angle_increment or 0.0
                for i, r in enumerate(self._scan.ranges):
                    if r is None or r <= 0 or r != r:
                        continue
                    if self._scan.range_max and r > self._scan.range_max:
                        continue
                    ang = math.degrees(self._scan.angle_min + i * inc)
                    pairs.append((ang % 360.0, float(r)))
                if pairs and not self._scan.raw_pairs:
                    return LidarData(
                        ranges=list(self._scan.ranges),
                        angle_min=self._scan.angle_min,
                        angle_max=self._scan.angle_max,
                        angle_increment=self._scan.angle_increment,
                        range_min=self._scan.range_min,
                        range_max=self._scan.range_max,
                        frame_id=self._scan.frame_id,
                        stamp=self._scan.stamp,
                        raw_pairs=pairs,
                    )
                return self._scan
            return LidarData()

    def get_imu(self) -> ImuData:
        with self._lock:
            if self._imu is not None and self._imu.stamp is not None:
                return self._imu
        if self._hw is not None:
            reading = self._hw.read_imu()
            if reading is not None:
                ox, oy, oz, ow = reading.orientation
                return ImuData(
                    orientation={"x": ox, "y": oy, "z": oz, "w": ow},
                    angular_velocity={
                        "x": reading.angular_velocity[0],
                        "y": reading.angular_velocity[1],
                        "z": reading.angular_velocity[2],
                    },
                    linear_acceleration={
                        "x": reading.linear_acceleration[0],
                        "y": reading.linear_acceleration[1],
                        "z": reading.linear_acceleration[2],
                    },
                    frame_id="imu_link",
                    stamp=time.time(),
                )
        with self._lock:
            return self._imu or ImuData()

    def get_ultrasonic(self) -> UltrasonicData:
        with self._lock:
            if self._us is not None and self._us.range_m is not None:
                return self._us
        if self._hw is not None:
            reading = self._hw.read_ultrasonic()
            if reading.range_m is not None or reading.ir_raw:
                return UltrasonicData(
                    range_m=reading.range_m,
                    ir_raw=reading.ir_raw,
                    frame_id="ultrasonic_link",
                )
        with self._lock:
            return self._us or UltrasonicData()

    def _call_service(self, client, request, timeout: float = 2.0) -> dict[str, Any]:
        if client is None:
            return {"success": False, "message": "service client unavailable"}
        if not client.wait_for_service(timeout_sec=timeout):
            return {"success": False, "message": "service not available"}
        future = client.call_async(request)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not future.done():
            return {"success": False, "message": "service timeout"}
        result = future.result()
        if hasattr(result, "success"):
            return {
                "success": bool(result.success),
                "message": getattr(result, "message", "") or getattr(result, "response", ""),
            }
        return {
            "success": True,
            "message": getattr(result, "response", "ok"),
        }

    def set_led(
        self,
        command: str = "fill",
        r: int = 0,
        g: int = 0,
        b: int = 0,
        pixels: list[int] | None = None,
    ) -> dict[str, Any]:
        if self._set_led_cli is None:
            return {"success": False, "message": "set_led client unavailable"}
        from pinky_interfaces.srv import SetLed

        req = SetLed.Request()
        req.command = command
        req.r = int(r)
        req.g = int(g)
        req.b = int(b)
        req.pixels = list(pixels or [])
        return self._call_service(self._set_led_cli, req)

    def set_brightness(self, brightness: int) -> dict[str, Any]:
        if self._set_brightness_cli is None:
            return {"success": False, "message": "set_brightness client unavailable"}
        from pinky_interfaces.srv import SetBrightness

        req = SetBrightness.Request()
        # field name may be brightness — check srv if needed
        if hasattr(req, "brightness"):
            req.brightness = int(brightness)
        elif hasattr(req, "value"):
            req.value = int(brightness)
        return self._call_service(self._set_brightness_cli, req)

    def set_emotion(self, emotion: str) -> dict[str, Any]:
        if emotion not in EMOTIONS:
            return {"success": False, "message": f"unknown emotion; use one of {EMOTIONS}"}
        if self._set_emotion_cli is None:
            return {"success": False, "message": "set_emotion client unavailable"}
        from pinky_interfaces.srv import Emotion

        req = Emotion.Request()
        req.emotion = emotion
        return self._call_service(self._set_emotion_cli, req)

    def drive(self, linear_x: float, angular_z: float) -> dict[str, Any]:
        if self._cmd_vel_pub is None:
            return {"success": False, "message": "cmd_vel publisher unavailable"}
        from geometry_msgs.msg import Twist

        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self._cmd_vel_pub.publish(msg)
        return {
            "success": True,
            "message": "cmd_vel published",
            "cmdVel": {"linearX": linear_x, "angularZ": angular_z},
        }
