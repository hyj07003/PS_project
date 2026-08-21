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
    # SmartShop mission LCD GIFs (pinky_emotion/emotion/*.gif)
    "pinky_charging",
    "pinky_payment",
    "pinky_moving",
    "pinky_loading",
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
        self._odom_pose: tuple[float, float, float] | None = None
        self._odom_twist: tuple[float, float] | None = None
        self._odom_stamp: float | None = None
        self._motion_lock = threading.Lock()
        self._battery_source = "ros2"
        self._imu_source = "ros2"
        self._us_source = "ros2"

        self._node = None
        self._cmd_vel_pub = None
        self._cmd_vel_aruco_pub = None
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
        self._visual_dock_odom: tuple[float, float, float] | None = None
        # 부트 홈 initialpose 루프 / 대기장소 강제 시드 차단
        self._boot_home_cancel = threading.Event()
        self._home_seed_locked_out = False
        # 웨이포인트 투어 중(홈 goal 도착 전까지) S1/S2 시드·TF 수용 금지
        self._tour_active = False
        self._motion_epoch = 0
        self._active_nav_goal: tuple[float, float, float] | None = None
        self._last_waypoint_pose: tuple[float, float, float] | None = None
        self._last_home_correct_t = 0.0
        self._nav_client = None
        self._plan_client = None
        self._initial_pose_pub = None
        self._cancel_client = None
        self._tf_buffer = None
        self._tf_listener = None
        self._nav_enabled = True
        self._Time = None
        self._GoalStatus = None
        self._navigation_action_state = "UNKNOWN"
        self._navigation_goal_id: str | None = None

    def start(self) -> None:
        if self._started:
            return
        try:
            from geometry_msgs.msg import Twist
            from nav_msgs.msg import Odometry
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
        self._node.create_subscription(Odometry, "odom", self._on_odom, 20)
        self._node.create_subscription(Imu, "imu_raw", self._on_imu, 10)
        self._node.create_subscription(Range, "us_sensor/range", self._on_us, 10)
        self._node.create_subscription(
            UInt16MultiArray, "ir_sensor/range", self._on_ir, 10
        )

        self._cmd_vel_pub = self._node.create_publisher(Twist, "cmd_vel_nav", 10)
        # 아루코 파킹은 선반에 붙으므로 collision_monitor(cmd_vel_nav)를 우회한다.
        aruco_topic = (
            os.environ.get("PINKY_ARUCO_CMD_VEL_TOPIC") or "cmd_vel"
        ).strip() or "cmd_vel"
        self._cmd_vel_aruco_pub = self._node.create_publisher(Twist, aruco_topic, 10)

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
            # Nav2 Jazzy planner_server publishes /plan only if subscription_count > 0.
            # Match both latched (transient local) and live (volatile) publishers.
            # /received_global_plan is the controller remaining-path (map frame only).
            plan_qos_tl = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            )
            plan_qos_vol = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=5,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
            )
            for topic, qos in (
                ("/plan", plan_qos_tl),
                ("/plan", plan_qos_vol),
                ("/received_global_plan", plan_qos_vol),
            ):
                self._node.create_subscription(
                    Path, topic, self._on_global_plan, qos
                )
            self._node.get_logger().info(
                "Subscribed to /plan and /received_global_plan (global path)"
            )

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
        flag = os.environ.get("PINKY_AMCL_IDLE_FREEZE", "0").lower().strip()
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
        if self._amcl_active:
            self._maybe_correct_amcl_home_jump()
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
        near_m = float(os.environ.get("PINKY_HOME_POSE_NEAR_M", "0.15"))
        dx = float(pose[0]) - hx
        dy = float(pose[1]) - hy
        return (dx * dx + dy * dy) ** 0.5 <= near_m

    def _is_home_teleport(
        self,
        new: tuple[float, float, float] | None,
        old: tuple[float, float, float] | None,
    ) -> bool:
        """작업 중 AMCL/TF 가 S1/S2 로 한 프레임에 점프했는지."""
        if new is None:
            return False
        if not self._pose_is_home(new):
            return False
        if old is None:
            return False
        if self._pose_is_home(old):
            return False
        jump = ((float(new[0]) - float(old[0])) ** 2 + (float(new[1]) - float(old[1])) ** 2) ** 0.5
        thresh = max(0.45, float(os.environ.get("PINKY_HOME_TELEPORT_M", "0.50")))
        return jump >= thresh

    def _goal_is_home(self) -> bool:
        g = self._active_nav_goal
        return g is not None and self._pose_is_home(g)

    def _non_home_seed_source(self) -> tuple[float, float, float] | None:
        lg = self._last_good_map_pose
        wp = self._last_waypoint_pose
        cur = self._nav_pose
        dock = self._visual_dock_pose
        for pose in (lg, wp, cur, dock):
            if pose is not None and not self._pose_is_home(pose):
                return pose
        return None

    def _spurious_home_tf(
        self,
        tf_pose: tuple[float, float, float] | None,
        last_good: tuple[float, float, float] | None = None,
    ) -> bool:
        """작업 중 S1/S2 로 '순간 점프'한 TF 인지. 홈 근처 통과·정상 주행은 허용."""
        if tf_pose is None or not self._pose_is_home(tf_pose):
            return False
        if last_good is None:
            last_good = self._last_good_map_pose
        if last_good is None or self._pose_is_home(last_good):
            return False
        # 홈 복귀 goal 이면 teleport 검사만 (정상 도착 허용)
        if self._goal_is_home():
            return self._is_home_teleport(tf_pose, last_good)
        # 비홈 goal / 투어 / ArUco: last_good 과 0.5m+ 떨어진 홈 TF 만 거부 (근처 통과 허용)
        if not (
            self._tour_active
            or self._home_seed_locked_out
            or self._is_navigating
            or self._visual_dock_active
        ):
            return False
        return self._is_home_teleport(tf_pose, last_good)

    def _remember_nav_goal(self, x: float, y: float, yaw: float) -> None:
        pose = (float(x), float(y), float(yaw))
        with self._lock:
            self._active_nav_goal = pose
            self._pose_hold_until = 0.0
            self._pose_hold_target = None
        if self._pose_is_home(pose):
            self._seed_monitor_path(float(x), float(y), float(yaw))
            return
        lg = self._last_good_map_pose
        if self._tour_active or (lg is not None and not self._pose_is_home(lg)):
            self._mark_tour_active("non-home goal")
        self._seed_monitor_path(float(x), float(y), float(yaw))

    def _on_nav_goal_succeeded(self, x: float, y: float, yaw: float) -> None:
        # Keep the live heading. Writing goal yaw into _nav_pose made W6→C
        # look aligned while the chassis was still facing the shelf.
        live = self._lookup_tf_pose()
        if live is not None and not self._spurious_home_tf(live):
            dist = math.hypot(float(live[0]) - float(x), float(live[1]) - float(y))
            if dist <= 0.40:
                pose = (float(live[0]), float(live[1]), float(live[2]))
            else:
                pose = (float(x), float(y), float(yaw))
        else:
            pose = (float(x), float(y), float(yaw))
        if self._pose_is_home((float(x), float(y), float(yaw))):
            with self._lock:
                self._nav_pose = pose
                self._last_good_map_pose = pose
            self._clear_tour_lock("home goal succeeded")
            return
        with self._lock:
            self._nav_pose = pose
            self._last_good_map_pose = pose
            self._last_waypoint_pose = pose
        self._lock_out_home_seed("waypoint arrived")
        with self._lock:
            self._active_nav_goal = None
        self._clear_nav_display_cache()

    def _clear_nav_display_cache(self) -> None:
        """Drop cached monitor path so idle robots do not keep showing old routes."""
        with self._lock:
            self._nav_plan = None

    def _mark_tour_active(self, reason: str = "") -> None:
        already = self._tour_active
        self._tour_active = True
        self._lock_out_home_seed(reason or "tour active")
        if not already and self._node:
            self._node.get_logger().info("tour lock on — refuse S1/S2 pose")

    def _clear_tour_lock(self, reason: str = "") -> None:
        self._tour_active = False
        self._active_nav_goal = None
        self._clear_nav_display_cache()
        lg = self._last_good_map_pose
        pose = self._nav_pose
        # last_good 이 매대면 TF 가 홈으로 점프해도 홈 시드 잠금 유지
        if lg is not None:
            truly_home = self._pose_is_home(lg)
        else:
            truly_home = pose is not None and self._pose_is_home(pose)
        if truly_home:
            self._home_seed_locked_out = False
            self._boot_home_cancel.clear()
        elif lg is not None and not self._pose_is_home(lg):
            self._home_seed_locked_out = True
        if self._node:
            self._node.get_logger().info(
                f"tour lock off{(': ' + reason) if reason else ''}"
            )

    def _bump_motion_epoch(self) -> int:
        with self._lock:
            self._motion_epoch += 1
            return self._motion_epoch

    def _motion_epoch_now(self) -> int:
        with self._lock:
            return int(self._motion_epoch)

    def motion_interrupted(self, epoch: int) -> bool:
        return self._motion_epoch_now() != int(epoch)

    def _correct_home_jump(self) -> None:
        """AMCL/TF 가 홈으로 점프하면 last_good(웨이포인트) 으로 즉시 되돌림."""
        now = time.time()
        if now - self._last_home_correct_t < 1.0:
            return
        src = self._non_home_seed_source()
        if src is None:
            return
        self._last_home_correct_t = now
        driving = self._is_navigating or self._nav_session_active
        if self._node:
            self._node.get_logger().warn(
                f"correct home jump → {'ignore TF' if driving else 'reseed'} "
                f"({src[0]:.3f},{src[1]:.3f})"
            )
        with self._lock:
            self._nav_pose = src
            self._last_good_map_pose = src
            if self._visual_dock_active:
                self._visual_dock_pose = src
        # Nav2 주행 중 initialpose 재발행은 AMCL/Nav2 와 싸워 pose 가 고정됨 → TF 만 무시
        if driving:
            return

        def _pub() -> None:
            try:
                self._publish_initial_pose_raw(
                    src[0], src[1], src[2], tight=True, allow_home=False
                )
            except Exception:
                pass

        threading.Thread(target=_pub, daemon=True).start()

    def _maybe_correct_amcl_home_jump(self) -> None:
        if not (
            self._tour_active
            or self._home_seed_locked_out
            or self._visual_dock_active
        ):
            return
        if self._non_home_seed_source() is None:
            return
        time.sleep(0.2)
        tf_now = self._lookup_tf_pose()
        if self._spurious_home_tf(tf_now):
            self._correct_home_jump()

    def _lookup_drive_tf(self) -> tuple[float, float, float] | None:
        tf_pose = self._lookup_tf_pose()
        if self._spurious_home_tf(tf_pose):
            if self._node and tf_pose is not None:
                self._node.get_logger().warn(
                    f"drive TF at home is spurious ({tf_pose[0]:.3f},{tf_pose[1]:.3f})"
                )
            self._correct_home_jump()
            return None
        return tf_pose

    def _wait_usable_tf(
        self, timeout_sec: float = 1.5
    ) -> tuple[float, float, float] | None:
        deadline = time.time() + max(0.0, float(timeout_sec))
        while True:
            tf_pose = self._lookup_tf_pose()
            if tf_pose is not None and not self._spurious_home_tf(tf_pose):
                return tf_pose
            if self._spurious_home_tf(tf_pose):
                self._correct_home_jump()
            if time.time() >= deadline:
                break
            time.sleep(0.1)
        tf_pose = self._lookup_tf_pose()
        if tf_pose is not None and not self._spurious_home_tf(tf_pose):
            return tf_pose
        return None

    def _should_block_home_seed(self) -> bool:
        """투어/작업/홈이 아닌 마지막 pose 가 있으면 S1/S2 강제 시드 금지.

        Note: `_boot_home_cancel` 은 ensure_localization 진입 시마다 부트 루프
        중단용으로 set 되므로, 그 플래그만으로 홈 시드를 막으면 대기장소
        첫 출발이 항상 실패한다.
        """
        with self._lock:
            if self._home_seed_locked_out:
                return True
            if (
                self._visual_dock_active
                or self._nav_session_active
                or self._is_navigating
                or self._visual_dock_pose is not None
                or self._tour_active
            ):
                return True
            lg = self._last_good_map_pose
            frozen = self._nav_pose
            idle = self._localization_idle_frozen
        if lg is not None and not self._pose_is_home(lg):
            return True
        if frozen is not None and not self._pose_is_home(frozen) and not idle:
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
                or self._tour_active
            ):
                return
            lg = self._last_good_map_pose
        # 투어 중 last_good 이 웨이포인트인데 TF 만 홈으로 점프한 경우 잠금 유지
        if lg is not None and not self._pose_is_home(lg):
            return
        with self._lock:
            self._home_seed_locked_out = False
        self._boot_home_cancel.clear()

    def _note_map_pose(self, pose: tuple[float, float, float] | None) -> None:
        if pose is None:
            return
        if self._spurious_home_tf(pose):
            return
        with self._lock:
            self._nav_pose = pose
            if not self._pose_is_home(pose):
                self._last_good_map_pose = pose
                self._home_seed_locked_out = True
                self._boot_home_cancel.set()

    def begin_visual_dock_hold(self) -> bool:
        """ArUco cmd_vel 구간: AMCL 유지, S1/S2 점프만 차단."""
        self._invalidate_pending_freeze()
        with self._lock:
            last_good = self._last_good_map_pose
            last_wp = self._last_waypoint_pose
            cur = self._nav_pose
        pose = self._lookup_tf_pose()
        if pose is not None and self._spurious_home_tf(pose, last_good):
            if self._node:
                self._node.get_logger().warn(
                    f"visual dock hold: ignore home TF teleport ({pose[0]:.3f},{pose[1]:.3f})"
                )
            pose = None
        if pose is None or self._pose_is_home(pose):
            for alt in (last_good, last_wp, cur):
                if alt is not None and not self._pose_is_home(alt):
                    pose = alt
                    break
        if pose is None:
            if self._node:
                self._node.get_logger().warn(
                    "visual dock hold: no pose at all — cannot guard home jump"
                )
            return False
        with self._lock:
            self._visual_dock_active = True
            self._visual_dock_pose = pose
            self._visual_dock_odom = self._odom_pose
            self._nav_pose = pose
            if not self._pose_is_home(pose):
                self._last_good_map_pose = pose
            self._home_seed_locked_out = True
            self._localization_idle_frozen = False
            self._pose_hold_until = 0.0
            self._pose_hold_target = None
        self._boot_home_cancel.set()
        try:
            self._amcl_activate()
        except Exception:
            pass
        if self._node:
            self._node.get_logger().info(
                f"visual dock hold (AMCL on, block home jump) → "
                f"({pose[0]:.4f},{pose[1]:.4f},yaw={pose[2]:.3f})"
            )
        return True

    def end_visual_dock_hold(self) -> None:
        """Release dock guard; keep AMCL. Reseed only if TF jumped home."""
        tf_now = self._lookup_tf_pose()
        with self._lock:
            last_good = self._last_good_map_pose
            fallback = self._visual_dock_pose or self._nav_pose
            self._visual_dock_active = False
            self._visual_dock_pose = None
            self._visual_dock_odom = None
            self._localization_idle_frozen = False
            self._pose_hold_until = 0.0
            self._pose_hold_target = None
        pose = fallback
        if tf_now is not None and not self._spurious_home_tf(tf_now, last_good):
            pose = tf_now
        elif self._spurious_home_tf(tf_now, last_good):
            pose = last_good or fallback
            if self._node and tf_now is not None:
                self._node.get_logger().warn(
                    f"visual dock end: TF still at home ({tf_now[0]:.3f},{tf_now[1]:.3f}) "
                    "— reseed last_good"
                )
        with self._lock:
            if pose is not None:
                self._nav_pose = pose
                if not self._pose_is_home(pose):
                    self._last_good_map_pose = pose
                    self._last_waypoint_pose = pose
        self._invalidate_pending_freeze()
        try:
            self._amcl_activate()
        except Exception:
            pass
        if pose is not None and not self._pose_is_home(pose):
            if tf_now is None or self._spurious_home_tf(tf_now, last_good):
                try:
                    self._publish_initial_pose_raw(
                        pose[0], pose[1], pose[2], tight=True, allow_home=False
                    )
                except Exception as exc:
                    if self._node:
                        self._node.get_logger().warn(
                            f"visual dock hold: AMCL reseed failed: {exc}"
                        )
        if self._node and pose is not None:
            self._node.get_logger().info(
                f"visual dock hold released → keep pose "
                f"({pose[0]:.4f},{pose[1]:.4f},yaw={pose[2]:.3f})"
            )

    @staticmethod
    def _shift_map_pose_by_odom(
        map_start: tuple[float, float, float],
        odom_start: tuple[float, float, float],
        odom_now: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        dx = float(odom_now[0]) - float(odom_start[0])
        dy = float(odom_now[1]) - float(odom_start[1])
        c = math.cos(-float(odom_start[2]))
        s = math.sin(-float(odom_start[2]))
        local_x = c * dx - s * dy
        local_y = s * dx + c * dy
        yaw0 = float(map_start[2])
        cm = math.cos(yaw0)
        sm = math.sin(yaw0)
        mx = float(map_start[0]) + cm * local_x - sm * local_y
        my = float(map_start[1]) + sm * local_x + cm * local_y
        myaw = yaw0 + (float(odom_now[2]) - float(odom_start[2]))
        while myaw > math.pi:
            myaw -= 2.0 * math.pi
        while myaw < -math.pi:
            myaw += 2.0 * math.pi
        return (mx, my, myaw)

    def _is_nav_session_live(self) -> bool:
        """Nav2 goal 세션 중에만 TF 를 실시간 반영."""
        with self._lock:
            return bool(self._is_navigating or self._nav_session_active)

    def _is_driving(self) -> bool:
        with self._lock:
            return bool(
                self._is_navigating
                or self._nav_session_active
                or self._tour_active
                or self._visual_dock_active
            )

    def _pin_visual_dock_pose(self) -> bool:
        """아루코 구간: AMCL TF를 따르되 S1/S2 점프는 버리고 last_good 유지."""
        with self._lock:
            if not self._visual_dock_active:
                return False
            last_good = self._last_good_map_pose
            fallback = self._visual_dock_pose or self._nav_pose or last_good
        tf_pose = self._lookup_tf_pose()
        if tf_pose is not None and not self._spurious_home_tf(tf_pose, last_good):
            with self._lock:
                self._nav_pose = tf_pose
                self._visual_dock_pose = tf_pose
                if not self._pose_is_home(tf_pose):
                    self._last_good_map_pose = tf_pose
            return True
        if self._spurious_home_tf(tf_pose, last_good):
            self._correct_home_jump()
        pose = fallback
        if pose is not None and self._pose_is_home(pose):
            src = self._non_home_seed_source()
            if src is not None:
                pose = src
        if pose is None:
            return True
        with self._lock:
            self._nav_pose = pose
            if not self._pose_is_home(pose):
                self._visual_dock_pose = pose
                self._last_good_map_pose = pose
        return True

    def _freeze_localization_idle(
        self, pose: tuple[float, float, float] | None = None
    ) -> None:
        """Stop AMCL updates and pin monitor pose (대기 모드)."""
        if self._is_driving():
            if self._node:
                self._node.get_logger().info(
                    "idle freeze skipped (nav/tour/dock active)"
                )
            return
        if pose is None:
            pose = self._lookup_tf_pose()
            if pose is None:
                with self._lock:
                    pose = self._nav_pose
                # Stale monitor seed is S1/S2 — never pin home while robot is elsewhere
                if pose is not None and self._pose_is_home(pose):
                    tf_pose = self._lookup_tf_pose()
                    if tf_pose is not None and not self._pose_is_home(tf_pose):
                        pose = tf_pose
        # 작업/투어 잠금 중이면 홈 좌표로 freeze 하지 않음
        if (
            pose is not None
            and self._pose_is_home(pose)
            and self._should_block_home_seed()
        ):
            tf_pose = self._lookup_tf_pose()
            with self._lock:
                lg = self._last_good_map_pose
                cur = self._nav_pose
            alt = None
            if tf_pose is not None and not self._pose_is_home(tf_pose):
                alt = tf_pose
            elif lg is not None and not self._pose_is_home(lg):
                alt = lg
            elif cur is not None and not self._pose_is_home(cur):
                alt = cur
            if alt is not None:
                if self._node:
                    self._node.get_logger().warn(
                        f"idle freeze skipped home pin; keep "
                        f"({alt[0]:.3f},{alt[1]:.3f})"
                    )
                pose = alt
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
        """Bump freeze-timer generation without tearing down an active nav session."""
        with self._lock:
            # Keep _nav_session_id if a navigate_to_wait session is live — bumping
            # it here used to desync sid and trigger reject/cancel storms.
            if not self._nav_session_active:
                self._nav_session_id += 1
                self._nav_session_saw_active = False
            # 새 ensure/goal 직전: 옛 status 기반 navigating 잔상 제거
            if not self._nav_session_active:
                self._is_navigating = False

    def _start_nav_session(self) -> int:
        self._boot_home_cancel.set()
        self._mark_tour_active("nav session start")
        with self._lock:
            self._nav_session_id += 1
            sid = self._nav_session_id
            self._nav_session_active = True
            self._nav_session_saw_active = False
            # Do NOT mark navigating until Nav2 accepts — otherwise every reject
            # retry thinks we are busy and cancel↔REJECT loops forever.
            self._is_navigating = False
            self._localization_idle_frozen = False
            # 주행 시작 시 pose hold 해제 — 안 하면 TF 갱신이 멈춤
            self._pose_hold_until = 0.0
            self._pose_hold_target = None
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
        self._clear_nav_display_cache()

    def _schedule_idle_freeze(self, sid: int, delay_sec: float = 0.8) -> None:
        def _run() -> None:
            with self._lock:
                if self._visual_dock_active:
                    return
                if sid != self._nav_session_id:
                    return
                if self._nav_session_active or self._is_navigating or self._tour_active:
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
                or self._tour_active
            )
            dock_pose = self._visual_dock_pose

        # 대기장소 idle freeze 상태에서 첫 출발: 이전 투어의 sticky lock 해제
        if was_idle_frozen and (
            (frozen is not None and self._pose_is_home(frozen))
            or (last_good is not None and self._pose_is_home(last_good))
            or (frozen is None and last_good is None)
        ):
            with self._lock:
                self._home_seed_locked_out = False
                self._tour_active = False
            tour_or_work = (
                self._visual_dock_active
                or self._nav_session_active
                or self._is_navigating
                or self._visual_dock_pose is not None
            )

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
        # 홈 *목표* 라도 AMCL 을 S1/S2 에 시드하면 안 된다.
        # (계산대에 있는데 pose 만 대기로 점프 → 복귀가 already-at-goal 로 끝남)
        seed: tuple[float, float, float] | None = None
        if x is not None and y is not None:
            seed = (float(x), float(y), float(yaw if yaw is not None else 0.0))
        else:
            tf_now = self._lookup_tf_pose()
            if self._spurious_home_tf(tf_now, last_good):
                if self._node:
                    self._node.get_logger().warn(
                        f"ensure localization → ignore home TF teleport "
                        f"({tf_now[0]:.3f},{tf_now[1]:.3f}); use last_good"
                    )
                tf_now = None
            if tf_now is not None and not (
                block_home and self._pose_is_home(tf_now) and last_good is not None
                and not self._pose_is_home(last_good)
            ):
                seed = (
                    tf_now[0],
                    tf_now[1],
                    float(yaw) if yaw is not None else tf_now[2],
                )
            elif dock_pose is not None and not self._pose_is_home(dock_pose):
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
            elif (
                self._last_waypoint_pose is not None
                and not self._pose_is_home(self._last_waypoint_pose)
            ):
                wp = self._last_waypoint_pose
                seed = (
                    wp[0],
                    wp[1],
                    float(yaw) if yaw is not None else wp[2],
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
            elif seed is None and block_home:
                repl = self._non_home_seed_source()
                if repl is not None:
                    seed = (
                        repl[0],
                        repl[1],
                        float(yaw) if yaw is not None else repl[2],
                    )
                    if self._node:
                        self._node.get_logger().warn(
                            "ensure localization → home fallback blocked; "
                            f"use last waypoint ({seed[0]:.3f},{seed[1]:.3f})"
                        )
                elif self._node:
                    self._node.get_logger().warn(
                        "ensure localization → home fallback blocked "
                        f"(tour_or_work={tour_or_work} away={away_from_home}); "
                        "waiting for TF without S1/S2 seed"
                    )

        label = self._amcl_get_state_label()
        amcl_on = label == "active"
        tf_now = self._lookup_tf_pose()
        if self._spurious_home_tf(tf_now, last_good):
            if self._node and tf_now is not None:
                self._node.get_logger().warn(
                    f"ensure localization → TF at home is a teleport; will reseed "
                    f"last_good not ({tf_now[0]:.3f},{tf_now[1]:.3f})"
                )
            tf_now = None
        # 투어 연속 구간: AMCL 켜져 있고 idle freeze 아님 + TF 유효 → 시드 생략
        skip_seed = (
            amcl_on
            and not was_idle_frozen
            and tf_now is not None
            and not (block_home and self._pose_is_home(tf_now))
        )

        with self._lock:
            self._localization_idle_frozen = False
            self._pose_hold_until = 0.0
            self._pose_hold_target = None

        if skip_seed:
            with self._lock:
                self._nav_pose = tf_now
                if tf_now is not None and not self._spurious_home_tf(tf_now, last_good):
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
            repl = self._non_home_seed_source()
            if repl is not None:
                seed = (
                    repl[0],
                    repl[1],
                    float(yaw) if yaw is not None else repl[2],
                )
                if self._node:
                    self._node.get_logger().warn(
                        "ensure localization → no TF; reseed from cached pose "
                        f"({seed[0]:.3f},{seed[1]:.3f})"
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
                if got is not None and not self._spurious_home_tf(got, last_good):
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
            # 대기장소 idle 출발: TF 가 안 오면 홈 시드로 한 번 더 시도
            if was_idle_frozen or (
                last_good is not None and self._pose_is_home(last_good)
            ):
                from ..home_poses import home_pose_for_device

                hx, hy, hyaw = home_pose_for_device()
                if frozen is not None and self._pose_is_home(frozen):
                    hx, hy, hyaw = frozen[0], frozen[1], frozen[2]
                elif last_good is not None and self._pose_is_home(last_good):
                    hx, hy, hyaw = last_good[0], last_good[1], last_good[2]
                seed = (hx, hy, float(yaw) if yaw is not None else hyaw)
                with self._lock:
                    self._home_seed_locked_out = False
                    self._tour_active = False
                if self._node:
                    self._node.get_logger().warn(
                        "ensure localization → last-resort wait-spot seed "
                        f"({seed[0]:.3f},{seed[1]:.3f},{seed[2]:.3f})"
                    )
            else:
                if self._node:
                    self._node.get_logger().error(
                        "ensure localization → no TF and home seed forbidden"
                    )
                return False

        if seed is None:
            if self._node:
                self._node.get_logger().error(
                    "ensure localization → no TF and home seed forbidden"
                )
            return False

        x, y, yaw = seed[0], seed[1], seed[2]
        # 계산대/선반 last_good 인데 시드만 S1/S2 이면 점프 — 실제 위치로 교정.
        # 대기장소에 실제로 있으면(새 작업 출발·복귀) 홈 시드를 허용해야 Nav2 가 ABORT 하지 않음.
        if self._pose_is_home((x, y, yaw)):
            repl = self._non_home_seed_source()
            if repl is not None and self._is_home_teleport((x, y, yaw), repl):
                if self._node:
                    self._node.get_logger().warn(
                        f"ensure localization → replace home seed "
                        f"({x:.3f},{y:.3f}) with ({repl[0]:.3f},{repl[1]:.3f})"
                    )
                x, y, yaw = repl[0], repl[1], repl[2]
                seed = (x, y, yaw)
                self._correct_home_jump()
            elif block_home and repl is None and not self._goal_is_home():
                # 잠금만 남고 실제 위치는 대기장소 — 출발 허용
                if self._node:
                    self._node.get_logger().info(
                        f"ensure localization → allow wait-spot seed "
                        f"({x:.3f},{y:.3f}) for departure"
                    )
                with self._lock:
                    self._home_seed_locked_out = False
                    self._last_good_map_pose = (x, y, yaw)
                    self._nav_pose = (x, y, yaw)
                self._boot_home_cancel.clear()

        allow_home = self._pose_is_home((x, y, yaw)) and (
            not block_home
            or self._goal_is_home()
            or self._non_home_seed_source() is None
        )
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

    def get_navigation_readiness(self) -> dict[str, Any]:
        return self._fresh_navigation_readiness()

    def get_navigation_action(self) -> dict[str, Any]:
        with self._lock:
            state = self._navigation_action_state
            goal_id = self._navigation_goal_id
        return {"state": state, "goalId": goal_id}

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
        with self._lock:
            if self._visual_dock_active:
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
        near_m = float(os.environ.get("PINKY_HOME_POSE_NEAR_M", "0.15"))

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
        frame = str(getattr(msg.header, "frame_id", "") or "").strip().lstrip("/")
        if frame and frame not in ("map",):
            return
        path = self._path_msg_to_plan_dict(msg)
        if not path.get("poses"):
            return
        with self._lock:
            self._nav_plan = path
        if self._node:
            self._node.get_logger().debug(
                f"cached nav path {path.get('pointCount', 0)} pts frame={frame or 'map'}"
            )

    def _seed_monitor_path(self, x: float, y: float, yaw: float) -> None:
        """Guarantee a drawable path as soon as a Nav2 goal is remembered."""
        start = self._lookup_tf_pose()
        if start is None:
            with self._lock:
                start = self._nav_pose
        if start is None:
            self._refresh_plan_async(x, y, yaw)
            return
        with self._lock:
            existing = self._nav_plan
            poses = (existing or {}).get("poses") or []
            if len(poses) > 2:
                last = poses[-1]
                try:
                    if math.hypot(float(last["x"]) - x, float(last["y"]) - y) < 0.20:
                        return
                except (KeyError, TypeError, ValueError):
                    pass
            dx = float(x) - float(start[0])
            dy = float(y) - float(start[1])
            if math.hypot(dx, dy) < 0.05:
                # Same XY, different yaw (W6→C): a 2-point path would be invisible.
                hx = float(start[0]) + 0.35 * math.cos(float(yaw))
                hy = float(start[1]) + 0.35 * math.sin(float(yaw))
                end = {"x": hx, "y": hy, "yaw": float(yaw)}
            else:
                end = {"x": float(x), "y": float(y), "yaw": float(yaw)}
            self._nav_plan = {
                "ok": True,
                "frameId": "map",
                "stampSec": time.time(),
                "pointCount": 2,
                "poses": [
                    {
                        "x": float(start[0]),
                        "y": float(start[1]),
                        "yaw": float(start[2]),
                    },
                    end,
                ],
            }
        self._refresh_plan_async(x, y, yaw)

    def _refresh_plan_async(self, x: float, y: float, yaw: float) -> None:
        def _run() -> None:
            time.sleep(0.8)
            with self._lock:
                poses = (self._nav_plan or {}).get("poses") or []
            if len(poses) > 2:
                return
            try:
                self.compute_path_to(
                    x, y, yaw, timeout_sec=4.0, ensure_localization=False, persist=True
                )
            except Exception:
                pass

        threading.Thread(target=_run, daemon=True, name="nav-plan-cache").start()

    def get_nav_plan(self) -> dict[str, Any]:
        with self._lock:
            navigating = bool(
                self._is_navigating or self._nav_session_active
            )
            if self._nav_plan is not None and navigating:
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
        *,
        ensure_localization: bool = True,
        persist: bool = False,
    ) -> dict[str, Any]:
        if not self._nav_enabled or self._plan_client is None:
            return {"success": False, "message": "navigation not enabled"}

        from action_msgs.msg import GoalStatus
        from nav2_msgs.action import ComputePathToPose

        if ensure_localization:
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

        action_status = int(wrapped.status)
        action_result = wrapped.result
        error_code = int(getattr(action_result, "error_code", 0) or 0)
        error_msg = str(getattr(action_result, "error_msg", "") or "")

        if action_status != GoalStatus.STATUS_SUCCEEDED:
            return {
                "success": False,
                "message": f"path planning ended with status {action_status}",
                "actionStatus": action_status,
                "errorCode": error_code,
                "errorMsg": error_msg,
            }

        path = self._path_msg_to_plan_dict(action_result.path)
        if not path["poses"]:
            return {"success": False, "message": "planner returned empty path"}
        if persist:
            with self._lock:
                self._nav_plan = path
        return {
            "success": True,
            "message": "path computed without moving robot",
            "actionStatus": action_status,
            "errorCode": error_code,
            "errorMsg": error_msg,
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
        status_labels = {
            self._GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
            self._GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
            self._GoalStatus.STATUS_EXECUTING: "EXECUTING",
            self._GoalStatus.STATUS_CANCELING: "CANCELING",
            self._GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            self._GoalStatus.STATUS_CANCELED: "CANCELED",
            self._GoalStatus.STATUS_ABORTED: "ABORTED",
        }
        # Empty status_list is common between updates — keep previous navigating state.
        if not msg.status_list:
            return
        latest = msg.status_list[-1]
        with self._lock:
            self._navigation_action_state = status_labels.get(
                int(latest.status), "UNKNOWN"
            )
            try:
                goal_id = latest.goal_info.goal_id
                self._navigation_goal_id = (
                    "".join(f"{b:02x}" for b in goal_id.uuid)
                    if goal_id is not None
                    else None
                )
            except Exception:
                self._navigation_goal_id = None
        navigating = any(s.status in active for s in msg.status_list)
        with self._lock:
            if self._nav_session_active:
                if navigating:
                    self._nav_session_saw_active = True
                    self._is_navigating = True
                elif self._nav_session_saw_active:
                    # Was executing; brief idle during replan — keep flag.
                    self._is_navigating = True
                else:
                    # Goal not accepted yet / rejected / canceled before execute.
                    self._is_navigating = False
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
        if self._pin_visual_dock_pose():
            return
        live_tf = self._is_nav_session_live()
        with self._lock:
            # Nav2 주행 중에만 idle freeze 무시; 아루코는 _pin_visual_dock_pose 가 처리
            if self._localization_idle_frozen and not live_tf:
                return
            last_good = self._last_good_map_pose
        tf_pose = self._lookup_tf_pose()
        if tf_pose is None:
            return
        x, y, yaw = tf_pose
        # Nav2 주행 중에만 TF 그대로; 그 외 홈 순간점프는 무시
        if not live_tf and self._spurious_home_tf(tf_pose, last_good):
            if self._node and (time.time() - self._last_home_correct_t) >= 1.0:
                keep = last_good or self._last_waypoint_pose
                keep_s = (
                    f"({keep[0]:.3f},{keep[1]:.3f})" if keep is not None else "none"
                )
                self._node.get_logger().warn(
                    f"ignore home TF teleport ({x:.3f},{y:.3f}) keep last_good={keep_s}"
                )
            self._correct_home_jump()
            return
        now = time.time()
        with self._lock:
            if live_tf:
                self._localization_idle_frozen = False
                self._pose_hold_until = 0.0
                self._pose_hold_target = None
            elif self._pose_hold_until > now and self._pose_hold_target is not None:
                hx, hy, _hyaw = self._pose_hold_target
                near_m = float(os.environ.get("PINKY_HOME_POSE_NEAR_M", "0.15"))
                dx = x - hx
                dy = y - hy
                if (dx * dx + dy * dy) ** 0.5 > near_m:
                    self._pose_hold_until = 0.0
                    self._pose_hold_target = None
            self._nav_pose = (x, y, yaw)
            if not self._pose_is_home((x, y, yaw)):
                self._last_good_map_pose = (x, y, yaw)
                self._home_seed_locked_out = True
                self._boot_home_cancel.set()
            elif not self._tour_active:
                self._last_good_map_pose = (x, y, yaw)

    def get_nav_pose(self) -> dict[str, float] | None:
        if self._pin_visual_dock_pose():
            pass
        elif self._is_nav_session_live():
            tf_pose = self._lookup_tf_pose()
            if tf_pose is not None and not self._spurious_home_tf(tf_pose):
                x, y, yaw = tf_pose
                with self._lock:
                    self._nav_pose = (x, y, yaw)
                    if not self._pose_is_home((x, y, yaw)):
                        self._last_good_map_pose = (x, y, yaw)
        with self._lock:
            if self._nav_pose is None:
                return None
            x, y, yaw = self._nav_pose
            return {"x": x, "y": y, "yaw": yaw}

    def get_odom_pose(self) -> tuple[float, float, float] | None:
        with self._lock:
            return self._odom_pose

    def get_active_nav_goal(self) -> dict[str, float] | None:
        with self._lock:
            if not (self._is_navigating or self._nav_session_active):
                return None
            g = self._active_nav_goal
        if g is None:
            return None
        return {"x": float(g[0]), "y": float(g[1]), "yaw": float(g[2])}

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
        force: bool = False,
        operator: bool = False,
    ) -> dict[str, Any]:
        if not self._nav_enabled or self._initial_pose_pub is None:
            return {"success": False, "message": "navigation not enabled"}
        with self._lock:
            last_good = self._last_good_map_pose
        # 홈 가드는 투어/작업 중이면 force 여부와 관계없이 차단.
        # last_good 이 웨이포인트인데 홈 좌표를 넣으려는 경우도 차단 (TF 점프 재시드 방지).
        home_req = self._pose_is_home((float(x), float(y), float(yaw)))
        if (
            home_req
            and not operator
            and not allow_home
            and (
                self._should_block_home_seed()
                or self._is_home_teleport((float(x), float(y), float(yaw)), last_good)
            )
        ):
            if self._node:
                self._node.get_logger().warn(
                    f"refuse home initialpose ({float(x):.3f},{float(y):.3f}) "
                    f"allow_home={allow_home} force={force} "
                    f"blocked={self._should_block_home_seed()} teleport=1"
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
        """관리자 수동 pose. 실패 후 대기여도 투어 잠금을 풀고 지정 좌표를 적용한다."""
        if not self._nav_enabled or self._initial_pose_pub is None:
            self._setup_navigation()
        if not self._nav_enabled or self._initial_pose_pub is None:
            return {
                "success": False,
                "message": "navigation not enabled (PINKY_NAV=0 또는 Nav2/tf2 import 실패)",
            }

        if self._visual_dock_active:
            try:
                self.end_visual_dock_hold()
            except Exception:
                pass
        if self._is_navigating or self._nav_session_active:
            try:
                self._cancel_nav_sync(timeout_sec=1.5)
            except Exception:
                pass
        self._clear_tour_lock("operator initialpose")

        pose = (float(x), float(y), float(yaw))
        with self._lock:
            self._localization_idle_frozen = False
        try:
            self._amcl_activate()
        except Exception as exc:
            if self._node:
                self._node.get_logger().warn(f"AMCL activate for initialpose: {exc}")

        result = self._publish_initial_pose_raw(
            float(x),
            float(y),
            float(yaw),
            tight=True,
            allow_home=True,
            force=True,
            operator=True,
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
            self._nav_pose = pose
            self._last_good_map_pose = pose
            self._last_waypoint_pose = pose
            self._pose_hold_target = pose
            self._pose_hold_until = time.time() + hold
            self._nav_session_id += 1
            freeze_sid = self._nav_session_id
            self._nav_session_active = False
            self._nav_session_saw_active = False
            self._is_navigating = False
        if self._pose_is_home(pose):
            with self._lock:
                self._home_seed_locked_out = False
            self._boot_home_cancel.clear()
        else:
            self._lock_out_home_seed("operator non-home pose")

        if self._amcl_idle_freeze_enabled():
            if delay <= 0:
                self._freeze_localization_idle(pose)
            else:
                self._schedule_idle_freeze(freeze_sid, delay)
        if self._node:
            self._node.get_logger().info(
                f"operator initialpose ({pose[0]:.3f},{pose[1]:.3f},{pose[2]:.3f})"
            )
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

        self._remember_nav_goal(x, y, yaw)
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
                if self._wait_usable_tf(0.4) is not None:
                    break
                time.sleep(0.3)
        except Exception as exc:
            if self._node:
                self._node.get_logger().warn(f"ensure localization: {exc}")

        # 이전 goal 이 실제로 돌 때만 cancel (항상 cancel 하면 새 goal REJECT 레이스)
        if self._nav_action_busy() or self.is_navigating():
            try:
                self._cancel_nav_sync(timeout_sec=2.5)
            except Exception:
                pass
        else:
            self._wait_nav_action_idle(1.0)

        if self._wait_usable_tf(1.5) is None:
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
                try:
                    wrapped = _rfut.result()
                    from action_msgs.msg import GoalStatus

                    if (
                        wrapped is not None
                        and int(wrapped.status) == GoalStatus.STATUS_SUCCEEDED
                    ):
                        self._on_nav_goal_succeeded(x, y, yaw)
                except Exception:
                    pass
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

        self._remember_nav_goal(x, y, yaw)
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
                if tf_ready or self._wait_usable_tf(0.4) is not None:
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
            tf_ready = self._wait_usable_tf(0.4) is not None

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

        pose_now = self._lookup_tf_pose()
        if pose_now is None:
            with self._lock:
                pose_now = self._nav_pose
        if pose_now is not None and self._spurious_home_tf(pose_now):
            with self._lock:
                pose_now = self._last_good_map_pose or self._nav_pose
        arrive_m = float(os.environ.get("PINKY_GOAL_ARRIVE_M", "0.12"))
        arrive_yaw = float(os.environ.get("PINKY_GOAL_ARRIVE_YAW_RAD", "0.12"))
        motion_epoch = self._motion_epoch_now()
        if pose_now is not None:
            dist = math.hypot(float(pose_now[0]) - float(x), float(pose_now[1]) - float(y))
            yaw_err = abs(self._angle_error(float(pose_now[2]), float(yaw)))
            fake_home = False
            if dist <= arrive_m and self._pose_is_home((float(x), float(y), float(yaw))):
                with self._lock:
                    lg = self._last_good_map_pose
                fake_home = lg is not None and not self._pose_is_home(lg)
            if dist <= arrive_m and yaw_err <= arrive_yaw and not fake_home:
                self._on_nav_goal_succeeded(x, y, yaw)
                return {
                    "success": True,
                    "status": "SUCCEEDED",
                    "message": (
                        f"already at goal dist={dist:.3f}m yawErr={yaw_err:.3f}rad"
                    ),
                    "goal": {"x": x, "y": y, "yaw": yaw},
                }
            if dist <= arrive_m and yaw_err > arrive_yaw and not fake_home:
                if self._node:
                    self._node.get_logger().info(
                        f"same XY but yaw err={yaw_err:.3f}rad "
                        f"(need {float(yaw):.3f}, have {float(pose_now[2]):.3f}) "
                        "— rotate in place (skip Nav2)"
                    )
                return self._rotate_in_place_to_yaw(
                    x,
                    y,
                    yaw,
                    timeout_sec=max(3.0, deadline - time.time()),
                    arrive_yaw=arrive_yaw,
                )

        # Default: do NOT cancel before a new goal. CancelGoal(empty) races and
        # leaves bt_navigator rejecting with action_state=UNKNOWN/CANCELED.
        cancel_before = (os.environ.get("PINKY_NAV_CANCEL_BEFORE_GOAL") or "0").strip().lower() in (
            "1",
            "true",
            "on",
            "yes",
        )
        if cancel_before:
            pre_state = ""
            with self._lock:
                pre_state = (self._navigation_action_state or "").upper()
            if pre_state in ("ACCEPTED", "EXECUTING", "CANCELING"):
                try:
                    self._cancel_nav_sync(timeout_sec=2.5)
                except Exception:
                    pass
                self._wait_nav_action_idle(
                    float(os.environ.get("PINKY_NAV_PRE_GOAL_IDLE_SEC", "3.0"))
                )
        else:
            # Soft wait only if something is clearly executing.
            with self._lock:
                pre_state = (self._navigation_action_state or "").upper()
            if pre_state in ("ACCEPTED", "EXECUTING", "CANCELING"):
                self._wait_nav_action_idle(
                    float(os.environ.get("PINKY_NAV_PRE_GOAL_IDLE_SEC", "5.0"))
                )
            with self._lock:
                self._navigation_action_state = "UNKNOWN"
                self._is_navigating = False

        if self._wait_usable_tf(1.5) is None:
            return {
                "success": False,
                "status": "NO_TF",
                "message": (
                    "no map→base_footprint/base_link TF after AMCL activate+initialpose — "
                    "check AMCL/lidar/map (Nav2 rejects goals without robot pose)"
                ),
            }

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            return {
                "success": False,
                "status": "UNAVAILABLE",
                "message": "navigate_to_pose Action Server not available",
            }

        # Idle-freeze may have deactivated AMCL after ensure — Nav2 then REJECTS.
        try:
            self._amcl_activate()
        except Exception as exc:
            if self._node:
                self._node.get_logger().warn(f"pre-goal AMCL activate: {exc}")
        settle_amcl = float(os.environ.get("PINKY_LOCALIZE_SETTLE_AFTER_FREEZE_SEC", "1.5"))
        if settle_amcl > 0:
            time.sleep(min(3.0, settle_amcl))
        if self._wait_usable_tf(2.0) is None:
            # One more ensure if TF still missing after activate.
            try:
                self._ensure_localization_for_drive(x, y, yaw)
            except Exception:
                pass
            if self._wait_usable_tf(2.0) is None:
                return {
                    "success": False,
                    "status": "NO_TF",
                    "message": (
                        "no map→base TF after AMCL activate — "
                        "set initialpose / check lidar+AMCL"
                    ),
                }

        sid = self._start_nav_session()

        def _finish(payload: dict[str, Any]) -> dict[str, Any]:
            if payload.get("success"):
                self._on_nav_goal_succeeded(x, y, yaw)
            elif payload.get("status") in ("CANCELED", "INTERRUPTED"):
                self._clear_tour_lock("nav interrupted")
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

        max_accept = max(1, int(os.environ.get("PINKY_NAV_ACCEPT_RETRIES", "5")))
        goal_handle = None
        for attempt in range(1, max_accept + 1):
            if self.motion_interrupted(motion_epoch):
                return _finish(
                    {
                        "success": False,
                        "status": "INTERRUPTED",
                        "message": "nav interrupted by stop/new job",
                    }
                )
            if time.time() > deadline:
                break
            send_future = self._nav_client.send_goal_async(_build_goal())
            goal_handle = _wait_accept(send_future)
            if goal_handle is not None and getattr(goal_handle, "accepted", False):
                break
            amcl_lbl = self._amcl_get_state_label()
            tf_ok = self._lookup_tf_pose() is not None
            with self._lock:
                action_state = self._navigation_action_state
            if self._node:
                self._node.get_logger().warn(
                    f"NavigateToPose rejected try {attempt}/{max_accept} "
                    f"action_state={action_state} amcl={amcl_lbl} tf={tf_ok}"
                )
            # Re-activate AMCL between rejects (idle freeze / lifecycle flake).
            try:
                self._amcl_activate()
            except Exception:
                pass
            settle = float(os.environ.get("PINKY_NAV_REJECT_SETTLE_SEC", "2.0"))
            time.sleep(max(0.5, settle) + 0.25 * attempt)
            try:
                self._nav_client.wait_for_server(timeout_sec=3.0)
            except Exception:
                pass
            with self._lock:
                self._navigation_action_state = "UNKNOWN"
                self._is_navigating = False
            goal_handle = None

        if goal_handle is None or not getattr(goal_handle, "accepted", False):
            with self._lock:
                action_state = self._navigation_action_state
            amcl_lbl = self._amcl_get_state_label()
            tf_ok = self._lookup_tf_pose() is not None
            return _finish(
                {
                    "success": False,
                    "status": "REJECTED",
                    "message": (
                        f"goal rejected (Nav2) action_state={action_state} "
                        f"amcl={amcl_lbl} tf={tf_ok}. "
                        "If amcl!=active: set PINKY_AMCL_IDLE_FREEZE=0 or check initialpose. "
                        "If duplicate bt_navigator WARNING: pkill nav2 and restart run.py once"
                    ),
                }
            )

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            if self.motion_interrupted(motion_epoch):
                try:
                    self._cancel_nav_sync(timeout_sec=1.5)
                except Exception:
                    pass
                return _finish(
                    {
                        "success": False,
                        "status": "INTERRUPTED",
                        "message": "nav interrupted by stop/new job",
                    }
                )
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
        status_name = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
        }.get(status, f"STATUS_{status}")
        action_result = getattr(wrapped, "result", None)
        error_code = int(getattr(action_result, "error_code", 0) or 0)
        error_msg = str(getattr(action_result, "error_msg", "") or "")
        if ok:
            message = "arrived"
        else:
            message = f"nav ended with status {status} ({status_name})"
            if error_code or error_msg:
                extra = " ".join(
                    part
                    for part in (f"error_code={error_code}" if error_code else "", error_msg)
                    if part
                )
                message = f"{message} {extra}".strip()

        pose_after = self._lookup_tf_pose()
        if pose_after is None:
            with self._lock:
                pose_after = self._nav_pose
        dist_after = None
        yaw_err_after = None
        if pose_after is not None:
            dist_after = math.hypot(
                float(pose_after[0]) - float(x), float(pose_after[1]) - float(y)
            )
            yaw_err_after = abs(self._angle_error(float(pose_after[2]), float(yaw)))
        # Nav2 often SUCCEEDED/ABORTED at the same XY without rotating.
        if (
            dist_after is not None
            and yaw_err_after is not None
            and dist_after <= arrive_m * 1.5
            and yaw_err_after > arrive_yaw
        ):
            if self._node:
                self._node.get_logger().info(
                    f"Nav2 {status_name} at XY but yaw err={yaw_err_after:.3f}rad "
                    "— rotate in place"
                )
            self._end_nav_session(sid)
            return self._rotate_in_place_to_yaw(
                x,
                y,
                yaw,
                timeout_sec=max(3.0, deadline - time.time()),
                arrive_yaw=arrive_yaw,
            )

        return _finish(
            {
                "success": ok,
                "status": status_name,
                "message": message,
                "errorCode": error_code,
                "errorMsg": error_msg,
                "goal": {"x": x, "y": y, "yaw": yaw},
            }
        )

    def _nav_action_busy(self) -> bool:
        with self._lock:
            state = (self._navigation_action_state or "").upper()
        return state in ("ACCEPTED", "EXECUTING", "CANCELING")

    def _wait_nav_action_idle(self, timeout_sec: float = 3.0) -> bool:
        """Wait until bt_navigator is ready to accept a new NavigateToPose."""
        deadline = time.time() + max(0.2, float(timeout_sec))
        while time.time() < deadline:
            if not self._nav_action_busy() and not self.is_navigating():
                # Double-check after a short settle — CANCELING often flashes UNKNOWN.
                time.sleep(0.15)
                if not self._nav_action_busy() and not self.is_navigating():
                    return True
            time.sleep(0.05)
        return not self._nav_action_busy() and not self.is_navigating()

    def _cancel_nav_sync(self, timeout_sec: float = 2.0) -> dict[str, Any]:
        """Cancel NavigateToPose and wait until the action is no longer CANCELING."""
        if not self._nav_enabled or self._cancel_client is None:
            self._end_nav_session()
            return {"success": False, "message": "navigation not enabled"}
        from action_msgs.srv import CancelGoal

        settle = float(os.environ.get("PINKY_NAV_CANCEL_SETTLE_SEC", "0.6"))
        idle_wait = float(os.environ.get("PINKY_NAV_CANCEL_IDLE_SEC", "4.0"))

        # Already idle — avoid cancel storms that leave bt_navigator rejecting goals.
        if not self._nav_action_busy() and not self.is_navigating():
            self._end_nav_session()
            return {"success": True, "message": "already idle"}

        if not self._cancel_client.wait_for_service(timeout_sec=1.0):
            self._end_nav_session()
            return {"success": False, "message": "cancel service not available"}

        with self._lock:
            self._navigation_action_state = "CANCELING"

        req = CancelGoal.Request()
        fut = self._cancel_client.call_async(req)
        deadline = time.time() + max(0.2, float(timeout_sec))
        while not fut.done() and time.time() < deadline:
            time.sleep(0.05)
        self._end_nav_session()
        # Give Nav2 time to leave CANCELING before a new goal (REJECT race).
        time.sleep(max(0.15, settle))
        self._wait_nav_action_idle(idle_wait)
        with self._lock:
            if (self._navigation_action_state or "").upper() == "CANCELING":
                # Status topic stuck; clear so we can retry send.
                self._navigation_action_state = "CANCELED"
                self._is_navigating = False
        return {"success": True, "message": "cancel requested"}

    def cancel_navigation(self, *, freeze: bool = True) -> dict[str, Any]:
        self._bump_motion_epoch()
        try:
            self.end_visual_dock_hold()
        except Exception:
            pass
        try:
            self.drive(0.0, 0.0)
        except Exception:
            pass
        if not self._nav_enabled or self._cancel_client is None:
            self._end_nav_session()
            if freeze:
                self._clear_tour_lock("nav stop")
                try:
                    self._freeze_localization_idle()
                except Exception:
                    pass
            return {"success": False, "message": "navigation not enabled"}
        result = self._cancel_nav_sync(timeout_sec=2.0)
        if freeze:
            self._clear_tour_lock("nav stop")
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

    def _on_odom(self, msg) -> None:
        pose = msg.pose.pose
        yaw = self._quat_to_yaw(pose.orientation)
        with self._lock:
            self._odom_pose = (
                float(pose.position.x),
                float(pose.position.y),
                float(yaw),
            )
            self._odom_twist = (
                float(msg.twist.twist.linear.x),
                float(msg.twist.twist.angular.z),
            )
            self._odom_stamp = time.time()

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
        emo = str(emotion or "").strip()
        if not emo:
            return {"success": False, "message": "empty emotion"}
        # EMOTIONS is the documented list; still forward unknown names so newly
        # installed GIFs work before pinky whitelist is redeployed.
        if self._set_emotion_cli is None:
            return {"success": False, "message": "set_emotion client unavailable"}
        from pinky_interfaces.srv import Emotion

        req = Emotion.Request()
        req.emotion = emo
        result = self._call_service(self._set_emotion_cli, req, timeout=3.0)
        msg = str(result.get("message") or "")
        low = msg.lower()
        if (
            "wrong" in low
            or "not found" in low
            or "not cached" in low
            or "unknown" in low
        ):
            result = {**result, "success": False}
        elif emo not in EMOTIONS and result.get("success"):
            # Accepted by ROS but not yet listed locally — still OK.
            result = {**result, "emotion": emo}
        else:
            result = {**result, "emotion": emo}
        return result

    @staticmethod
    def _angle_error(current: float, reference: float) -> float:
        return math.atan2(math.sin(current - reference), math.cos(current - reference))

    def _rotate_in_place_to_yaw(
        self,
        x: float,
        y: float,
        yaw: float,
        timeout_sec: float,
        arrive_yaw: float,
    ) -> dict[str, Any]:
        """Turn in place when XY is already at the goal (W6→C). Nav2 skips this."""
        self._remember_nav_goal(x, y, yaw)
        try:
            self._cancel_nav_sync(timeout_sec=1.5)
        except Exception:
            pass
        self._wait_nav_action_idle(2.0)

        if not self._motion_lock.acquire(blocking=False):
            return {
                "success": False,
                "status": "BUSY",
                "message": "another relative motion is active",
                "goal": {"x": x, "y": y, "yaw": yaw},
            }

        sid = self._start_nav_session()
        max_rate = float(os.environ.get("PINKY_INPLACE_YAW_RATE", "0.45"))
        min_rate = float(os.environ.get("PINKY_INPLACE_YAW_MIN_RATE", "0.18"))
        deadline = time.time() + max(3.0, float(timeout_sec))
        start_map = self._lookup_tf_pose()
        if start_map is None:
            with self._lock:
                start_map = self._nav_pose
        with self._lock:
            start_odom = self._odom_pose

        def _current_pose() -> tuple[float, float, float] | None:
            # Close the loop on odom — AMCL/TF often does not update during a
            # same-XY spin, so TF-only control would keep turning past the goal.
            with self._lock:
                odom = self._odom_pose
            if start_map is not None and start_odom is not None and odom is not None:
                dyaw = self._angle_error(float(odom[2]), float(start_odom[2]))
                heading = float(start_map[2]) + dyaw
                heading = math.atan2(math.sin(heading), math.cos(heading))
                return (float(start_map[0]), float(start_map[1]), heading)
            tf_pose = self._lookup_tf_pose()
            if tf_pose is not None and not self._spurious_home_tf(tf_pose):
                return tf_pose
            with self._lock:
                return self._nav_pose

        last_err = None
        ok = False
        try:
            while time.time() < deadline:
                pose = _current_pose()
                if pose is None:
                    time.sleep(0.05)
                    continue
                err = self._angle_error(float(yaw), float(pose[2]))
                last_err = err
                with self._lock:
                    self._nav_pose = (float(pose[0]), float(pose[1]), float(pose[2]))
                if abs(err) <= arrive_yaw:
                    ok = True
                    break
                rate = max(-max_rate, min(max_rate, err * 1.6))
                if abs(rate) < min_rate:
                    rate = min_rate if rate >= 0.0 else -min_rate
                self.drive(0.0, rate, bypass_collision=True)
                time.sleep(0.05)
            self.drive(0.0, 0.0, bypass_collision=True)
            time.sleep(0.12)
            pose = _current_pose()
            if pose is not None:
                last_err = self._angle_error(float(yaw), float(pose[2]))
                if abs(last_err) <= arrive_yaw * 1.25:
                    ok = True
                    with self._lock:
                        self._nav_pose = (
                            float(pose[0]),
                            float(pose[1]),
                            float(pose[2]),
                        )
        finally:
            try:
                self.drive(0.0, 0.0, bypass_collision=True)
            except Exception:
                pass
            self._end_nav_session(sid)
            self._motion_lock.release()

        if ok:
            pose = _current_pose()
            if pose is None:
                pose = (float(x), float(y), float(yaw))
            with self._lock:
                self._nav_pose = pose
                self._last_good_map_pose = pose
                self._last_waypoint_pose = pose
            self._lock_out_home_seed("in-place yaw aligned")
            err_abs = 0.0 if last_err is None else abs(last_err)
            if self._node:
                self._node.get_logger().info(
                    f"in-place yaw aligned err={err_abs:.3f}rad "
                    f"goal=({x:.3f},{y:.3f},{yaw:.3f})"
                )
            return {
                "success": True,
                "status": "SUCCEEDED",
                "message": f"rotated in place yawErr={err_abs:.3f}rad",
                "goal": {"x": x, "y": y, "yaw": yaw},
            }
        err_txt = "unknown" if last_err is None else f"{abs(last_err):.3f}rad"
        if self._node:
            self._node.get_logger().warn(
                f"in-place yaw timeout remaining={err_txt} "
                f"goal=({x:.3f},{y:.3f},{yaw:.3f})"
            )
        return {
            "success": False,
            "status": "TIMEOUT",
            "message": f"in-place yaw timeout remaining={err_txt}",
            "goal": {"x": x, "y": y, "yaw": yaw},
        }

    def _fresh_navigation_readiness(self) -> dict[str, Any]:
        failures: list[str] = []
        tf_valid = self._lookup_tf_pose() is not None
        if not tf_valid:
            failures.append("no map→base TF")
        with self._lock:
            scan = self._scan
        scan_fresh = False
        if scan is not None and scan.stamp is not None:
            scan_fresh = (time.time() - float(scan.stamp)) <= 1.0
        if not scan_fresh:
            failures.append("scan stale or unavailable")
        ready = bool(self._nav_enabled and tf_valid and scan_fresh)
        return {
            "ready": ready,
            "tfValid": tf_valid,
            "scanFresh": scan_fresh,
            "failures": failures,
        }

    def _publish_zero_velocity(self, repeats: int = 4, gap_sec: float = 0.04) -> None:
        for _ in range(max(1, int(repeats))):
            try:
                self.drive(0.0, 0.0)
            except Exception:
                pass
            if gap_sec > 0.0:
                time.sleep(gap_sec)

    def _wait_for_odom_stop(
        self, timeout_sec: float = 0.8, linear_tol: float = 0.01, angular_tol: float = 0.08
    ) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while time.monotonic() < deadline:
            with self._lock:
                twist = self._odom_twist
                stamp = self._odom_stamp
            if twist is not None and stamp is not None and time.time() - float(stamp) <= 0.5:
                if abs(float(twist[0])) <= linear_tol and abs(float(twist[1])) <= angular_tol:
                    return True
            time.sleep(0.04)
        return False

    def _scan_clearance_base_x(
        self,
        direction: float,
        corridor_half_width_m: float = 0.07,
    ) -> tuple[float | None, str | None]:
        """Nearest obstacle distance from base origin along +/- base X corridor."""
        with self._lock:
            scan = self._scan
        if scan is None or scan.stamp is None:
            return None, "scan unavailable"
        age = time.time() - float(scan.stamp)
        if age > 1.0:
            return None, f"scan stale ({age:.2f}s)"
        if self._tf_buffer is None or self._Time is None:
            return None, "tf unavailable"
        try:
            trans = self._tf_buffer.lookup_transform(
                "base_footprint", scan.frame_id or "rplidar_link", self._Time()
            )
        except Exception as exc:
            return None, f"scan TF unavailable: {exc}"
        t = trans.transform.translation
        yaw = self._quat_to_yaw(trans.transform.rotation)
        cy, sy = math.cos(yaw), math.sin(yaw)
        sign = 1.0 if direction >= 0.0 else -1.0
        nearest = math.inf
        for i, raw_range in enumerate(scan.ranges):
            r = float(raw_range)
            if not math.isfinite(r) or r <= 0.0:
                continue
            if scan.range_min and r < float(scan.range_min):
                continue
            if scan.range_max and r > float(scan.range_max):
                continue
            angle = float(scan.angle_min) + i * float(scan.angle_increment)
            sx = r * math.cos(angle)
            sy_scan = r * math.sin(angle)
            bx = float(t.x) + cy * sx - sy * sy_scan
            by = float(t.y) + sy * sx + cy * sy_scan
            forward = sign * bx
            if forward <= 0.0 or abs(by) > float(corridor_half_width_m):
                continue
            nearest = min(nearest, forward)
        return (nearest if math.isfinite(nearest) else math.inf), None

    def _open_loop_translate(
        self,
        distance_m: float,
        speed_mps: float,
        timeout_sec: float | None,
        *,
        bypass_collision: bool = True,
        hold_lock: bool = False,
    ) -> dict[str, Any]:
        """Timed cmd_vel translate when odom is missing (abort undock)."""
        distance = float(distance_m)
        speed = abs(float(speed_mps))
        if abs(distance) < 1e-4 or speed <= 0.0:
            return {"success": True, "movedM": 0.0, "message": "open-loop none"}
        sign = 1.0 if distance > 0.0 else -1.0
        duration = abs(distance) / max(speed, 0.01)
        if timeout_sec is not None:
            duration = min(duration, max(0.2, float(timeout_sec)))
        got_lock = False
        if not hold_lock:
            got_lock = bool(self._motion_lock.acquire(timeout=2.5))
            if not got_lock:
                return {
                    "success": False,
                    "movedM": 0.0,
                    "message": "another relative motion is active",
                }
        start_epoch = self._motion_epoch_now()
        t0 = time.monotonic()
        try:
            while time.monotonic() - t0 < duration:
                if self.motion_interrupted(start_epoch):
                    break
                self.drive(sign * speed, 0.0, bypass_collision=bypass_collision)
                time.sleep(0.05)
        finally:
            try:
                self.drive(0.0, 0.0, bypass_collision=bypass_collision)
            except Exception:
                pass
            if got_lock:
                self._motion_lock.release()
        moved = speed * min(duration, max(0.0, time.monotonic() - t0))
        return {
            "success": True,
            "movedM": moved,
            "message": f"open-loop translate {moved:.3f}m",
        }

    def relative_move(
        self,
        distance_m: float,
        speed_mps: float = 0.02,
        timeout_sec: float | None = None,
        *,
        dry_run: bool = False,
        bypass_collision: bool = False,
        ignore_scan: bool = False,
    ) -> dict[str, Any]:
        """Closed-loop short translation using odom.

        Undock (reverse out of a wall pocket) should pass bypass_collision and
        ignore_scan — inflation/side walls otherwise freeze the robot in place.
        """
        distance = float(distance_m)
        speed = abs(float(speed_mps))
        if abs(distance) < 1e-4:
            return {"success": True, "message": "relative move: no movement", "movedM": 0.0}
        if speed <= 0.0 or speed > 0.08:
            return {"success": False, "message": "speedMps must be > 0 and <= 0.08"}
        if abs(distance) > 0.30:
            return {"success": False, "message": "relative move limited to 0.30m per call"}
        if self.is_navigating() and not bypass_collision:
            return {"success": False, "message": "navigation is active; relative move refused"}

        if not ignore_scan:
            readiness = self._fresh_navigation_readiness()
            if (
                readiness.get("ready") is not True
                or readiness.get("tfValid") is not True
                or readiness.get("scanFresh") is not True
            ):
                return {
                    "success": False,
                    "message": "navigation/TF/scan precheck failed",
                    "readiness": readiness,
                }

        with self._lock:
            odom = self._odom_pose
            odom_stamp = self._odom_stamp
        odom_age = time.time() - float(odom_stamp) if odom_stamp is not None else math.inf
        odom_limit = 3.0 if ignore_scan else 0.5
        if odom is None or odom_age > odom_limit:
            if ignore_scan and bypass_collision:
                if self._node:
                    self._node.get_logger().warn(
                        f"relative move odom stale ({odom_age:.2f}s) — open-loop "
                        f"{distance:.3f}m"
                    )
                return self._open_loop_translate(
                    distance, speed, timeout_sec, bypass_collision=True
                )
            return {
                "success": False,
                "message": f"odom unavailable/stale ({odom_age:.2f}s)",
                "movedM": 0.0,
            }

        sign = 1.0 if distance > 0.0 else -1.0
        corridor_half_width = 0.07
        robot_half_length = 0.06
        safety_margin = 0.03
        clearance: float | None = None
        required_initial = robot_half_length + abs(distance) + safety_margin
        if not ignore_scan:
            clearance, clearance_error = self._scan_clearance_base_x(
                sign, corridor_half_width
            )
            if clearance_error is not None:
                return {"success": False, "message": clearance_error}
            if (
                clearance is not None
                and math.isfinite(clearance)
                and clearance < required_initial
            ):
                shrink = float(clearance) - robot_half_length - safety_margin
                if shrink < 0.01:
                    return {
                        "success": False,
                        "message": "relative move blocked by scan",
                        "clearanceM": clearance,
                        "requiredClearanceM": required_initial,
                    }
                distance = sign * shrink

        if dry_run:
            return {
                "success": True,
                "dryRun": True,
                "message": "relative move safety precheck ok; no cmd_vel sent",
                "requestedM": distance,
                "speedMps": speed,
                "odomAgeSec": odom_age,
                "clearanceM": clearance,
                "requiredClearanceM": required_initial,
            }

        if bypass_collision:
            try:
                self._cancel_nav_sync(timeout_sec=1.0)
            except Exception:
                pass
            self._end_nav_session()

        if not self._motion_lock.acquire(timeout=2.5):
            return {
                "success": False,
                "message": "another relative motion is active",
                "movedM": 0.0,
            }

        lateral_lim = 0.08 if ignore_scan else 0.035
        yaw_lim = 0.35 if ignore_scan else 0.18
        start_x, start_y, start_yaw = odom
        deadline = time.monotonic() + (
            float(timeout_sec)
            if timeout_sec is not None
            else max(2.5, abs(distance) / speed + 2.0)
        )
        last_progress = 0.0
        result: dict[str, Any] = {
            "success": False,
            "message": "relative move ended unexpectedly",
            "movedM": 0.0,
        }
        stopped = False
        move_epoch = self._motion_epoch_now()
        try:
            while True:
                if self.motion_interrupted(move_epoch):
                    result = {
                        "success": False,
                        "message": "relative move interrupted",
                        "movedM": max(0.0, last_progress),
                    }
                    break
                if time.monotonic() >= deadline:
                    result = {
                        "success": False,
                        "message": "relative move timeout",
                        "movedM": max(0.0, last_progress),
                    }
                    break
                if self.is_navigating() and not bypass_collision:
                    result = {
                        "success": False,
                        "message": "navigation became active during relative move",
                        "movedM": max(0.0, last_progress),
                    }
                    break
                with self._lock:
                    cur = self._odom_pose
                    cur_stamp = self._odom_stamp
                    visual_dock_active = self._visual_dock_active
                if visual_dock_active and not bypass_collision:
                    result = {
                        "success": False,
                        "message": "visual docking became active during relative move",
                        "movedM": max(0.0, last_progress),
                    }
                    break
                cur_age = (
                    time.time() - float(cur_stamp) if cur_stamp is not None else math.inf
                )
                if cur is None or cur_age > odom_limit:
                    if ignore_scan:
                        remain = max(0.0, abs(distance) - last_progress)
                        extra = self._open_loop_translate(
                            sign * remain,
                            speed,
                            remain / max(speed, 0.01) + 1.0,
                            bypass_collision=bypass_collision,
                            hold_lock=True,
                        )
                        moved = last_progress + abs(float(extra.get("movedM") or 0.0))
                        result = {
                            "success": bool(extra.get("success")),
                            "message": (
                                f"odom stale mid-move; open-loop {extra.get('message')}"
                            ),
                            "movedM": moved,
                        }
                        break
                    result = {
                        "success": False,
                        "message": f"odom became stale ({cur_age:.2f}s)",
                        "movedM": max(0.0, last_progress),
                    }
                    break
                dx, dy = cur[0] - start_x, cur[1] - start_y
                forward = math.cos(start_yaw) * dx + math.sin(start_yaw) * dy
                lateral = -math.sin(start_yaw) * dx + math.cos(start_yaw) * dy
                progress = sign * forward
                last_progress = progress
                yaw_drift = abs(self._angle_error(cur[2], start_yaw))
                if abs(lateral) > lateral_lim:
                    result = {
                        "success": False,
                        "message": f"relative move lateral drift {lateral:.3f}m",
                        "movedM": max(0.0, progress),
                    }
                    break
                if yaw_drift > yaw_lim:
                    result = {
                        "success": False,
                        "message": f"relative move yaw drift {yaw_drift:.3f}rad",
                        "movedM": max(0.0, progress),
                    }
                    break
                if progress >= abs(distance) - 0.003:
                    result = {
                        "success": True,
                        "message": "relative move complete",
                        "requestedM": distance,
                        "movedM": max(0.0, progress),
                        "lateralDriftM": lateral,
                        "yawDriftRad": yaw_drift,
                    }
                    break

                if not ignore_scan:
                    remaining = max(0.0, abs(distance) - progress)
                    clearance, clearance_error = self._scan_clearance_base_x(
                        sign, corridor_half_width
                    )
                    if clearance_error is not None:
                        result = {
                            "success": False,
                            "message": clearance_error,
                            "movedM": max(0.0, progress),
                        }
                        break
                    required = robot_half_length + remaining + safety_margin
                    if clearance is not None and clearance < required:
                        result = {
                            "success": False,
                            "message": "relative move obstacle detected",
                            "movedM": max(0.0, progress),
                            "clearanceM": clearance,
                            "requiredClearanceM": required,
                        }
                        break
                self.drive(sign * speed, 0.0, bypass_collision=bypass_collision)
                time.sleep(0.05)
        finally:
            try:
                self.drive(0.0, 0.0, bypass_collision=bypass_collision)
            except Exception:
                pass
            self._publish_zero_velocity()
            stopped = self._wait_for_odom_stop()
            self._motion_lock.release()

        result["stopped"] = bool(stopped)
        if result.get("success") and not stopped and not bypass_collision:
            result["success"] = False
            result["message"] = "relative move reached distance but odom stop was not confirmed"
        return result

    def drive(
        self,
        linear_x: float,
        angular_z: float,
        *,
        bypass_collision: bool = False,
    ) -> dict[str, Any]:
        pub = self._cmd_vel_pub
        if bypass_collision and self._cmd_vel_aruco_pub is not None:
            pub = self._cmd_vel_aruco_pub
        if pub is None:
            return {"success": False, "message": "cmd_vel publisher unavailable"}
        from geometry_msgs.msg import Twist

        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        pub.publish(msg)
        return {
            "success": True,
            "message": "cmd_vel published",
            "cmdVel": {"linearX": linear_x, "angularZ": angular_z},
        }
