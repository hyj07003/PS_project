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
        self._is_navigating = False
        self._nav_client = None
        self._plan_client = None
        self._nav_path: dict[str, Any] | None = None
        self._initial_pose_pub = None
        self._cancel_client = None
        self._tf_buffer = None
        self._tf_listener = None
        self._nav_enabled = True

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
            return
        self._nav_enabled = True
        try:
            from geometry_msgs.msg import PoseWithCovarianceStamped
            from nav_msgs.msg import Path
            from nav2_msgs.action import ComputePathToPose, NavigateToPose
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
            return

        self._GoalStatus = GoalStatus
        self._Time = Time

        self._nav_client = ActionClient(self._node, NavigateToPose, "navigate_to_pose")
        self._plan_client = ActionClient(
            self._node, ComputePathToPose, "compute_path_to_pose"
        )
        self._node.create_subscription(Path, "plan", self._on_plan, 10)
        # AMCL subscribes with transient_local; match so late/missed msgs still apply
        initialpose_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._initial_pose_pub = self._node.create_publisher(
            PoseWithCovarianceStamped, "initialpose", initialpose_qos
        )
        self._cancel_client = self._node.create_client(
            CancelGoal, "navigate_to_pose/_action/cancel_goal"
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

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self._node, spin_thread=False
        )
        self._node.create_timer(0.1, self._update_pose_from_tf)
        self._node.get_logger().info("Navigation bridge (Nav2 + TF map→base_link) ready")
        # Apply S1/S2 (or PINKY_INITIAL_POSE) after AMCL is up unless manual RViz
        # localization is being used for a test session.
        auto_initial = os.environ.get("PINKY_AUTO_INITIAL_POSE", "1").lower().strip()
        if auto_initial not in ("0", "false", "off", "no"):
            delay = float(os.environ.get("PINKY_INITIAL_POSE_DELAY_SEC", "2.0"))
            if delay <= 0:
                self._apply_boot_initial_pose()
            else:
                threading.Timer(delay, self._apply_boot_initial_pose).start()
        elif self._node:
            self._node.get_logger().info(
                "PINKY_AUTO_INITIAL_POSE=0: waiting for manual /initialpose (RViz 2D Pose Estimate)"
            )

    def _default_home_pose(self) -> tuple[float, float, float]:
        """cart-1→S1, cart-2→S2 (controller waypoints.py 와 동일)."""
        raw = (os.environ.get("PINKY_INITIAL_POSE") or "").strip()
        if raw:
            parts = [p.strip() for p in raw.split(",")]
            if len(parts) >= 2:
                return (
                    float(parts[0]),
                    float(parts[1]),
                    float(parts[2]) if len(parts) > 2 else 0.0,
                )
        code = (os.environ.get("PINKY_DEVICE_CODE") or "cart-1").strip().lower()
        if code in ("cart-2", "cart2", "2"):
            return (0.04742698442363813, -0.20226078567130157, -0.004704072590981645)
        return (0.009931882239292611, 0.021114122581406713, 0.01045265830576832)

    def _apply_boot_initial_pose(self) -> None:
        if not self._nav_enabled:
            return
        try:
            x, y, yaw = self._default_home_pose()
            result = self.set_initial_pose(x, y, yaw)
            if self._node:
                self._node.get_logger().info(
                    f"boot initialpose → ({x:.4f},{y:.4f},yaw={yaw:.3f}) "
                    f"ok={result.get('success')}"
                )
        except Exception as exc:
            if self._node:
                self._node.get_logger().warn(f"boot initialpose failed: {exc}")

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
            self._is_navigating = navigating

    def _update_pose_from_tf(self) -> None:
        if self._tf_buffer is None:
            return
        try:
            trans = self._tf_buffer.lookup_transform(
                "map", "base_link", self._Time()
            )
            t = trans.transform
            yaw = self._quat_to_yaw(t.rotation)
            with self._lock:
                self._nav_pose = (t.translation.x, t.translation.y, yaw)
        except Exception:
            pass

    def _path_to_dict(self, msg) -> dict[str, Any]:
        poses = [
            {
                "x": float(p.pose.position.x),
                "y": float(p.pose.position.y),
            }
            for p in msg.poses
        ]
        return {
            "frameId": str(msg.header.frame_id or "map"),
            "count": len(poses),
            "poses": poses,
        }

    def _on_plan(self, msg) -> None:
        path = self._path_to_dict(msg)
        with self._lock:
            self._nav_path = path

    def get_nav_path(self) -> dict[str, Any] | None:
        with self._lock:
            if self._nav_path is None:
                return None
            return {
                "frameId": self._nav_path.get("frameId", "map"),
                "count": int(self._nav_path.get("count", 0)),
                "poses": [dict(p) for p in self._nav_path.get("poses", [])],
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

        path = self._path_to_dict(wrapped.result.path)
        if not path["poses"]:
            return {"success": False, "message": "planner returned empty path"}
        with self._lock:
            self._nav_path = path
        return {
            "success": True,
            "message": "path computed without moving robot",
            "goal": {"x": x, "y": y, "yaw": yaw},
            "path": path,
        }

    def get_nav_pose(self) -> dict[str, float] | None:
        with self._lock:
            if self._nav_pose is None:
                return None
            x, y, yaw = self._nav_pose
            return {"x": x, "y": y, "yaw": yaw}

    def is_navigating(self) -> bool:
        with self._lock:
            return self._is_navigating

    def set_initial_pose(self, x: float, y: float, yaw: float = 0.0) -> dict[str, Any]:
        if not self._nav_enabled or self._initial_pose_pub is None:
            return {"success": False, "message": "navigation not enabled"}
        from geometry_msgs.msg import PoseWithCovarianceStamped

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        msg.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        # Modest covariance so AMCL accepts and can refine with lidar
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891909122467

        # AMCL often misses a single publish (startup / QoS). Repeat briefly.
        repeats = max(1, int(os.environ.get("PINKY_INITIAL_POSE_PUBLISHES", "5")))
        gap = float(os.environ.get("PINKY_INITIAL_POSE_GAP_SEC", "0.15"))
        for _ in range(repeats):
            msg.header.stamp = self._node.get_clock().now().to_msg()
            self._initial_pose_pub.publish(msg)
            if gap > 0:
                time.sleep(gap)

        with self._lock:
            self._nav_pose = (float(x), float(y), float(yaw))
        return {
            "success": True,
            "message": f"initialpose published x{repeats}",
            "pose": {"x": x, "y": y, "yaw": yaw},
        }

    def navigate_to(self, x: float, y: float, yaw: float = 0.0) -> dict[str, Any]:
        if not self._nav_enabled or self._nav_client is None:
            return {"success": False, "message": "navigation not enabled"}
        from nav2_msgs.action import NavigateToPose

        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            return {
                "success": False,
                "message": "navigate_to_pose Action Server not available (Nav2 기동 확인)",
            }

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        goal.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        self._nav_client.send_goal_async(goal)
        with self._lock:
            self._is_navigating = True
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

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        goal.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)

        send_future = self._nav_client.send_goal_async(goal)
        with self._lock:
            self._is_navigating = True

        deadline = time.time() + max(1.0, float(timeout_sec))
        while not send_future.done():
            if time.time() > deadline:
                with self._lock:
                    self._is_navigating = False
                return {
                    "success": False,
                    "status": "TIMEOUT",
                    "message": "goal accept timeout",
                }
            time.sleep(0.05)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            with self._lock:
                self._is_navigating = False
            return {
                "success": False,
                "status": "REJECTED",
                "message": "goal rejected",
            }

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            if time.time() > deadline:
                try:
                    self.cancel_navigation()
                except Exception:
                    pass
                with self._lock:
                    self._is_navigating = False
                return {
                    "success": False,
                    "status": "TIMEOUT",
                    "message": "nav result timeout",
                }
            time.sleep(0.05)

        wrapped = result_future.result()
        with self._lock:
            self._is_navigating = False
        if wrapped is None:
            return {
                "success": False,
                "status": "FAILED",
                "message": "no result",
            }
        status = int(wrapped.status)
        ok = status == GoalStatus.STATUS_SUCCEEDED
        return {
            "success": ok,
            "status": "SUCCEEDED" if ok else f"STATUS_{status}",
            "message": "arrived" if ok else f"nav ended with status {status}",
            "goal": {"x": x, "y": y, "yaw": yaw},
        }

    def cancel_navigation(self) -> dict[str, Any]:
        if not self._nav_enabled or self._cancel_client is None:
            return {"success": False, "message": "navigation not enabled"}
        from action_msgs.srv import CancelGoal

        if not self._cancel_client.wait_for_service(timeout_sec=1.0):
            return {"success": False, "message": "cancel service not available"}
        req = CancelGoal.Request()
        self._cancel_client.call_async(req)
        with self._lock:
            self._is_navigating = False
        return {"success": True, "message": "cancel requested"}

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
