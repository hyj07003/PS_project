from __future__ import annotations

import math
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

        ros2_runtime.add_node(self._node)
        self._started = True

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
