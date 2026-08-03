from __future__ import annotations

import math
import os
from typing import Any

from .hardware import HardwareSensors
from .lidar import LidarReader


class SensorPublisherController:
    """
    pinky_pro 호환 센서 토픽을 발행하는 ROS2 컨트롤러.

    Publishes:
      battery/percent, battery/voltage
      imu_raw
      us_sensor/range, ir_sensor/range
      scan (LaserScan)  ← RPLidar /dev/ttyAMA0
    """

    def __init__(self) -> None:
        self._node: Any = None
        self._hw = HardwareSensors()
        self._lidar = LidarReader.shared()
        self._started = False
        self._fallback = os.environ.get("PINKY_SENSOR_FALLBACK", "0") in (
            "1",
            "true",
            "True",
        )

    @property
    def hardware_status(self) -> list[str]:
        return self._hw.status + self._lidar.status

    @property
    def lidar(self) -> LidarReader:
        return self._lidar

    def start(self) -> None:
        if self._started:
            return
        from modules.backends import ros2_runtime

        ros2_runtime.ensure_runtime()
        from rclpy.node import Node
        from sensor_msgs.msg import Imu, LaserScan, Range
        from std_msgs.msg import Float32, UInt16MultiArray

        class _PublisherNode(Node):
            pass

        self._lidar.start()

        self._node = _PublisherNode("pinky_sensor_publisher")
        self._batt_pct_pub = self._node.create_publisher(Float32, "battery/percent", 10)
        self._batt_v_pub = self._node.create_publisher(Float32, "battery/voltage", 10)
        self._imu_pub = self._node.create_publisher(Imu, "imu_raw", 10)
        self._us_pub = self._node.create_publisher(Range, "us_sensor/range", 10)
        self._ir_pub = self._node.create_publisher(UInt16MultiArray, "ir_sensor/range", 10)
        self._scan_pub = self._node.create_publisher(LaserScan, "scan", 10)

        batt_hz = float(os.environ.get("PINKY_BATT_HZ", "0.5"))
        imu_hz = float(os.environ.get("PINKY_IMU_HZ", "20"))
        us_hz = float(os.environ.get("PINKY_US_HZ", "10"))
        lidar_hz = float(os.environ.get("PINKY_LIDAR_HZ", "5"))

        self._node.create_timer(1.0 / max(batt_hz, 0.1), self._tick_battery)
        self._node.create_timer(1.0 / max(imu_hz, 1.0), self._tick_imu)
        self._node.create_timer(1.0 / max(us_hz, 1.0), self._tick_ultrasonic)
        self._node.create_timer(1.0 / max(lidar_hz, 1.0), self._tick_lidar)

        ros2_runtime.add_node(self._node)
        self._started = True
        self._node.get_logger().info(
            "pinky_sensor_publisher started | " + " ; ".join(self.hardware_status)
        )

    def stop(self) -> None:
        if not self._started:
            return
        from modules.backends import ros2_runtime

        if self._node is not None:
            ros2_runtime.remove_node(self._node)
            self._node = None
        self._lidar.stop()
        self._started = False

    def _tick_battery(self) -> None:
        from std_msgs.msg import Float32

        reading = self._hw.read_battery()
        if reading.voltage is None and reading.percent is None:
            if not self._fallback:
                return
            reading.voltage = 7.4
            reading.percent = 50.0
        if reading.percent is not None:
            msg = Float32()
            msg.data = float(reading.percent)
            self._batt_pct_pub.publish(msg)
        if reading.voltage is not None:
            msg = Float32()
            msg.data = float(reading.voltage)
            self._batt_v_pub.publish(msg)

    def _tick_imu(self) -> None:
        from sensor_msgs.msg import Imu

        reading = self._hw.read_imu()
        if reading is None:
            if not self._fallback:
                return
            ox, oy, oz, ow = (0.0, 0.0, 0.0, 1.0)
            av = (0.0, 0.0, 0.0)
            la = (0.0, 0.0, 9.81)
        else:
            ox, oy, oz, ow = reading.orientation
            av = reading.angular_velocity
            la = reading.linear_acceleration

        msg = Imu()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"
        msg.orientation.x = float(ox)
        msg.orientation.y = float(oy)
        msg.orientation.z = float(oz)
        msg.orientation.w = float(ow)
        msg.angular_velocity.x = float(av[0])
        msg.angular_velocity.y = float(av[1])
        msg.angular_velocity.z = float(av[2])
        msg.linear_acceleration.x = float(la[0])
        msg.linear_acceleration.y = float(la[1])
        msg.linear_acceleration.z = float(la[2])
        self._imu_pub.publish(msg)

    def _tick_ultrasonic(self) -> None:
        from sensor_msgs.msg import Range
        from std_msgs.msg import UInt16MultiArray

        reading = self._hw.read_ultrasonic()
        if reading.range_m is None and not reading.ir_raw:
            if not self._fallback:
                return
            reading.range_m = 0.5
            reading.ir_raw = [0, 0, 0]

        if reading.range_m is not None:
            msg = Range()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.header.frame_id = "ultrasonic_link"
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = 0.26
            msg.min_range = 0.02
            msg.max_range = 3.0
            msg.range = float(reading.range_m)
            self._us_pub.publish(msg)

        if reading.ir_raw:
            ir = UInt16MultiArray()
            ir.data = [int(x) for x in reading.ir_raw]
            self._ir_pub.publish(ir)

    def _tick_lidar(self) -> None:
        from sensor_msgs.msg import LaserScan

        # sllidar가 직접 /scan 을 발행 중이면 중복 발행하지 않음
        if getattr(self._lidar, "external_sllidar", False):
            return

        scan = self._lidar.read()
        if scan is None or not scan.ranges:
            return

        msg = LaserScan()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = scan.frame_id
        msg.angle_min = float(scan.angle_min)
        msg.angle_max = float(scan.angle_max)
        msg.angle_increment = float(scan.angle_increment) or (
            2 * math.pi / max(len(scan.ranges), 1)
        )
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = float(scan.range_min)
        msg.range_max = float(scan.range_max)
        msg.ranges = [float(r) if r > 0 else float("nan") for r in scan.ranges]
        self._scan_pub.publish(msg)
