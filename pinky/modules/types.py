from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any
import math


@dataclass
class BatteryData:
    percent: float | None = None
    voltage: float | None = None
    source: str = "unknown"

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class LidarData:
    ranges: list[float] = field(default_factory=list)
    angle_min: float = 0.0
    angle_max: float = 0.0
    angle_increment: float = 0.0
    range_min: float = 0.0
    range_max: float = 0.0
    frame_id: str = "rplidar_link"
    stamp: float | None = None
    raw_pairs: list[tuple[float, float]] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        import os

        d = asdict(self)
        count = len(self.ranges)
        raw_count = len(self.raw_pairs)
        d["rangesCount"] = max(count, raw_count)
        d["rawCount"] = raw_count
        # 맵 표시용 포인트 수 (기본 2880, 상한 8640)
        max_points = int(os.environ.get("PINKY_LIDAR_API_POINTS", "2880"))
        max_points = max(360, min(8640, max_points))

        points: list[dict[str, float]] = []
        # 원시 각-거리 쌍이 있으면 그걸 우선 (빈 샘플링보다 조밀)
        if self.raw_pairs:
            n = len(self.raw_pairs)
            step = max(1, (n + max_points - 1) // max_points) if n > max_points else 1
            for angle_deg, r in self.raw_pairs[::step]:
                if r is None or r <= 0 or r != r:
                    continue
                if self.range_max and r > self.range_max:
                    continue
                ang = math.radians(float(angle_deg))
                points.append(
                    {
                        "x": float(r * math.cos(ang)),
                        "y": float(r * math.sin(ang)),
                        "r": float(r),
                    }
                )
                if len(points) >= max_points:
                    break
            sample_inc = (2 * math.pi) / max(len(points), 1)
            sample = [p["r"] for p in points]
        else:
            if count > max_points and count > 0:
                step = max(1, (count + max_points - 1) // max_points)
                sample = self.ranges[::step][:max_points]
                sample_inc = self.angle_increment * step
            else:
                sample = list(self.ranges)
                sample_inc = self.angle_increment or (
                    (self.angle_max - self.angle_min) / max(count, 1)
                )
            for i, r in enumerate(sample):
                if r is None or r <= 0 or r != r:
                    continue
                if self.range_max and r > self.range_max:
                    continue
                angle = self.angle_min + i * sample_inc
                points.append(
                    {
                        "x": float(r * math.cos(angle)),
                        "y": float(r * math.sin(angle)),
                        "r": float(r),
                    }
                )
                if len(points) >= max_points:
                    break

        d["rangesSample"] = sample if not self.raw_pairs else [p["r"] for p in points]
        d["angleMin"] = self.angle_min
        d["angleMax"] = self.angle_max
        d["angleIncrement"] = (
            sample_inc if not self.raw_pairs else (2 * math.pi / max(len(points), 1))
        )
        d["rangeMin"] = self.range_min
        d["rangeMax"] = self.range_max
        d["frameId"] = self.frame_id
        d["points"] = points
        d.pop("ranges", None)
        d.pop("raw_pairs", None)
        return d


@dataclass
class ImuData:
    orientation: dict[str, float] = field(
        default_factory=lambda: {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    )
    angular_velocity: dict[str, float] = field(
        default_factory=lambda: {"x": 0.0, "y": 0.0, "z": 0.0}
    )
    linear_acceleration: dict[str, float] = field(
        default_factory=lambda: {"x": 0.0, "y": 0.0, "z": 0.0}
    )
    frame_id: str = "imu_link"
    stamp: float | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "orientation": self.orientation,
            "angularVelocity": self.angular_velocity,
            "linearAcceleration": self.linear_acceleration,
            "frameId": self.frame_id,
            "stamp": self.stamp,
        }


@dataclass
class UltrasonicData:
    range_m: float | None = None
    min_range: float = 0.02
    max_range: float = 3.0
    frame_id: str = "ultrasonic_link"
    ir_raw: list[int] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "rangeM": self.range_m,
            "minRange": self.min_range,
            "maxRange": self.max_range,
            "frameId": self.frame_id,
            "irRaw": self.ir_raw,
        }


@dataclass
class RobotSnapshot:
    device_code: str
    battery: BatteryData
    lidar: LidarData
    imu: ImuData
    ultrasonic: UltrasonicData
    backend: str
    online: bool = True
    pose: dict[str, float] | None = None
    navigating: bool = False

    def to_dict(self) -> dict[str, Any]:
        battery = self.battery.to_dict()
        lidar = self.lidar.to_dict()
        imu = self.imu.to_dict()
        ultrasonic = self.ultrasonic.to_dict()
        has_battery = battery.get("percent") is not None or battery.get("voltage") is not None
        has_lidar = int(lidar.get("rangesCount") or 0) > 0
        has_imu = imu.get("stamp") is not None
        has_us = ultrasonic.get("rangeM") is not None
        warnings: list[str] = []
        if not has_battery:
            warnings.append("battery: no data (publisher/pinkylib/ADC 확인)")
        if not has_lidar:
            warnings.append("lidar: no /scan (sllidar bringup 확인)")
        if not has_imu:
            warnings.append("imu: no data (BNO055/I2C 또는 publisher 확인)")
        if not has_us:
            warnings.append("ultrasonic: no data (ADC/I2C 또는 publisher 확인)")
        return {
            "deviceCode": self.device_code,
            "backend": self.backend,
            "online": self.online,
            "battery": battery,
            "lidar": lidar,
            "imu": imu,
            "ultrasonic": ultrasonic,
            "pose": self.pose,
            "navigating": self.navigating,
            "hasData": {
                "battery": has_battery,
                "lidar": has_lidar,
                "imu": has_imu,
                "ultrasonic": has_us,
            },
            "warnings": warnings,
        }
