from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from ..types import BatteryData, ImuData, LidarData, UltrasonicData


class RobotBackend(ABC):
    """Hardware/ROS/Mock access for Pinky Pro sensors & actuators."""

    name: str = "base"

    @abstractmethod
    def start(self) -> None: ...

    @abstractmethod
    def stop(self) -> None: ...

    @abstractmethod
    def get_battery(self) -> BatteryData: ...

    @abstractmethod
    def get_lidar(self) -> LidarData: ...

    @abstractmethod
    def get_imu(self) -> ImuData: ...

    @abstractmethod
    def get_ultrasonic(self) -> UltrasonicData: ...

    @abstractmethod
    def set_led(
        self,
        command: str = "fill",
        r: int = 0,
        g: int = 0,
        b: int = 0,
        pixels: list[int] | None = None,
    ) -> dict[str, Any]: ...

    @abstractmethod
    def set_brightness(self, brightness: int) -> dict[str, Any]: ...

    @abstractmethod
    def set_emotion(self, emotion: str) -> dict[str, Any]: ...

    @abstractmethod
    def drive(self, linear_x: float, angular_z: float) -> dict[str, Any]: ...

    @abstractmethod
    def is_online(self) -> bool: ...
