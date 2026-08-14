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

    # ---- Navigation (map frame) — optional; defaults for backends without nav ----
    def get_nav_pose(self) -> dict[str, float] | None:
        return None

    def is_navigating(self) -> bool:
        return False

    def get_nav_path(self) -> dict[str, Any] | None:
        """Latest Nav2 global path observed from the planner."""
        return None

    def compute_path_to(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        timeout_sec: float = 10.0,
        planner_id: str = "",
    ) -> dict[str, Any]:
        del x, y, yaw, timeout_sec, planner_id
        return {"success": False, "message": "path planning not supported"}

    def set_initial_pose(self, x: float, y: float, yaw: float = 0.0) -> dict[str, Any]:
        return {"success": False, "message": "navigation not supported"}

    def navigate_to(self, x: float, y: float, yaw: float = 0.0) -> dict[str, Any]:
        return {"success": False, "message": "navigation not supported"}

    def navigate_to_wait(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        timeout_sec: float = 180.0,
    ) -> dict[str, Any]:
        del timeout_sec
        return self.navigate_to(x, y, yaw)

    def cancel_navigation(self) -> dict[str, Any]:
        return {"success": False, "message": "navigation not supported"}
