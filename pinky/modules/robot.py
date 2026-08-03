from __future__ import annotations

from typing import Any

from .backends import create_backend
from .backends.base import RobotBackend
from .battery import BatteryModule
from .imu import ImuModule
from .lcd import LcdModule
from .led import LedModule
from .lidar import LidarModule
from .types import RobotSnapshot
from .ultrasonic import UltrasonicModule


class PinkyRobot:
    """pinky_pro 센서/액추에이터 통합 파사드."""

    def __init__(
        self,
        backend: str | RobotBackend | None = None,
        device_code: str = "cart-1",
    ):
        if isinstance(backend, RobotBackend):
            self._backend = backend
        else:
            self._backend = create_backend(backend)
        self.device_code = device_code
        self.battery = BatteryModule(self._backend)
        self.lidar = LidarModule(self._backend)
        self.imu = ImuModule(self._backend)
        self.ultrasonic = UltrasonicModule(self._backend)
        self.led = LedModule(self._backend)
        self.lcd = LcdModule(self._backend)

    @property
    def backend_name(self) -> str:
        return self._backend.name

    def start(self) -> None:
        self._backend.start()

    def stop(self) -> None:
        self._backend.stop()

    def snapshot(self) -> RobotSnapshot:
        return RobotSnapshot(
            device_code=self.device_code,
            battery=self.battery.read(),
            lidar=self.lidar.read(),
            imu=self.imu.read(),
            ultrasonic=self.ultrasonic.read(),
            backend=self.backend_name,
            online=self._backend.is_online(),
        )

    def drive(self, linear_x: float, angular_z: float) -> dict[str, Any]:
        return self._backend.drive(linear_x, angular_z)
