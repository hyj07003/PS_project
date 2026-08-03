from __future__ import annotations

from .backends.base import RobotBackend
from .types import ImuData


class ImuModule:
    """IMU: /imu_raw (sensor_msgs/Imu, BNO055)."""

    def __init__(self, backend: RobotBackend):
        self._backend = backend

    def read(self) -> ImuData:
        return self._backend.get_imu()
