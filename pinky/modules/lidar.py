from __future__ import annotations

from .backends.base import RobotBackend
from .types import LidarData


class LidarModule:
    """라이다: /scan (sensor_msgs/LaserScan, RPLidar C1)."""

    def __init__(self, backend: RobotBackend):
        self._backend = backend

    def read(self) -> LidarData:
        return self._backend.get_lidar()
