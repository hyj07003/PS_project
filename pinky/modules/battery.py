from __future__ import annotations

from .backends.base import RobotBackend
from .types import BatteryData


class BatteryModule:
    """배터리: /battery/percent, /battery/voltage (pinky_pro bringup)."""

    def __init__(self, backend: RobotBackend):
        self._backend = backend

    def read(self) -> BatteryData:
        return self._backend.get_battery()
