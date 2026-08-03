from __future__ import annotations

from .backends.base import RobotBackend
from .types import UltrasonicData


class UltrasonicModule:
    """초음파/IR: /us_sensor/range, /ir_sensor/range (pinky_sensor_adc)."""

    def __init__(self, backend: RobotBackend):
        self._backend = backend

    def read(self) -> UltrasonicData:
        return self._backend.get_ultrasonic()
