from __future__ import annotations

from typing import Any

from .backends.base import RobotBackend


class LedModule:
    """LED: /set_led, /set_brightness (pinky_led / pinkylib)."""

    def __init__(self, backend: RobotBackend):
        self._backend = backend

    def fill(self, r: int, g: int, b: int) -> dict[str, Any]:
        return self._backend.set_led("fill", r, g, b)

    def set_pixel(self, pixels: list[int], r: int, g: int, b: int) -> dict[str, Any]:
        return self._backend.set_led("set_pixel", r, g, b, pixels)

    def clear(self) -> dict[str, Any]:
        return self._backend.set_led("clear", 0, 0, 0)

    def set_brightness(self, brightness: int) -> dict[str, Any]:
        return self._backend.set_brightness(brightness)
