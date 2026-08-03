from __future__ import annotations

from typing import Any

from .backends.base import RobotBackend
from .backends.mock import EMOTIONS


class LcdModule:
    """LCD 표정: /set_emotion (pinky_emotion SPI LCD)."""

    EMOTIONS = EMOTIONS

    def __init__(self, backend: RobotBackend):
        self._backend = backend

    def set_emotion(self, emotion: str) -> dict[str, Any]:
        return self._backend.set_emotion(emotion)

    def list_emotions(self) -> list[str]:
        return list(EMOTIONS)
