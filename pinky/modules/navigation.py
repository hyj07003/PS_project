from __future__ import annotations

from typing import Any

from .backends.base import RobotBackend
from .map_loader import MapInfo, load_map_info, map_png_bytes


class NavigationModule:
    """맵 좌표 네비게이션 (Nav2 NavigateToPose / initialpose 브리지)."""

    def __init__(self, backend: RobotBackend):
        self._backend = backend
        self._map: MapInfo | None = None
        try:
            self._map = load_map_info()
        except Exception:
            self._map = None

    def map_info(self) -> dict[str, Any] | None:
        if self._map is None:
            try:
                self._map = load_map_info()
            except Exception:
                return None
        return self._map.to_dict()

    def map_png(self) -> bytes:
        return map_png_bytes()

    def get_pose(self) -> dict[str, float] | None:
        return self._backend.get_nav_pose()

    def is_navigating(self) -> bool:
        return self._backend.is_navigating()

    def set_initial_pose(self, x: float, y: float, yaw: float = 0.0) -> dict[str, Any]:
        return self._backend.set_initial_pose(float(x), float(y), float(yaw))

    def go_to(self, x: float, y: float, yaw: float = 0.0) -> dict[str, Any]:
        return self._backend.navigate_to(float(x), float(y), float(yaw))

    def cancel(self) -> dict[str, Any]:
        return self._backend.cancel_navigation()

    def state(self) -> dict[str, Any]:
        info = self.map_info()
        return {
            "pose": self.get_pose(),
            "navigating": self.is_navigating(),
            "mapId": info.get("mapId") if info else None,
            "map": info,
        }
