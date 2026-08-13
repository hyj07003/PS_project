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

    def go_to_wait(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        timeout_sec: float = 180.0,
    ) -> dict[str, Any]:
        return self._backend.navigate_to_wait(
            float(x), float(y), float(yaw), float(timeout_sec)
        )

    def cancel(self) -> dict[str, Any]:
        return self._backend.cancel_navigation()

    def get_plan(self) -> dict[str, Any]:
        getter = getattr(self._backend, "get_nav_plan", None)
        if callable(getter):
            try:
                return getter() or {
                    "ok": True,
                    "frameId": "map",
                    "stampSec": None,
                    "pointCount": 0,
                    "poses": [],
                    "message": "no plan",
                }
            except Exception as exc:
                return {
                    "ok": False,
                    "frameId": "map",
                    "stampSec": None,
                    "pointCount": 0,
                    "poses": [],
                    "message": str(exc),
                }
        return {
            "ok": True,
            "frameId": "map",
            "stampSec": None,
            "pointCount": 0,
            "poses": [],
            "message": "no plan",
        }

    def state(self) -> dict[str, Any]:
        info = self.map_info()
        loc = {}
        getter = getattr(self._backend, "get_localization_mode", None)
        if callable(getter):
            try:
                loc = getter() or {}
            except Exception:
                loc = {}
        return {
            "pose": self.get_pose(),
            "navigating": self.is_navigating(),
            "mapId": info.get("mapId") if info else None,
            "map": info,
            "amclActive": loc.get("amclActive"),
            "localizationMode": loc.get("localizationMode") or "active",
            "amclIdleFreeze": bool(loc.get("amclIdleFreeze")),
        }
