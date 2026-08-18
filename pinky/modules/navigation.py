from __future__ import annotations

from typing import Any

from .backends.base import RobotBackend
from .map_loader import MapInfo, load_map_info, map_png_bytes


class NavigationModule:
    """맵 좌표 네비게이션 (Nav2 NavigateToPose / initialpose 브리지)."""

    def __init__(self, backend: RobotBackend):
        self._backend = backend
        self._map: MapInfo | None = None
        self._aruco_status: dict[str, Any] = {
            "active": False,
            "phase": None,
            "phaseLabel": None,
            "markerId": None,
        }
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

    def get_path(self) -> dict[str, Any] | None:
        """Latest cached path in /nav/path shape (compat for RViz bridge)."""
        plan = self.get_plan()
        poses = plan.get("poses") if isinstance(plan, dict) else None
        if not poses:
            return None
        return {
            "frameId": plan.get("frameId") or "map",
            "count": int(plan.get("pointCount") or len(poses)),
            "poses": [
                {"x": float(p["x"]), "y": float(p["y"])}
                for p in poses
                if isinstance(p, dict) and "x" in p and "y" in p
            ],
        }

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

    def plan_to(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        timeout_sec: float = 10.0,
        planner_id: str = "",
    ) -> dict[str, Any]:
        return self._backend.compute_path_to(
            float(x),
            float(y),
            float(yaw),
            float(timeout_sec),
            str(planner_id),
        )

    def cancel(self) -> dict[str, Any]:
        return self._backend.cancel_navigation()

    def relative_move(
        self,
        distance_m: float,
        speed_mps: float = 0.02,
        timeout_sec: float | None = None,
        *,
        dry_run: bool = False,
    ) -> dict[str, Any]:
        return self._backend.relative_move(
            float(distance_m),
            float(speed_mps),
            float(timeout_sec) if timeout_sec is not None else None,
            dry_run=bool(dry_run),
        )

    def aruco_dock(
        self,
        marker_id: int,
        standoff_m: float | None = None,
        timeout_sec: float | None = None,
    ) -> dict[str, Any]:
        from .aruco_dock import run_aruco_dock

        mock = getattr(self._backend, "name", "") == "mock"

        def _cancel_nav() -> Any:
            cancel = getattr(self._backend, "cancel_navigation", None)
            if callable(cancel):
                try:
                    # Pose pin is done by begin_visual_dock_hold — avoid home re-freeze
                    return cancel(freeze=False)
                except TypeError:
                    return cancel()
            return self.cancel()

        def _hold_pose() -> None:
            begin = getattr(self._backend, "begin_visual_dock_hold", None)
            if callable(begin):
                try:
                    if begin():
                        return
                except Exception:
                    pass
            # 홈으로 freeze 하면 맵이 S1/S2 로 점프 — fallback 금지

        def _release_hold() -> None:
            end = getattr(self._backend, "end_visual_dock_hold", None)
            if callable(end):
                try:
                    end()
                except Exception:
                    pass

        get_lidar = getattr(self._backend, "get_lidar", None)
        get_ultrasonic = getattr(self._backend, "get_ultrasonic", None)

        def _drive(linear_x: float, angular_z: float) -> Any:
            try:
                return self._backend.drive(
                    linear_x, angular_z, bypass_collision=True
                )
            except TypeError:
                return self._backend.drive(linear_x, angular_z)

        def _on_progress(info: dict[str, Any]) -> None:
            self._aruco_status = {
                "active": bool(info.get("active", True)),
                "phase": info.get("phase"),
                "phaseLabel": info.get("phaseLabel"),
                "markerId": info.get("markerId"),
                "distanceM": info.get("distanceM"),
                "distanceSource": info.get("distanceSource"),
                "ultrasonicM": info.get("ultrasonicM"),
                "centerErrorPx": info.get("centerErrorPx"),
                "lateralM": info.get("lateralM"),
                "yawErrRad": info.get("yawErrRad"),
            }

        def _get_odom():
            odom = getattr(self._backend, "_odom_pose", None)
            return odom

        try:
            return run_aruco_dock(
                marker_id=int(marker_id),
                drive=_drive,
                cancel_nav=_cancel_nav,
                hold_pose=_hold_pose,
                release_hold=_release_hold,
                get_lidar=get_lidar if callable(get_lidar) else None,
                get_ultrasonic=get_ultrasonic if callable(get_ultrasonic) else None,
                on_progress=_on_progress,
                get_odom=_get_odom,
                standoff_m=standoff_m,
                timeout_sec=timeout_sec,
                mock=mock,
            )
        finally:
            self._aruco_status = {
                **self._aruco_status,
                "active": False,
            }

    def get_aruco_status(self) -> dict[str, Any]:
        return dict(self._aruco_status)

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
            "arucoDock": self.get_aruco_status(),
        }
