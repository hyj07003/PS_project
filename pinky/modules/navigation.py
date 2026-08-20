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
        if not self.is_navigating():
            return {
                "frameId": "map",
                "count": 0,
                "poses": [],
            }
        plan = self.get_plan()
        poses = plan.get("poses") if isinstance(plan, dict) else None
        out: list[dict[str, float]] = []
        if isinstance(poses, list):
            for p in poses:
                if isinstance(p, dict) and "x" in p and "y" in p:
                    out.append({"x": float(p["x"]), "y": float(p["y"])})
        if not out:
            return {
                "frameId": "map",
                "count": 0,
                "poses": [],
            }
        return {
            "frameId": plan.get("frameId") or "map",
            "count": len(out),
            "poses": out,
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
            persist=False,
        )

    def cancel(self, *, freeze: bool = True) -> dict[str, Any]:
        cancel = getattr(self._backend, "cancel_navigation", None)
        if callable(cancel):
            try:
                return cancel(freeze=bool(freeze))
            except TypeError:
                return cancel()
        return {"success": False, "message": "cancel not supported"}

    def prepare_new_job(self) -> dict[str, Any]:
        """실패/정지 잔여 Nav2·아루코를 끊고 새 할당이 주행할 수 있게 한다."""
        return self.cancel()

    def relative_move(
        self,
        distance_m: float,
        speed_mps: float = 0.02,
        timeout_sec: float | None = None,
        *,
        dry_run: bool = False,
        bypass_collision: bool = False,
        ignore_scan: bool = False,
    ) -> dict[str, Any]:
        return self._backend.relative_move(
            float(distance_m),
            float(speed_mps),
            float(timeout_sec) if timeout_sec is not None else None,
            dry_run=bool(dry_run),
            bypass_collision=bool(bypass_collision),
            ignore_scan=bool(ignore_scan),
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
            # 아루코 시작 시 잔여 Nav2만 취소. cancel_navigation 은 epoch 를
            # 올려 이 도킹 루프를 즉시 중단시키므로 sync cancel 만 쓴다.
            sync = getattr(self._backend, "_cancel_nav_sync", None)
            if callable(sync):
                try:
                    return sync(timeout_sec=2.0)
                except TypeError:
                    return sync()
            cancel = getattr(self._backend, "cancel_navigation", None)
            if callable(cancel):
                try:
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
            getter = getattr(self._backend, "get_odom_pose", None)
            if callable(getter):
                try:
                    return getter()
                except Exception:
                    return None
            return getattr(self._backend, "_odom_pose", None)

        start_epoch = 0
        epoch_now = getattr(self._backend, "_motion_epoch_now", None)
        if callable(epoch_now):
            try:
                start_epoch = int(epoch_now())
            except Exception:
                start_epoch = 0

        def _should_cancel() -> bool:
            interrupted = getattr(self._backend, "motion_interrupted", None)
            if callable(interrupted):
                try:
                    return bool(interrupted(start_epoch))
                except TypeError:
                    return bool(interrupted())
            return False

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
                should_cancel=_should_cancel,
            )
        finally:
            self._aruco_status = {
                **self._aruco_status,
                "active": False,
            }

    def aruco_undock(
        self,
        marker_id: int,
        target_range_m: float,
        *,
        timeout_sec: float | None = None,
        speed_mps: float | None = None,
        max_travel_m: float | None = None,
    ) -> dict[str, Any]:
        from .aruco_dock import run_aruco_undock

        mock = getattr(self._backend, "name", "") == "mock"
        get_ultrasonic = getattr(self._backend, "get_ultrasonic", None)

        def _hold_pose() -> None:
            begin = getattr(self._backend, "begin_visual_dock_hold", None)
            if callable(begin):
                try:
                    if begin():
                        return
                except Exception:
                    pass

        def _release_hold() -> None:
            end = getattr(self._backend, "end_visual_dock_hold", None)
            if callable(end):
                try:
                    end()
                except Exception:
                    pass

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
                "targetRangeM": info.get("targetRangeM"),
                "distanceSource": info.get("distanceSource"),
                "movedM": info.get("movedM"),
            }

        def _get_odom():
            getter = getattr(self._backend, "get_odom_pose", None)
            if callable(getter):
                try:
                    return getter()
                except Exception:
                    return None
            return getattr(self._backend, "_odom_pose", None)

        start_epoch = 0
        epoch_now = getattr(self._backend, "_motion_epoch_now", None)
        if callable(epoch_now):
            try:
                start_epoch = int(epoch_now())
            except Exception:
                start_epoch = 0

        def _should_cancel() -> bool:
            interrupted = getattr(self._backend, "motion_interrupted", None)
            if callable(interrupted):
                try:
                    return bool(interrupted(start_epoch))
                except TypeError:
                    return bool(interrupted())
            return False

        try:
            return run_aruco_undock(
                int(marker_id),
                float(target_range_m),
                _drive,
                hold_pose=_hold_pose,
                release_hold=_release_hold,
                get_ultrasonic=get_ultrasonic if callable(get_ultrasonic) else None,
                get_odom=_get_odom,
                on_progress=_on_progress,
                should_cancel=_should_cancel,
                speed_mps=speed_mps,
                timeout_sec=timeout_sec,
                max_travel_m=max_travel_m,
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
        readiness = {}
        readiness_getter = getattr(self._backend, "get_navigation_readiness", None)
        if callable(readiness_getter):
            try:
                readiness = readiness_getter() or {}
            except Exception:
                readiness = {}
        action = {}
        action_getter = getattr(self._backend, "get_navigation_action", None)
        if callable(action_getter):
            try:
                action = action_getter() or {}
            except Exception:
                action = {}
        path_pts: list[dict[str, float]] = []
        if self.is_navigating():
            try:
                packed = self.get_path() or {}
                for p in packed.get("poses") or []:
                    if not isinstance(p, dict):
                        continue
                    x = p.get("x")
                    y = p.get("y")
                    if isinstance(x, (int, float)) and isinstance(y, (int, float)):
                        path_pts.append({"x": float(x), "y": float(y)})
            except Exception:
                path_pts = []
        goal = None
        if self.is_navigating():
            goal_getter = getattr(self._backend, "get_active_nav_goal", None)
            if callable(goal_getter):
                try:
                    goal = goal_getter()
                except Exception:
                    goal = None
        return {
            "pose": self.get_pose(),
            "navigating": self.is_navigating(),
            "path": path_pts,
            "goal": goal,
            "mapId": info.get("mapId") if info else None,
            "map": info,
            "amclActive": loc.get("amclActive"),
            "localizationMode": loc.get("localizationMode") or "active",
            "amclIdleFreeze": bool(loc.get("amclIdleFreeze")),
            "nav2Readiness": readiness,
            "navigationAction": action,
            "arucoDock": self.get_aruco_status(),
        }
