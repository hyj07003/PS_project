"""Central multi-robot traffic coordination for controller-server."""

from __future__ import annotations

import logging
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Protocol

from .traffic_paths import (
    advance_path_index,
    distance_to_point,
    find_path_conflicts,
    has_passed_path_index,
    path_intersects_zones,
    path_points,
)

LOGGER = logging.getLogger(__name__)


class CartPort(Protocol):
    def plan_pose(
        self,
        device_code: str,
        x: float,
        y: float,
        yaw: float = 0.0,
        timeout_sec: float = 10.0,
    ) -> dict[str, Any]: ...

    def get_pose(self, device_code: str) -> dict[str, float] | None: ...

    def get_nav_state_full(self, device_code: str) -> dict[str, Any]: ...

    def get_active_path(self, device_code: str) -> dict[str, Any]: ...

    def is_reachable(self, device_code: str) -> bool: ...


@dataclass(frozen=True)
class NavGoal:
    x: float
    y: float
    yaw: float


@dataclass
class RobotTrafficState:
    device_code: str
    mission_id: int | None = None
    mission_assigned_at: float = 0.0
    phase: str = "IDLE"
    active_path: list[tuple[float, float]] = field(default_factory=list)
    release_index: int | None = None
    planned_goal: NavGoal | None = None
    returning_home: bool = False
    occupied_waypoint: str | None = None
    remaining_waypoints: list[str] = field(default_factory=list)


class TrafficTimeoutError(RuntimeError):
    pass


class TrafficCoordinator:
    """Mission-FIFO path reservation for two Pinky carts."""

    def __init__(self, cart_port: CartPort, robot_codes: list[str] | None = None) -> None:
        self._cart_port = cart_port
        codes = robot_codes or []
        self._robot_codes = sorted({c.strip().lower() for c in codes if c})
        self._lock = threading.RLock()
        self._cond = threading.Condition(self._lock)
        self._robots: dict[str, RobotTrafficState] = {
            code: RobotTrafficState(device_code=code) for code in self._robot_codes
        }
        self._home_owner: str | None = None
        self._home_waiters: list[str] = []
        self._abort_gen: dict[str, int] = {}
        self._clearance_m = float(os.environ.get("TRAFFIC_CLEARANCE_M", "0.20"))
        self._release_margin_m = float(os.environ.get("TRAFFIC_RELEASE_MARGIN_M", "0.20"))
        self._plan_timeout = float(os.environ.get("TRAFFIC_PLAN_TIMEOUT_SEC", "10"))
        self._poll_hz = float(os.environ.get("TRAFFIC_POLL_HZ", "2.0"))
        self._mission_timeout = float(os.environ.get("TRAFFIC_MISSION_TIMEOUT", "300"))
        self._goal_tolerance = float(os.environ.get("TRAFFIC_GOAL_TOLERANCE_M", "0.10"))
        self._home_priority = (
            os.environ.get("TRAFFIC_HOME_PRIORITY", "fifo").strip().lower()
        )
        from ..waypoints import staging_waypoint_id, waypoint_zone_radius_m

        self._zone_radius_m = waypoint_zone_radius_m()
        self._staging_waypoint = staging_waypoint_id()

    def enabled(self) -> bool:
        flag = (os.environ.get("TRAFFIC_ENABLED") or "1").strip().lower()
        if flag in ("0", "false", "off", "no"):
            return False
        return len(self._robot_codes) >= 2

    def register_mission(self, device_code: str, mission_id: int) -> None:
        if not self.enabled():
            return
        code = device_code.strip().lower()
        with self._lock:
            state = self._robots.setdefault(code, RobotTrafficState(device_code=code))
            state.mission_id = int(mission_id)
            state.mission_assigned_at = time.monotonic()

    def unregister_mission(self, device_code: str) -> None:
        code = device_code.strip().lower()
        with self._cond:
            state = self._robots.get(code)
            if state is None:
                return
            state.mission_id = None
            state.phase = "IDLE"
            state.active_path = []
            state.release_index = None
            state.planned_goal = None
            state.returning_home = False
            state.occupied_waypoint = None
            state.remaining_waypoints = []
            self._cond.notify_all()

    def interrupt_robot(self, device_code: str) -> None:
        """정지/실패 시 교통 대기를 깨워 다음 할당이 막히지 않게 한다."""
        code = device_code.strip().lower()
        with self._cond:
            self._abort_gen[code] = int(self._abort_gen.get(code, 0)) + 1
            state = self._robots.get(code)
            if state is not None:
                state.phase = "IDLE"
                state.active_path = []
                state.release_index = None
                state.planned_goal = None
                state.returning_home = False
                state.occupied_waypoint = None
                state.remaining_waypoints = []
            if self._home_owner == code:
                self._home_owner = None
            self._cond.notify_all()

    def update_remaining(self, device_code: str, waypoint_ids: list[str]) -> None:
        if not self.enabled():
            return
        code = device_code.strip().lower()
        with self._lock:
            state = self._robots.setdefault(code, RobotTrafficState(device_code=code))
            state.remaining_waypoints = list(waypoint_ids)
            self._cond.notify_all()

    def try_claim_waypoint_zone(self, device_code: str, waypoint_id: str) -> bool:
        """Atomically claim zone; False if another robot already holds it."""
        if not self.enabled():
            return True
        from ..waypoints import is_zone_occupiable

        wid = (waypoint_id or "").strip().upper()
        if not is_zone_occupiable(wid):
            return True
        code = device_code.strip().lower()
        with self._lock:
            for other_code, other_state in self._robots.items():
                if other_code == code:
                    continue
                if other_state.occupied_waypoint == wid:
                    return False
            state = self._robots.setdefault(code, RobotTrafficState(device_code=code))
            state.occupied_waypoint = wid
            self._cond.notify_all()
            return True

    def claim_waypoint_zone(self, device_code: str, waypoint_id: str) -> None:
        if not self.try_claim_waypoint_zone(device_code, waypoint_id):
            raise TrafficTimeoutError(
                f"waypoint zone already occupied: {waypoint_id}"
            )

    def release_waypoint_zone(self, device_code: str) -> None:
        if not self.enabled():
            return
        code = device_code.strip().lower()
        with self._lock:
            state = self._robots.get(code)
            if state is None:
                return
            state.occupied_waypoint = None
            self._cond.notify_all()

    def occupied_zones(
        self, exclude_device: str | None = None
    ) -> list[tuple[float, float, float]]:
        from ..waypoints import waypoint_zone_center

        exclude = (exclude_device or "").strip().lower()
        zones: list[tuple[float, float, float]] = []
        with self._lock:
            for code, state in self._robots.items():
                if code == exclude or not state.occupied_waypoint:
                    continue
                cx, cy = waypoint_zone_center(state.occupied_waypoint)
                zones.append((cx, cy, self._zone_radius_m))
        return zones

    def conflicting_waypoints(
        self, device_code: str, shelf_ids: list[str]
    ) -> set[str]:
        """Shelf ids to defer: other occupancy + remaining overlap (waiter loses)."""
        if not self.enabled():
            return set()
        code = device_code.strip().lower()
        targets = {s.strip().upper() for s in shelf_ids}
        defer: set[str] = set()
        with self._lock:
            self_state = self._robots.get(code)
            self_remaining = set(self_state.remaining_waypoints) if self_state else set()
            for other_code, other_state in self._robots.items():
                if other_code == code or other_state.mission_id is None:
                    continue
                if other_state.occupied_waypoint in targets:
                    defer.add(other_state.occupied_waypoint)
                overlap = targets & set(other_state.remaining_waypoints)
                if not overlap:
                    continue
                if self_state is None:
                    defer.update(overlap)
                    continue
                if self_state.mission_assigned_at > other_state.mission_assigned_at:
                    defer.update(overlap)
                elif self_state.mission_assigned_at == other_state.mission_assigned_at:
                    if code > other_code:
                        defer.update(overlap)
        return defer

    def waypoint_access_granted(self, device_code: str, waypoint_id: str) -> bool:
        if not self.enabled():
            return True
        from ..waypoints import is_zone_occupiable

        wid = (waypoint_id or "").strip().upper()
        if not is_zone_occupiable(wid):
            return True
        code = device_code.strip().lower()
        with self._lock:
            for other_code, other_state in self._robots.items():
                if other_code == code:
                    continue
                if wid == "P" and other_state.returning_home:
                    return False
                if other_state.mission_id is None:
                    continue
                if other_state.occupied_waypoint == wid:
                    return False
            self_state = self._robots.get(code)
            if self_state is None:
                return True
            for other_code, other_state in self._robots.items():
                if other_code == code or other_state.mission_id is None:
                    continue
                if wid not in other_state.remaining_waypoints:
                    continue
                if wid not in self_state.remaining_waypoints:
                    continue
                # Earlier mission assignment = owner for overlapping shelves.
                if self_state.mission_assigned_at > other_state.mission_assigned_at:
                    return False
                if self_state.mission_assigned_at < other_state.mission_assigned_at:
                    continue
                if code > other_code:
                    return False
        return True

    def acquire_waypoint_access(
        self,
        device_code: str,
        waypoint_id: str,
        timeout_sec: float | None = None,
    ) -> None:
        if not self.enabled():
            return
        code = device_code.strip().lower()
        wait = (
            float(timeout_sec)
            if timeout_sec is not None and float(timeout_sec) > 0
            else None
        )
        deadline = (
            time.monotonic() + max(0.5, wait) if wait is not None else None
        )
        with self._cond:
            start_gen = int(self._abort_gen.get(code, 0))
            while deadline is None or time.monotonic() < deadline:
                if int(self._abort_gen.get(code, 0)) != start_gen:
                    raise TrafficTimeoutError(f"traffic interrupted for {code}")
                if self.waypoint_access_granted(code, waypoint_id):
                    return
                self._cond.wait(timeout=1.0 / max(0.2, self._poll_hz))
        if not self.waypoint_access_granted(code, waypoint_id):
            raise TrafficTimeoutError(
                f"waypoint access timed out for {code} -> {waypoint_id}"
            )

    def acquire_nav_leg(
        self,
        device_code: str,
        goal: NavGoal,
        mission_id: int,
        waypoint_id: str,
        *,
        skip_traffic_wait: bool = False,
    ) -> None:
        if not self.enabled() or skip_traffic_wait:
            if self.enabled():
                code = device_code.strip().lower()
                with self._lock:
                    state = self._robots.setdefault(
                        code, RobotTrafficState(device_code=code)
                    )
                    state.mission_id = int(mission_id)
                    state.planned_goal = goal
                    state.phase = "NAVIGATING"
            return
        code = device_code.strip().lower()
        deadline = time.monotonic() + self._mission_timeout
        with self._lock:
            start_gen = int(self._abort_gen.get(code, 0))
            state = self._robots.setdefault(code, RobotTrafficState(device_code=code))
            state.mission_id = int(mission_id)
            if state.mission_assigned_at <= 0.0:
                state.mission_assigned_at = time.monotonic()
            state.planned_goal = goal
            state.phase = "PLANNING"
            others_busy = self._has_other_busy(code)

        # Solo robot: do not block driving on POST /nav/plan.
        # A failed planner used to fail the whole mission before navigate_pose.
        if others_busy:
            planned = self._plan_leg(code, goal)
        else:
            planned = self._fallback_path(code, goal)

        while time.monotonic() < deadline:
            with self._lock:
                if int(self._abort_gen.get(code, 0)) != start_gen:
                    raise TrafficTimeoutError(f"traffic interrupted for {code}")
            other_paths = self._collect_other_paths(code)
            other_poses = self._collect_other_poses(code)
            with self._lock:
                grant, wait_reason = self._evaluate_leg_grant(
                    code, planned, other_paths, other_poses
                )
                if grant:
                    state = self._robots[code]
                    state.phase = "NAVIGATING"
                    state.active_path = list(planned)
                    self._cond.notify_all()
                    return
                state = self._robots[code]
                state.phase = "WAITING"
            self._sleep_poll()
            if wait_reason == "owner_passed":
                continue
        LOGGER.warning(
            "traffic wait timed out for %s -> (%.3f, %.3f); granting so nav can proceed",
            code,
            goal.x,
            goal.y,
        )
        with self._lock:
            state = self._robots[code]
            state.phase = "NAVIGATING"
            state.active_path = list(planned)
            self._cond.notify_all()

    def release_nav_leg(self, device_code: str) -> None:
        if not self.enabled():
            return
        code = device_code.strip().lower()
        with self._lock:
            state = self._robots.get(code)
            if state is None:
                return
            state.phase = "IDLE"
            state.active_path = []
            state.release_index = None
            state.planned_goal = None
            self._cond.notify_all()

    def mark_returning_home(self, device_code: str) -> None:
        """Flag P→home motion so the other cart waits at W7 before entering P."""
        if not self.enabled():
            return
        code = device_code.strip().lower()
        with self._lock:
            state = self._robots.setdefault(code, RobotTrafficState(device_code=code))
            state.returning_home = True
            if state.phase in ("IDLE", "NAVIGATING", "WAITING", "PLANNING"):
                state.phase = "RETURNING_HOME"
            self._cond.notify_all()

    def acquire_return_home(
        self, device_code: str, timeout_sec: float | None = None
    ) -> None:
        if not self.enabled():
            return
        code = device_code.strip().lower()
        wait = (
            float(timeout_sec)
            if timeout_sec is not None
            else self._mission_timeout
        )
        deadline = time.monotonic() + max(0.5, wait)
        with self._lock:
            start_gen = int(self._abort_gen.get(code, 0))
            state = self._robots.setdefault(code, RobotTrafficState(device_code=code))
            state.returning_home = True
            state.phase = "RETURNING_HOME"
            self._cond.notify_all()

        while time.monotonic() < deadline:
            with self._lock:
                if int(self._abort_gen.get(code, 0)) != start_gen:
                    raise TrafficTimeoutError(f"traffic interrupted for {code}")
                if self._home_owner is None:
                    self._home_owner = code
                    return
                if self._home_owner == code:
                    return
            if self._home_priority == "cart-1" and code == "cart-2":
                if self._cart1_home_complete():
                    with self._lock:
                        if self._home_owner is None:
                            self._home_owner = code
                            return
            self._sleep_poll()
        LOGGER.warning(
            "traffic return-home wait timed out for %s; granting so nav can proceed",
            code,
        )
        with self._lock:
            self._home_owner = code

    def release_return_home(self, device_code: str) -> None:
        if not self.enabled():
            return
        code = device_code.strip().lower()
        with self._lock:
            state = self._robots.get(code)
            if state is not None:
                state.returning_home = False
                state.phase = "IDLE"
            if self._home_owner == code:
                self._home_owner = None
            self._cond.notify_all()

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            robots = {
                code: {
                    "deviceCode": code,
                    "missionId": st.mission_id,
                    "phase": st.phase,
                    "pathPoints": len(st.active_path),
                    "releaseIndex": st.release_index,
                    "returningHome": st.returning_home,
                    "occupiedWaypoint": st.occupied_waypoint,
                    "remainingWaypoints": list(st.remaining_waypoints),
                    "plannedGoal": (
                        {
                            "x": st.planned_goal.x,
                            "y": st.planned_goal.y,
                            "yaw": st.planned_goal.yaw,
                        }
                        if st.planned_goal is not None
                        else None
                    ),
                }
                for code, st in self._robots.items()
            }
            return {
                "enabled": self.enabled(),
                "clearanceM": self._clearance_m,
                "releaseMarginM": self._release_margin_m,
                "zoneRadiusM": self._zone_radius_m,
                "stagingWaypoint": self._staging_waypoint,
                "homeOwner": self._home_owner,
                "robots": robots,
            }

    def _plan_leg(self, device_code: str, goal: NavGoal) -> list[tuple[float, float]]:
        try:
            result = self._cart_port.plan_pose(
                device_code,
                goal.x,
                goal.y,
                goal.yaw,
                timeout_sec=self._plan_timeout,
            )
        except Exception as exc:
            LOGGER.warning("traffic plan_pose raised for %s: %s", device_code, exc)
            return self._fallback_path(device_code, goal)
        points = path_points(result) if isinstance(result, dict) else []
        if not (isinstance(result, dict) and result.get("success") and points):
            LOGGER.warning(
                "traffic plan failed for %s; using fallback path: %s",
                device_code,
                result.get("message", result) if isinstance(result, dict) else result,
            )
            return self._fallback_path(device_code, goal)
        return points

    def _fallback_path(
        self, device_code: str, goal: NavGoal
    ) -> list[tuple[float, float]]:
        pose: dict[str, Any] | None = None
        try:
            pose = self._cart_port.get_pose(device_code)
        except Exception:
            pose = None
        start_x = 0.0
        start_y = 0.0
        if isinstance(pose, dict):
            try:
                start_x = float(pose.get("x") or 0.0)
                start_y = float(pose.get("y") or 0.0)
            except (TypeError, ValueError):
                pass
        return [(start_x, start_y), (goal.x, goal.y)]

    def _has_other_busy(self, device_code: str) -> bool:
        for code, state in self._robots.items():
            if code == device_code:
                continue
            if (
                state.mission_id is not None
                or state.active_path
                or state.phase not in ("IDLE",)
            ):
                return True
        return False

    def _collect_other_paths(
        self, device_code: str
    ) -> dict[str, list[tuple[float, float]]]:
        with self._lock:
            others = [
                (code, RobotTrafficState(
                    device_code=st.device_code,
                    mission_id=st.mission_id,
                    phase=st.phase,
                    active_path=list(st.active_path),
                ))
                for code, st in self._robots.items()
                if code != device_code and st.mission_id is not None
            ]
        out: dict[str, list[tuple[float, float]]] = {}
        for code, state in others:
            out[code] = self._live_path(code, state)
        return out

    def _collect_other_poses(self, device_code: str) -> dict[str, dict[str, float] | None]:
        with self._lock:
            codes = [
                code
                for code, st in self._robots.items()
                if code != device_code and st.mission_id is not None
            ]
        poses: dict[str, dict[str, float] | None] = {}
        for code in codes:
            try:
                poses[code] = self._cart_port.get_pose(code)
            except Exception:
                poses[code] = None
        return poses

    def _evaluate_leg_grant(
        self,
        device_code: str,
        planned_path: list[tuple[float, float]],
        other_paths: dict[str, list[tuple[float, float]]],
        other_poses: dict[str, dict[str, float] | None],
    ) -> tuple[bool, str]:
        others = [
            (code, st)
            for code, st in self._robots.items()
            if code != device_code and st.mission_id is not None
        ]
        if not others:
            return True, "solo"

        self_state = self._robots[device_code]
        occupied_zones = self.occupied_zones(exclude_device=device_code)
        if occupied_zones and path_intersects_zones(planned_path, occupied_zones):
            return False, "wait_zone"

        for other_code, other_state in others:
            other_path = other_paths.get(other_code) or list(other_state.active_path)
            if not other_path:
                continue
            conflict = find_path_conflicts(
                planned_path,
                other_path,
                clearance_m=self._clearance_m,
                max_samples=50,
            )
            segments = conflict.get("segments") or []
            if not segments:
                continue

            self_is_owner = self_state.mission_assigned_at <= other_state.mission_assigned_at
            if self_is_owner:
                release_segment = max(
                    segments, key=lambda item: int(item["cart1_end_index"])
                )
                self_state.release_index = advance_path_index(
                    planned_path,
                    int(release_segment["cart1_end_index"]),
                    self._release_margin_m,
                )
                return True, "owner"
            owner_code = other_code
            owner_state = other_state
            if owner_state.phase == "IDLE":
                return True, "owner_done"
            if owner_state.phase != "NAVIGATING" or not owner_state.active_path:
                return False, "wait_owner"
            owner_pose = other_poses.get(owner_code)
            release_idx = owner_state.release_index
            if release_idx is None:
                owner_conflict = find_path_conflicts(
                    owner_state.active_path,
                    planned_path,
                    clearance_m=self._clearance_m,
                    max_samples=10,
                )
                owner_segments = owner_conflict.get("segments") or []
                if owner_segments:
                    release_segment = max(
                        owner_segments,
                        key=lambda item: int(item["cart1_end_index"]),
                    )
                    release_idx = advance_path_index(
                        owner_state.active_path,
                        int(release_segment["cart1_end_index"]),
                        self._release_margin_m,
                    )
                    owner_state.release_index = release_idx
            if has_passed_path_index(
                owner_state.active_path, owner_pose, release_idx or 0
            ):
                return True, "owner_passed"
            return False, "wait_owner"
        return True, "clear"

    def _live_path(
        self,
        device_code: str,
        state: RobotTrafficState,
    ) -> list[tuple[float, float]]:
        try:
            nav_state = self._cart_port.get_nav_state_full(device_code)
        except Exception:
            nav_state = {}
        if bool(nav_state.get("navigating")):
            try:
                live = path_points(self._cart_port.get_active_path(device_code))
            except Exception:
                live = []
            if live:
                return live
        return list(state.active_path)

    def _cart1_home_complete(self) -> bool:
        from ..waypoints import home_for_device

        if not self._cart_port.is_reachable("cart-1"):
            return True
        home = home_for_device("cart-1")
        state = self._cart_port.get_nav_state_full("cart-1")
        if bool(state.get("navigating")):
            return False
        pose = state.get("pose") if isinstance(state.get("pose"), dict) else None
        if not pose:
            pose = self._cart_port.get_pose("cart-1")
        return distance_to_point(pose, (home.x, home.y)) <= self._goal_tolerance

    def _sleep_poll(self) -> None:
        delay = 1.0 / max(0.2, self._poll_hz)
        with self._cond:
            self._cond.wait(timeout=delay)
