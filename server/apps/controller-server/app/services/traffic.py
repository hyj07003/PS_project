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
    densify_path,
    distance_to_point,
    find_path_conflicts,
    has_passed_path_index,
    nearest_path_index,
    path_distance_between,
    path_goal_in_zones,
    path_points,
    pose_near_path,
    retreat_path_index,
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
    hold_index: int | None = None
    planned_goal: NavGoal | None = None
    returning_home: bool = False
    occupied_waypoint: str | None = None
    remaining_waypoints: list[str] = field(default_factory=list)
    conflict_owner_sticky: str | None = None
    last_wait_reason: str | None = None
    wait_started_at: float = 0.0


class TrafficTimeoutError(RuntimeError):
    pass


class TrafficYieldError(TrafficTimeoutError):
    """Soft wait limit: yield this leg so a peer order/path can proceed."""

    pass


class TrafficEmergencyWaitError(TrafficTimeoutError):
    """On peer NAV path — stage at W7 (비상대기). Not 충돌대기."""

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
        self._hold_margin_m = float(os.environ.get("TRAFFIC_HOLD_MARGIN_M", "0.35"))
        self._release_margin_m = float(os.environ.get("TRAFFIC_RELEASE_MARGIN_M", "0.20"))
        self._plan_timeout = float(os.environ.get("TRAFFIC_PLAN_TIMEOUT_SEC", "10"))
        self._poll_hz = float(os.environ.get("TRAFFIC_POLL_HZ", "2.0"))
        self._mission_timeout = float(os.environ.get("TRAFFIC_MISSION_TIMEOUT", "300"))
        # Soft yield before hard mission timeout — frees stuck waiters for peer work.
        self._yield_sec = float(os.environ.get("TRAFFIC_YIELD_SEC", "60"))
        self._goal_tolerance = float(os.environ.get("TRAFFIC_GOAL_TOLERANCE_M", "0.10"))
        self._wait_replan_sec = float(os.environ.get("TRAFFIC_WAIT_REPLAN_SEC", "3.0"))
        self._home_priority = (
            os.environ.get("TRAFFIC_HOME_PRIORITY", "fifo").strip().lower()
        )
        # S1/S2 are ~0.22m apart — serialize leaving the pad so paths don't clash.
        self._home_clear_m = float(os.environ.get("TRAFFIC_HOME_CLEAR_M", "0.55"))
        self._home_depart_owner: str | None = None
        from ..waypoints import staging_waypoint_id, waypoint_zone_radius_m

        self._zone_radius_m = waypoint_zone_radius_m()
        self._staging_waypoint = staging_waypoint_id()

    def enabled(self) -> bool:
        flag = (os.environ.get("TRAFFIC_ENABLED") or "1").strip().lower()
        if flag in ("0", "false", "off", "no"):
            return False
        if len(self._robot_codes) < 2:
            return False
        # Only coordinate when two carts both have active missions.
        # One order / one robot with two URLs registered must not hit stop_nav.
        with self._lock:
            active_missions = sum(
                1 for st in self._robots.values() if st.mission_id is not None
            )
        if active_missions < 2:
            return False
        reachable = 0
        for code in self._robot_codes:
            try:
                if self._cart_port.is_reachable(code):
                    reachable += 1
            except Exception:
                continue
        return reachable >= 2

    def last_wait_reason(self, device_code: str) -> str | None:
        code = device_code.strip().lower()
        with self._lock:
            state = self._robots.get(code)
            return state.last_wait_reason if state else None

    def _pose_near_point(
        self, pose: dict[str, float] | None, xy: tuple[float, float], radius_m: float
    ) -> bool:
        if not pose or pose.get("x") is None or pose.get("y") is None:
            return False
        try:
            dx = float(pose["x"]) - float(xy[0])
            dy = float(pose["y"]) - float(xy[1])
        except (TypeError, ValueError):
            return False
        return (dx * dx + dy * dy) ** 0.5 <= radius_m

    def _device_home_xy(self, device_code: str) -> tuple[float, float]:
        from ..waypoints import home_for_device

        home = home_for_device(device_code)
        return (home.x, home.y)

    def _pose_near_own_home(
        self, device_code: str, pose: dict[str, float] | None
    ) -> bool:
        return self._pose_near_point(
            pose, self._device_home_xy(device_code), self._home_clear_m
        )

    def _pose_clear_of_home_pads(self, pose: dict[str, float] | None) -> bool:
        """True when pose is outside both S1 and S2 clearance bubbles."""
        from ..waypoints import WAYPOINTS

        if not pose:
            return False
        for wid in ("S1", "S2"):
            wp = WAYPOINTS[wid]
            if self._pose_near_point(pose, (wp.x, wp.y), self._home_clear_m):
                return False
        return True

    def _release_home_depart_if_clear(self, device_code: str) -> None:
        code = device_code.strip().lower()
        with self._lock:
            if self._home_depart_owner != code:
                return
        try:
            pose = self._cart_port.get_pose(code)
        except Exception:
            pose = None
        if self._pose_clear_of_home_pads(pose):
            with self._lock:
                if self._home_depart_owner == code:
                    self._home_depart_owner = None
                    self._cond.notify_all()
                    LOGGER.info("traffic home-depart released %s (cleared pad)", code)

    def _try_claim_home_departure(
        self,
        device_code: str,
        self_pose: dict[str, float] | None,
        other_poses: dict[str, dict[str, float] | None],
    ) -> tuple[bool, str]:
        """Serialize leaving S1/S2. Returns (ok, reason)."""
        code = device_code.strip().lower()
        # Already off the pad — no departure lock needed; free slot if we held it.
        if not self._pose_near_own_home(code, self_pose):
            with self._lock:
                if self._home_depart_owner == code:
                    self._home_depart_owner = None
                    self._cond.notify_all()
            return True, "home_clear"

        with self._lock:
            self_state = self._robots.get(code)
            owner = self._home_depart_owner
            if owner and owner != code:
                owner_pose = other_poses.get(owner)
                owner_st = self._robots.get(owner)
                owner_gone = (
                    owner_st is None
                    or owner_st.mission_id is None
                    or owner_st.phase in ("IDLE",)
                    or self._pose_clear_of_home_pads(owner_pose)
                )
                if owner_gone:
                    self._home_depart_owner = None
                    owner = None

            if self._home_depart_owner == code:
                return True, "home_depart_owner"

            if self._home_depart_owner is not None:
                return False, "wait_home_depart"

            # Both still on pads: earlier mission leaves first.
            candidates: list[tuple[float, str]] = [
                (
                    self_state.mission_assigned_at if self_state else 0.0,
                    code,
                )
            ]
            for other, st in self._robots.items():
                if other == code or st.mission_id is None:
                    continue
                opose = other_poses.get(other)
                if self._pose_near_own_home(other, opose):
                    candidates.append((st.mission_assigned_at, other))
            candidates.sort(key=lambda t: (t[0], t[1]))
            winner = candidates[0][1]
            if winner != code:
                return False, "wait_home_depart"
            self._home_depart_owner = code
            LOGGER.info("traffic home-depart claimed by %s", code)
            return True, "home_depart_owner"

    def clear_stale_emergency_wait(self, device_code: str) -> None:
        """Drop sticky emergency_wait when pose is no longer on a peer NAV path."""
        code = device_code.strip().lower()
        if self.is_on_peer_active_path(code):
            return
        with self._lock:
            state = self._robots.get(code)
            if state is not None and state.last_wait_reason == "emergency_wait":
                state.last_wait_reason = None
                self._cond.notify_all()

    def is_on_peer_active_path(self, device_code: str) -> bool:
        """True when pose overlaps a peer that is actually NAVIGATING on remaining path.

        Only the path *ahead of* the peer (from their current index) counts — sitting
        near the start of a long reserved path must not freeze forever.
        """
        if not self.enabled():
            return False
        code = device_code.strip().lower()
        try:
            pose = self._cart_port.get_pose(code)
        except Exception:
            pose = None
        if not pose:
            return False
        # Parked on own S1/S2: home-depart serialization handles this — do not
        # enter emergency hold loops that cancel Nav2 forever.
        if self._pose_near_own_home(code, pose):
            return False
        with self._lock:
            navigating = [
                (peer, list(st.active_path))
                for peer, st in self._robots.items()
                if peer != code
                and st.mission_id is not None
                and st.phase == "NAVIGATING"
                and st.active_path
            ]
        for peer, path in navigating:
            try:
                peer_pose = self._cart_port.get_pose(peer)
            except Exception:
                peer_pose = None
            dense = self._densify_for_traffic(path)
            if not dense:
                continue
            # Remaining path from peer progress (not the already-cleared tail).
            start_idx = nearest_path_index(dense, peer_pose) if peer_pose else 0
            remaining = dense[max(0, start_idx) :]
            if len(remaining) < 2:
                remaining = dense[-2:] if len(dense) >= 2 else dense
            if pose_near_path(pose, remaining, self._clearance_m):
                return True
        return False

    def register_mission(self, device_code: str, mission_id: int) -> None:
        # Always record assignment (enabled() depends on active mission count).
        code = device_code.strip().lower()
        with self._lock:
            state = self._robots.setdefault(code, RobotTrafficState(device_code=code))
            state.mission_id = int(mission_id)
            # Keep the first assignment time (dispatch) for FIFO home-depart.
            if state.mission_assigned_at <= 0.0:
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
            state.hold_index = None
            state.planned_goal = None
            state.returning_home = False
            state.occupied_waypoint = None
            state.remaining_waypoints = []
            state.conflict_owner_sticky = None
            state.last_wait_reason = None
            state.wait_started_at = 0.0
            if self._home_depart_owner == code:
                self._home_depart_owner = None
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
                state.hold_index = None
                state.planned_goal = None
                state.returning_home = False
                state.occupied_waypoint = None
                state.remaining_waypoints = []
                state.conflict_owner_sticky = None
                state.last_wait_reason = None
                state.wait_started_at = 0.0
            if self._home_owner == code:
                self._home_owner = None
            if self._home_depart_owner == code:
                self._home_depart_owner = None
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
        """Atomically claim zone; False if another robot already holds it.

        W6 and C share one footprint — peer holding either blocks the other.
        Same robot may upgrade W6↔C without releasing (checkout after sandwich).
        """
        if not self.enabled():
            return True
        from ..waypoints import is_zone_occupiable, zones_overlap

        wid = (waypoint_id or "").strip().upper()
        if not is_zone_occupiable(wid):
            return True
        code = device_code.strip().lower()
        with self._lock:
            state = self._robots.setdefault(code, RobotTrafficState(device_code=code))
            # Already holding equivalent zone (W6→C or C→W6): just retarget claim.
            if zones_overlap(state.occupied_waypoint, wid):
                state.occupied_waypoint = wid
                self._cond.notify_all()
                return True
            for other_code, other_state in self._robots.items():
                if other_code == code:
                    continue
                if zones_overlap(other_state.occupied_waypoint, wid):
                    return False
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
        """Shelf ids to defer — only peers *currently occupying* that zone.

        Do NOT defer just because a peer still lists the shelf in
        ``remaining_waypoints`` (next work). That made carts wait for each
        other's entire tour even when paths never overlapped.
        Live path conflicts are handled by ``acquire_nav_leg``.
        """
        if not self.enabled():
            return set()
        code = device_code.strip().lower()
        targets = {s.strip().upper() for s in shelf_ids}
        defer: set[str] = set()
        from ..waypoints import zone_equivalent_ids

        with self._lock:
            for other_code, other_state in self._robots.items():
                if other_code == code or other_state.mission_id is None:
                    continue
                occ = other_state.occupied_waypoint
                if not occ:
                    continue
                occ_eq = zone_equivalent_ids(occ)
                for tid in targets:
                    if zone_equivalent_ids(tid) & occ_eq:
                        defer.add(tid)
        return defer

    def waypoint_access_granted(self, device_code: str, waypoint_id: str) -> bool:
        """True when this cart may claim/approach the waypoint zone.

        Only blocks on *real* zone use:
        - peer currently occupies the same waypoint zone, or
        - target is P and peer is returning home (shared corridor).

        Do NOT block just because the peer still lists this shelf in
        ``remaining_waypoints`` — that caused "충돌 대기" freezes while paths
        did not overlap and nobody was on the peer path.
        Spatial conflicts are handled by ``acquire_nav_leg`` / path grant;
        simultaneous claim races by ``try_claim_waypoint_zone``.
        """
        if not self.enabled():
            return True
        from ..waypoints import is_zone_occupiable, zones_overlap

        wid = (waypoint_id or "").strip().upper()
        if not is_zone_occupiable(wid):
            return True
        code = device_code.strip().lower()
        with self._lock:
            self_state = self._robots.get(code)
            # Already hold W6 or C → grant the other (same footprint).
            if self_state is not None and zones_overlap(
                self_state.occupied_waypoint, wid
            ):
                return True
            for other_code, other_state in self._robots.items():
                if other_code == code:
                    continue
                if wid == "P" and other_state.returning_home:
                    return False
                if other_state.mission_id is None:
                    continue
                if zones_overlap(other_state.occupied_waypoint, wid):
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
                    state.last_wait_reason = None
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

        last_replan = time.monotonic()
        replan_every = max(0.5, self._wait_replan_sec)
        wait_started: float | None = None
        yield_sec = max(5.0, self._yield_sec)

        while time.monotonic() < deadline:
            with self._lock:
                if int(self._abort_gen.get(code, 0)) != start_gen:
                    raise TrafficTimeoutError(f"traffic interrupted for {code}")

            now = time.monotonic()
            # Soft yield: blocked waiter frees the corridor for peer work / other orders.
            if wait_started is not None and (now - wait_started) >= yield_sec:
                with self._lock:
                    st = self._robots[code]
                    st.phase = "WAITING"
                    st.active_path = []
                    st.wait_started_at = 0.0
                    reason = st.last_wait_reason or "wait"
                    st.conflict_owner_sticky = None
                    for other in self._robots.values():
                        if other.device_code == code:
                            continue
                        if other.conflict_owner_sticky == code:
                            other.conflict_owner_sticky = None
                LOGGER.info(
                    "traffic yield %s after %.1fs (%s) -> (%.3f, %.3f)",
                    code,
                    yield_sec,
                    reason,
                    goal.x,
                    goal.y,
                )
                raise TrafficYieldError(
                    f"traffic yield for {code} after {yield_sec:.0f}s ({reason}) "
                    f"-> ({goal.x:.3f}, {goal.y:.3f})"
                )

            # While waiting (or after wait_blocked), refresh plan from live pose so
            # both carts can escape stale-path mutual freezes.
            if others_busy and (now - last_replan) >= replan_every:
                with self._lock:
                    self._robots[code].phase = "PLANNING"
                planned = self._plan_leg(code, goal)
                last_replan = time.monotonic()
                LOGGER.info(
                    "traffic replan while waiting %s -> (%.3f, %.3f) points=%d",
                    code,
                    goal.x,
                    goal.y,
                    len(planned),
                )

            other_paths = self._collect_other_paths(code)
            other_poses = self._collect_other_poses(code)
            try:
                other_poses[code] = self._cart_port.get_pose(code)
            except Exception:
                other_poses[code] = None

            # S1/S2: earlier-assigned mission leaves first; later cart stays on pad.
            home_ok, home_reason = self._try_claim_home_departure(
                code, other_poses.get(code), other_poses
            )
            if not home_ok:
                with self._lock:
                    state = self._robots[code]
                    state.phase = "WAITING"
                    state.active_path = []
                    state.last_wait_reason = home_reason
                # Stay put at S1/S2 — do not soft-yield / do not force W7.
                wait_started = None
                self._sleep_poll()
                continue

            with self._lock:
                grant, wait_reason = self._evaluate_leg_grant(
                    code, planned, other_paths, other_poses
                )
                if grant:
                    state = self._robots[code]
                    state.phase = "NAVIGATING"
                    state.active_path = list(planned)
                    state.last_wait_reason = None
                    state.wait_started_at = 0.0
                    self._cond.notify_all()
                    return
                state = self._robots[code]
                state.phase = "WAITING"
                # Do NOT reserve the planned path while WAITING — mutual reserved
                # paths caused both carts to freeze indefinitely.
                state.active_path = []
                state.last_wait_reason = wait_reason
                if wait_reason == "wait_home_depart":
                    wait_started = None
                elif wait_started is None:
                    wait_started = now
                    state.wait_started_at = now
            # Emergency: leave acquire loop so orders can stage W7 (비상대기).
            # Do not soft-yield / 충돌대기 here — that would stack with emergency.
            if wait_reason == "emergency_wait":
                if not self._pose_near_own_home(code, other_poses.get(code)):
                    try:
                        self._cart_port.stop_nav(code, freeze=False)
                    except TypeError:
                        try:
                            self._cart_port.stop_nav(code)
                        except Exception:
                            pass
                    except Exception:
                        pass
                raise TrafficEmergencyWaitError(
                    f"emergency_wait for {code} — stage at W7"
                )
            self._sleep_poll()
            if wait_reason in (
                "owner_passed",
                "wait_blocked",
                "wait_owner_idle",
                "wait_home_depart",
            ):
                # Force an immediate replan so we leave from the current pose.
                last_replan = 0.0
                continue
        LOGGER.warning(
            "traffic wait timed out for %s -> (%.3f, %.3f); conflict still active",
            code,
            goal.x,
            goal.y,
        )
        raise TrafficTimeoutError(
            f"traffic wait timed out for {code} -> ({goal.x:.3f}, {goal.y:.3f})"
        )

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
            state.hold_index = None
            state.planned_goal = None
            state.conflict_owner_sticky = None
            state.last_wait_reason = None
            state.wait_started_at = 0.0
            # Peers tagged emergency_wait for this owner must not stay frozen.
            for other in self._robots.values():
                if other.device_code == code:
                    continue
                if other.last_wait_reason == "emergency_wait":
                    other.last_wait_reason = None
                if other.conflict_owner_sticky == code:
                    other.conflict_owner_sticky = None
            self._cond.notify_all()
        self._release_home_depart_if_clear(code)

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
                    "holdIndex": st.hold_index,
                    "returningHome": st.returning_home,
                    "occupiedWaypoint": st.occupied_waypoint,
                    "remainingWaypoints": list(st.remaining_waypoints),
                    "conflictOwnerSticky": st.conflict_owner_sticky,
                    "lastWaitReason": st.last_wait_reason,
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
                "holdMarginM": self._hold_margin_m,
                "releaseMarginM": self._release_margin_m,
                "zoneRadiusM": self._zone_radius_m,
                "stagingWaypoint": self._staging_waypoint,
                "homeOwner": self._home_owner,
                "homeDepartOwner": self._home_depart_owner,
                "homeClearM": self._home_clear_m,
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

    def _robot_conflict_metrics(
        self,
        path: list[tuple[float, float]],
        pose: dict[str, float] | None,
        start_index: int,
        end_index: int,
    ) -> dict[str, Any]:
        current_index = nearest_path_index(path, pose)
        inside = start_index <= current_index <= end_index
        if current_index < start_index:
            distance_to_entry = path_distance_between(path, current_index, start_index)
        else:
            distance_to_entry = 0.0
        if current_index <= end_index:
            distance_to_exit = path_distance_between(path, current_index, end_index)
        else:
            distance_to_exit = 0.0
        release_index = advance_path_index(path, end_index, self._release_margin_m)
        available_release_margin = path_distance_between(path, end_index, release_index)
        can_clear = available_release_margin + 1e-6 >= self._release_margin_m
        return {
            "currentIndex": current_index,
            "entryIndex": start_index,
            "exitIndex": end_index,
            "releaseIndex": release_index,
            "insideConflict": inside,
            "distanceToEntryM": distance_to_entry,
            "distanceToExitM": distance_to_exit,
            "availableReleaseMarginM": available_release_margin,
            "canClearConflict": can_clear,
        }

    def _select_conflict_owner(
        self,
        self_code: str,
        other_code: str,
        self_metrics: dict[str, Any],
        other_metrics: dict[str, Any],
        self_state: RobotTrafficState,
        other_state: RobotTrafficState,
    ) -> str | None:
        selectable: list[str] = []
        if self_metrics["canClearConflict"]:
            selectable.append(self_code)
        if other_metrics["canClearConflict"]:
            selectable.append(other_code)
        if not selectable:
            return None

        sticky = self_state.conflict_owner_sticky or other_state.conflict_owner_sticky
        if sticky in selectable:
            # Prefer the cart that is actually driving over a sticky WAITING owner.
            if sticky == self_code and other_state.phase == "NAVIGATING" and (
                self_state.phase != "NAVIGATING"
            ):
                if other_code in selectable:
                    return other_code
            if sticky == other_code and self_state.phase == "NAVIGATING" and (
                other_state.phase != "NAVIGATING"
            ):
                if self_code in selectable:
                    return self_code
            return sticky

        # Prefer a peer already NAVIGATING so their order keeps progressing.
        if (
            other_state.phase == "NAVIGATING"
            and other_code in selectable
            and self_state.phase != "NAVIGATING"
        ):
            return other_code
        if (
            self_state.phase == "NAVIGATING"
            and self_code in selectable
            and other_state.phase != "NAVIGATING"
        ):
            return self_code

        if len(selectable) == 1:
            return selectable[0]

        if self_code not in selectable:
            return other_code
        if other_code not in selectable:
            return self_code

        if self_metrics["insideConflict"] != other_metrics["insideConflict"]:
            # Robot already sitting in the shared corridor waits in place;
            # the peer that still needs the corridor keeps working.
            return other_code if self_metrics["insideConflict"] else self_code

        if self_metrics["insideConflict"] and other_metrics["insideConflict"]:
            delta = float(self_metrics["distanceToExitM"]) - float(
                other_metrics["distanceToExitM"]
            )
            if abs(delta) > 1e-6:
                return self_code if delta < 0.0 else other_code
        else:
            delta = float(self_metrics["distanceToEntryM"]) - float(
                other_metrics["distanceToEntryM"]
            )
            if abs(delta) > 1e-6:
                return self_code if delta < 0.0 else other_code

        return (
            self_code
            if self_state.mission_assigned_at <= other_state.mission_assigned_at
            else other_code
        )

    def _densify_for_traffic(
        self, path: list[tuple[float, float]]
    ) -> list[tuple[float, float]]:
        return densify_path(path, step_m=min(self._clearance_m * 0.5, 0.05))

    def _evaluate_leg_grant(
        self,
        device_code: str,
        planned_path: list[tuple[float, float]],
        other_paths: dict[str, list[tuple[float, float]]],
        other_poses: dict[str, dict[str, float] | None],
    ) -> tuple[bool, str]:
        dense_planned = self._densify_for_traffic(planned_path)
        dense_others = {
            code: self._densify_for_traffic(path)
            for code, path in other_paths.items()
        }
        others = [
            (code, st)
            for code, st in self._robots.items()
            if code != device_code and st.mission_id is not None
        ]
        if not others:
            return True, "solo"

        self_state = self._robots[device_code]
        self_pose = other_poses.get(device_code)

        # Emergency wait (on peer remaining path) takes priority over zone/path
        # conflict waits — never stack "충돌대기" with "비상대기".
        for other_code, other_state in others:
            other_path = dense_others.get(other_code) or self._densify_for_traffic(
                other_paths.get(other_code) or list(other_state.active_path)
            )
            if not other_path:
                continue
            if other_state.phase == "NAVIGATING" and other_path:
                other_pose = other_poses.get(other_code)
                start_idx = (
                    nearest_path_index(other_path, other_pose) if other_pose else 0
                )
                remaining = other_path[max(0, start_idx) :]
                if len(remaining) < 2:
                    remaining = other_path[-2:] if len(other_path) >= 2 else other_path
                if pose_near_path(self_pose, remaining, self._clearance_m):
                    if self._pose_near_own_home(device_code, self_pose):
                        self_state.last_wait_reason = "wait_home_depart"
                        return False, "wait_home_depart"
                    sticky = (
                        self_state.conflict_owner_sticky
                        or other_state.conflict_owner_sticky
                    )
                    if sticky != device_code:
                        other_state.conflict_owner_sticky = other_code
                        self_state.conflict_owner_sticky = other_code
                        self_state.last_wait_reason = "emergency_wait"
                        return False, "emergency_wait"
            elif self_state.last_wait_reason == "emergency_wait":
                self_state.last_wait_reason = None

        occupied_zones = self.occupied_zones(exclude_device=device_code)
        # Only block when our *goal* sits in a peer-occupied zone — not when the
        # planned path merely grazes a distant shelf the peer is working.
        if occupied_zones and path_goal_in_zones(planned_path, occupied_zones):
            self_state.last_wait_reason = "wait_zone"
            return False, "wait_zone"

        for other_code, other_state in others:
            other_path = dense_others.get(other_code) or self._densify_for_traffic(
                other_paths.get(other_code) or list(other_state.active_path)
            )
            if not other_path:
                continue

            conflict = find_path_conflicts(
                dense_planned,
                other_path,
                clearance_m=self._clearance_m,
                max_samples=50,
            )
            segments = conflict.get("segments") or []
            if not segments:
                continue

            segment = segments[0]
            self_metrics = self._robot_conflict_metrics(
                dense_planned,
                self_pose,
                int(segment["cart1_start_index"]),
                int(segment["cart1_end_index"]),
            )
            other_metrics = self._robot_conflict_metrics(
                other_path,
                other_poses.get(other_code),
                int(segment["cart2_start_index"]),
                int(segment["cart2_end_index"]),
            )

            if not self_metrics["canClearConflict"] and not other_metrics["canClearConflict"]:
                # Stale sticky with neither able to clear → mutual WAIT; drop sticky
                # so the next replan cycle can re-pick once a path clears.
                self_state.conflict_owner_sticky = None
                other_state.conflict_owner_sticky = None
                self_state.last_wait_reason = "wait_blocked"
                return False, "wait_blocked"

            owner_code = self._select_conflict_owner(
                device_code,
                other_code,
                self_metrics,
                other_metrics,
                self_state,
                other_state,
            )
            if owner_code is None:
                self_state.conflict_owner_sticky = None
                other_state.conflict_owner_sticky = None
                self_state.last_wait_reason = "wait_blocked"
                return False, "wait_blocked"

            if owner_code == device_code:
                # Peer parked on our path: freeze them in place so we keep working
                # instead of either robot reversing to dodge.
                other_pose = other_poses.get(other_code)
                if pose_near_path(other_pose, dense_planned, self._clearance_m):
                    if self._pose_near_own_home(other_code, other_pose):
                        # Peer still on S1/S2 — leave them parked; do not cancel Nav2.
                        if other_state.phase == "NAVIGATING":
                            other_state.phase = "WAITING"
                        other_state.last_wait_reason = "wait_home_depart"
                    elif other_state.phase == "NAVIGATING":
                        try:
                            self._cart_port.stop_nav(other_code, freeze=False)
                        except TypeError:
                            try:
                                self._cart_port.stop_nav(other_code)
                            except Exception:
                                pass
                        except Exception:
                            pass
                        other_state.phase = "WAITING"
                        other_state.last_wait_reason = "emergency_wait"
                    else:
                        other_state.last_wait_reason = "emergency_wait"
                self_state.release_index = nearest_path_index(
                    planned_path,
                    {
                        "x": dense_planned[int(self_metrics["releaseIndex"])][0],
                        "y": dense_planned[int(self_metrics["releaseIndex"])][1],
                    },
                )
                self_state.conflict_owner_sticky = device_code
                other_state.conflict_owner_sticky = device_code
                self_state.hold_index = None
                self_state.last_wait_reason = None
                return True, "owner"

            owner_state = other_state
            owner_dense_path = other_path
            owner_raw_path = other_paths.get(other_code) or list(other_state.active_path)
            owner_pose = other_poses.get(other_code)
            if owner_state.phase == "IDLE":
                return True, "owner_done"
            # Sticky/selected owner is not actually driving — drop sticky and let
            # this cart take ownership if it can clear (avoids mutual WAIT freeze).
            if owner_state.phase != "NAVIGATING" or not owner_state.active_path:
                self_state.conflict_owner_sticky = None
                other_state.conflict_owner_sticky = None
                if self_metrics["canClearConflict"]:
                    other_pose = other_poses.get(other_code)
                    if pose_near_path(other_pose, dense_planned, self._clearance_m):
                        if self._pose_near_own_home(other_code, other_pose):
                            if other_state.phase == "NAVIGATING":
                                other_state.phase = "WAITING"
                            other_state.last_wait_reason = "wait_home_depart"
                        else:
                            try:
                                self._cart_port.stop_nav(other_code, freeze=False)
                            except TypeError:
                                try:
                                    self._cart_port.stop_nav(other_code)
                                except Exception:
                                    pass
                            except Exception:
                                pass
                            if other_state.phase == "NAVIGATING":
                                other_state.phase = "WAITING"
                            other_state.last_wait_reason = "emergency_wait"
                    self_state.release_index = nearest_path_index(
                        planned_path,
                        {
                            "x": dense_planned[int(self_metrics["releaseIndex"])][0],
                            "y": dense_planned[int(self_metrics["releaseIndex"])][1],
                        },
                    )
                    self_state.conflict_owner_sticky = device_code
                    other_state.conflict_owner_sticky = device_code
                    self_state.hold_index = None
                    self_state.last_wait_reason = None
                    return True, "owner_takeover"
                self_state.last_wait_reason = "wait_owner_idle"
                return False, "wait_owner_idle"

            release_idx = owner_state.release_index
            if release_idx is None:
                owner_conflict = find_path_conflicts(
                    owner_dense_path,
                    dense_planned,
                    clearance_m=self._clearance_m,
                    max_samples=10,
                )
                owner_segments = owner_conflict.get("segments") or []
                if owner_segments:
                    owner_segment = owner_segments[0]
                    owner_metrics = self._robot_conflict_metrics(
                        owner_dense_path,
                        owner_pose,
                        int(owner_segment["cart1_start_index"]),
                        int(owner_segment["cart1_end_index"]),
                    )
                    release_idx = nearest_path_index(
                        owner_raw_path,
                        {
                            "x": owner_dense_path[int(owner_metrics["releaseIndex"])][0],
                            "y": owner_dense_path[int(owner_metrics["releaseIndex"])][1],
                        },
                    )
                    owner_state.release_index = release_idx

            entry_index = nearest_path_index(
                planned_path,
                {
                    "x": dense_planned[int(self_metrics["entryIndex"])][0],
                    "y": dense_planned[int(self_metrics["entryIndex"])][1],
                },
            )
            hold_index = retreat_path_index(
                planned_path, entry_index, self._hold_margin_m
            )
            self_state.hold_index = hold_index
            self_state.last_wait_reason = "wait_owner"

            if has_passed_path_index(owner_raw_path, owner_pose, release_idx or 0):
                self_state.conflict_owner_sticky = None
                owner_state.conflict_owner_sticky = None
                return True, "owner_passed"
            return False, "wait_owner"

        self_state.hold_index = None
        self_state.last_wait_reason = None
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
        # WAITING/PLANNING reserved paths used to deadlock both carts; only the
        # actively navigating path (or empty) participates in conflict checks.
        if state.phase == "NAVIGATING":
            return list(state.active_path)
        return []

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
