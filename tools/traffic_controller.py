#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
import urllib.error
import urllib.request
import uuid
from dataclasses import dataclass, field
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from traffic_common import (
    advance_path_index,
    distance_to_point,
    find_path_conflicts,
    has_passed_path_index,
    path_heading,
    path_points,
    retreat_path_index,
)


@dataclass(frozen=True)
class Goal:
    x: float
    y: float
    yaw: float


@dataclass
class RobotContext:
    name: str
    base_url: str
    final_goal: Goal
    original_planned_path: list[tuple[float, float]] = field(default_factory=list)
    release_planned_path: list[tuple[float, float]] = field(default_factory=list)
    hold_goal: Goal | None = None
    phase: str = "INIT"
    command_sent: bool = False
    final_command_sent: bool = False
    last_valid_pose: dict[str, float] | None = None
    last_pose_time: float | None = None
    command_time: float | None = None
    start_hold: bool = False
    mock: bool = False
    mock_docked: bool = False
    approach_goal: Goal | None = None
    approach_command_sent: bool = False
    approach_command_time: float | None = None
    pose_history: list[tuple[float, float, float, float]] = field(default_factory=list)
    localization_violation_count: int = 0


class TrafficControllerError(RuntimeError):
    pass


class HttpResponseError(TrafficControllerError):
    """HTTP failure that preserves a parsed JSON error payload when available."""

    def __init__(
        self,
        status_code: int,
        url: str,
        payload: dict[str, Any] | None,
        raw_body: str,
    ) -> None:
        self.status_code = int(status_code)
        self.url = url
        self.payload = payload or {}
        self.raw_body = raw_body
        super().__init__(f"HTTP {self.status_code} {self.url}: {self.raw_body[:300]}")


class LocalizationLostError(TrafficControllerError):
    pass


class PathBlockedError(TrafficControllerError):
    """Planning retries were exhausted without a localization safety fault."""

    pass


DOCKING_GOALS = {
    "cart1": Goal(
        0.036703343955750284,
        0.0005066978948139312,
        0.009148818566518708,
    ),
    "cart2": Goal(
        0.038474577957370054,
        -0.1911947634013857,
        -0.02685422279113792,
    ),
}

# Both deployed Nav2 configs use the same physical footprint:
#   [[0.06, 0.06], [0.06, -0.06], [-0.06, -0.06], [-0.06, 0.06]]
# Keep these explicit and configurable from the CLI so a hardware footprint
# change cannot silently alter the docking safety model.
NAV2_FOOTPRINT_HALF_LENGTH_M = 0.06
NAV2_FOOTPRINT_HALF_WIDTH_M = 0.06
NAV2_FOOTPRINT_PADDING_M = 0.03


def validate_docking_goals(
    carts: dict[str, RobotContext],
    position_tolerance: float = 0.03,
    yaw_tolerance: float = 0.10,
) -> None:
    """Keep terminal release restricted to the approved CART-1/S1 + CART-2/S2 pair."""
    for key, expected in DOCKING_GOALS.items():
        actual = carts[key].final_goal
        yaw_error = abs(math.atan2(
            math.sin(actual.yaw - expected.yaw),
            math.cos(actual.yaw - expected.yaw),
        ))
        if (
            math.hypot(actual.x - expected.x, actual.y - expected.y) > position_tolerance
            or yaw_error > yaw_tolerance
        ):
            raise TrafficControllerError(
                f"--docking requires {key.upper()} final goal S{1 if key == 'cart1' else 2}; "
                f"got ({actual.x:.3f},{actual.y:.3f},yaw={actual.yaw:.3f})"
            )


def point_to_segment_distance(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    """Shortest Euclidean distance from point to a finite path segment."""
    px, py = point
    ax, ay = start
    bx, by = end
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    if length2 <= 1e-12:
        return math.hypot(px - ax, py - ay)
    ratio = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length2))
    return math.hypot(px - (ax + ratio * dx), py - (ay + ratio * dy))


def _box_axes(yaw: float) -> tuple[tuple[float, float], tuple[float, float]]:
    forward = (math.cos(yaw), math.sin(yaw))
    left = (-forward[1], forward[0])
    return forward, left


def _oriented_boxes_overlap(
    first: Goal,
    second: Goal,
    half_length: float,
    half_width: float,
    safety_margin: float,
) -> bool:
    """SAT overlap test for two identical rectangular robot footprints.

    ``safety_margin`` expands each robot independently, matching Nav2's
    per-footprint ``footprint_padding`` semantics.
    """
    expanded_length = float(half_length) + float(safety_margin)
    expanded_width = float(half_width) + float(safety_margin)
    first_axes = _box_axes(first.yaw)
    second_axes = _box_axes(second.yaw)
    delta = (second.x - first.x, second.y - first.y)
    for axis in (*first_axes, *second_axes):
        center_distance = abs(delta[0] * axis[0] + delta[1] * axis[1])
        first_radius = (
            expanded_length * abs(first_axes[0][0] * axis[0] + first_axes[0][1] * axis[1])
            + expanded_width * abs(first_axes[1][0] * axis[0] + first_axes[1][1] * axis[1])
        )
        second_radius = (
            expanded_length * abs(second_axes[0][0] * axis[0] + second_axes[0][1] * axis[1])
            + expanded_width * abs(second_axes[1][0] * axis[0] + second_axes[1][1] * axis[1])
        )
        if center_distance > first_radius + second_radius + 1e-9:
            return False
    return True


def _docking_swept_poses(
    path: list[tuple[float, float]],
    final_yaw: float,
    linear_step: float = 0.01,
    angular_step: float = math.radians(5.0),
):
    """Yield a conservative swept pose sequence for docking collision checks."""
    for index in range(len(path) - 1):
        x0, y0 = path[index]
        x1, y1 = path[index + 1]
        length = math.hypot(x1 - x0, y1 - y0)
        count = max(1, int(math.ceil(length / linear_step)))
        heading = math.atan2(y1 - y0, x1 - x0) if length > 1e-9 else path_heading(path, index)
        for sample in range(count):
            ratio = sample / count
            yield index, Goal(
                x0 + (x1 - x0) * ratio,
                y0 + (y1 - y0) * ratio,
                heading,
            )

    final_x, final_y = path[-1]
    approach_yaw = path_heading(path, len(path) - 1)
    turn = math.atan2(
        math.sin(final_yaw - approach_yaw),
        math.cos(final_yaw - approach_yaw),
    )
    turn_count = max(1, int(math.ceil(abs(turn) / angular_step)))
    for sample in range(turn_count + 1):
        ratio = sample / turn_count
        yield len(path) - 1, Goal(
            final_x,
            final_y,
            approach_yaw + turn * ratio,
        )


def check_approved_docking_path(
    owner_pose: Goal,
    waiter_path: list[tuple[float, float]],
    waiter_final_yaw: float,
    footprint_half_length: float,
    footprint_half_width: float,
    docking_margin: float,
) -> dict[str, Any]:
    """Check the moving footprint along a path against a parked footprint.

    Normal traffic still uses center-line clearance and reservation.  This
    function is only used after CART-1 is treated as PARKED in approved
    RETURN_HOME docking mode.
    """
    if len(waiter_path) < 2:
        return {"safe": False, "reason": "waiter path has fewer than two points"}
    if footprint_half_length <= 0.0 or footprint_half_width <= 0.0:
        return {"safe": False, "reason": "docking footprint dimensions must be positive"}
    if docking_margin < 0.0:
        return {"safe": False, "reason": "docking margin must not be negative"}

    minimum = minimum_path_separation(owner_pose, waiter_path)
    endpoint_distance = math.hypot(
        waiter_path[-1][0] - owner_pose.x,
        waiter_path[-1][1] - owner_pose.y,
    )
    collision_index: int | None = None
    for index, moving_pose in _docking_swept_poses(
        waiter_path, waiter_final_yaw
    ):
        if _oriented_boxes_overlap(
            owner_pose,
            moving_pose,
            footprint_half_length,
            footprint_half_width,
            docking_margin,
        ):
            collision_index = index
            break

    collision = collision_index is not None
    return {
        "safe": not collision,
        "reason": (
            "moving footprint overlaps parked footprint"
            if collision
            else "moving footprint clears parked footprint"
        ),
        "minimumDistanceM": minimum,
        "endpointDistanceM": endpoint_distance,
        "footprintCollision": collision,
        "collisionIndex": collision_index,
        "footprintHalfLengthM": float(footprint_half_length),
        "footprintHalfWidthM": float(footprint_half_width),
        "dockingMarginM": float(docking_margin),
    }


def print_docking_check(robot_name: str, check: dict[str, Any]) -> None:
    print(f"[DOCK] {robot_name} -> S2 path checked")
    print(
        f"[DOCK] min center separation = "
        f"{float(check.get('minimumDistanceM', math.inf)):.3f}m"
    )
    print(
        f"[DOCK] footprint collision = "
        f"{'true' if check.get('footprintCollision') else 'false'}"
    )
    print(f"[DOCK] {'docking permitted' if check.get('safe') else 'BLOCKED'}")


def minimum_path_separation(
    obstacle: Goal, path: list[tuple[float, float]]
) -> float:
    if len(path) < 2:
        return math.inf
    point = (obstacle.x, obstacle.y)
    return min(
        point_to_segment_distance(point, path[index], path[index + 1])
        for index in range(len(path) - 1)
    )


def yaw_error(actual: float, expected: float) -> float:
    return abs(math.atan2(math.sin(actual - expected), math.cos(actual - expected)))


def select_docking_route(
    owner_pose: Goal,
    waiter: RobotContext,
    plan_timeout: float,
    footprint_half_length: float,
    footprint_half_width: float,
    docking_margin: float,
) -> dict[str, Any]:
    """Replan direct first; consider the optional alternate only if direct is blocked."""
    print(f"[DIRECT REPLAN] {waiter.name} current pose -> S2")
    try:
        direct_path = plan_leg(waiter, waiter.final_goal, plan_timeout)
        direct_check = check_approved_docking_path(
            owner_pose,
            direct_path,
            waiter.final_goal.yaw,
            footprint_half_length,
            footprint_half_width,
            docking_margin,
        )
        print_docking_check(waiter.name, direct_check)
        if direct_check.get("safe"):
            return {
                "safe": True,
                "route": "DIRECT",
                "path": direct_path,
                "check": direct_check,
            }
        direct_reason = str(direct_check.get("reason", "direct path is unsafe"))
    except TrafficControllerError as exc:
        direct_path = None
        direct_check = None
        direct_reason = str(exc)
        print(f"[DIRECT PATH CHECK] BLOCKED: {direct_reason}")

    if waiter.approach_goal is None:
        print("[ALTERNATE PATH] no optional waypoint configured")
        return {
            "safe": False,
            "route": "WAIT",
            "reason": direct_reason,
            "directPath": direct_path,
            "directCheck": direct_check,
        }

    print_goal("[ALTERNATE PATH] waypoint", waiter.approach_goal)
    try:
        approach_path = plan_leg(waiter, waiter.approach_goal, plan_timeout)
        final_path = plan_leg(
            waiter, waiter.final_goal, plan_timeout, start=waiter.approach_goal
        )
        approach_check = check_approved_docking_path(
            owner_pose,
            approach_path,
            waiter.approach_goal.yaw,
            footprint_half_length,
            footprint_half_width,
            docking_margin,
        )
        final_check = check_approved_docking_path(
            owner_pose,
            final_path,
            waiter.final_goal.yaw,
            footprint_half_length,
            footprint_half_width,
            docking_margin,
        )
        print(
            f"[APPROACH PATH CHECK] minimum separation="
            f"{float(approach_check.get('minimumDistanceM', math.inf)):.3f}m "
            f"footprint collision="
            f"{'true' if approach_check.get('footprintCollision') else 'false'}"
        )
        print(
            f"[FINAL DOCK PATH CHECK] minimum separation="
            f"{float(final_check.get('minimumDistanceM', math.inf)):.3f}m "
            f"footprint collision="
            f"{'true' if final_check.get('footprintCollision') else 'false'}"
        )
        if approach_check.get("safe") and final_check.get("safe"):
            return {
                "safe": True,
                "route": "ALTERNATE",
                "path": approach_path + final_path[1:],
                "approachPath": approach_path,
                "finalPath": final_path,
                "check": final_check,
            }
        reason = (
            str(approach_check.get("reason", "alternate approach is unsafe"))
            if not approach_check.get("safe")
            else str(final_check.get("reason", "alternate final path is unsafe"))
        )
    except TrafficControllerError as exc:
        reason = str(exc)
        print(f"[ALTERNATE PATH CHECK] BLOCKED: {reason}")
    return {
        "safe": False,
        "route": "WAIT",
        "reason": reason,
        "directPath": direct_path,
        "directCheck": direct_check,
    }


class AuthorityPublisher(Node):
    """Publish immutable reference paths and authoritative traffic state."""

    def __init__(self, snapshot_provider) -> None:
        super().__init__("multi_robot_traffic_controller")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._path_pubs = {
            "cart1": self.create_publisher(
                Path, "/multi_robot/reference/cart1/path", qos
            ),
            "cart2": self.create_publisher(
                Path, "/multi_robot/reference/cart2/path", qos
            ),
        }
        self._state_pub = self.create_publisher(
            String, "/multi_robot/traffic_state", qos
        )
        self._snapshot_provider = snapshot_provider
        self._session_id = str(uuid.uuid4())
        self._sequence = 0
        self._last_fingerprint = ""
        self._last_publish = 0.0
        self.create_timer(0.1, self._timer)

    def publish_reference(self, cart: str, points: list[tuple[float, float]]) -> None:
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self._path_pubs[cart].publish(msg)

    def publish_state(self, active: bool = True, force: bool = False) -> None:
        snapshot = self._snapshot_provider()
        fingerprint = json.dumps(snapshot, sort_keys=True, separators=(",", ":"))
        now = time.monotonic()
        if not force and fingerprint == self._last_fingerprint and now - self._last_publish < 1.0:
            return
        self._sequence += 1
        snapshot.update(
            {
                "version": 1,
                "sessionId": self._session_id,
                "sequence": self._sequence,
                "stamp": time.time(),
                "controllerActive": bool(active),
            }
        )
        msg = String()
        msg.data = json.dumps(snapshot, separators=(",", ":"))
        self._state_pub.publish(msg)
        self._last_fingerprint = fingerprint
        self._last_publish = now

    def _timer(self) -> None:
        # Changed snapshots publish within 0.1 s; unchanged state heartbeats at 1 Hz.
        self.publish_state()


class RosPublisherRuntime:
    def __init__(self, snapshot_provider) -> None:
        rclpy.init()
        self.node = AuthorityPublisher(snapshot_provider)
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.thread.start()

    def close(self) -> None:
        try:
            if rclpy.ok():
                self.node.publish_state(active=False, force=True)
                time.sleep(0.15)
        finally:
            try:
                self.node.destroy_node()
            except Exception:
                pass
            if rclpy.ok():
                rclpy.shutdown()
            self.thread.join(timeout=1.0)


def request_json(
    method: str,
    url: str,
    body: dict[str, Any] | None = None,
    timeout: float = 5.0,
) -> dict[str, Any]:
    data = None
    headers = {"Accept": "application/json"}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    request = urllib.request.Request(url, data=data, headers=headers, method=method)
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            raw = response.read().decode("utf-8")
            return json.loads(raw) if raw else {}
    except urllib.error.HTTPError as exc:
        raw = exc.read().decode("utf-8", errors="replace")
        payload: dict[str, Any] | None = None
        if raw:
            try:
                parsed = json.loads(raw)
                if isinstance(parsed, dict):
                    payload = parsed
            except json.JSONDecodeError:
                pass
        raise HttpResponseError(exc.code, url, payload, raw) from exc
    except (urllib.error.URLError, TimeoutError, OSError, ValueError) as exc:
        raise TrafficControllerError(f"request failed {url}: {exc}") from exc


def goal_body(goal: Goal, timeout_sec: float | None = None) -> dict[str, float]:
    body: dict[str, float] = {"x": goal.x, "y": goal.y, "yaw": goal.yaw}
    if timeout_sec is not None:
        body["timeoutSec"] = timeout_sec
    return body


def plan_robot(robot: RobotContext, timeout_sec: float) -> None:
    robot.phase = "PLANNING"
    if robot.mock:
        # A docked test double occupies the approved S1 point.  It still enters
        # the normal conflict and stationary-owner safety calculations.
        robot.original_planned_path = [(robot.final_goal.x, robot.final_goal.y)]
        robot.phase = "PLANNED"
        return
    result = request_json(
        "POST",
        f"{robot.base_url}/nav/plan",
        goal_body(robot.final_goal, timeout_sec),
        timeout=timeout_sec + 3.0,
    )
    if not result.get("success"):
        robot.phase = "FAILED"
        raise TrafficControllerError(
            f"{robot.name} final plan failed: {result.get('message', result)}"
        )
    robot.original_planned_path = path_points(result)
    if not robot.original_planned_path:
        robot.phase = "FAILED"
        raise TrafficControllerError(f"{robot.name} final plan returned an empty path")
    robot.phase = "PLANNED"


def _wrapped_yaw_error(first: float, second: float) -> float:
    return abs(math.atan2(math.sin(first - second), math.cos(first - second)))


def _cart2_s2_undock_eligible(
    robot: RobotContext,
    *,
    position_tolerance: float,
    yaw_tolerance: float,
) -> tuple[bool, str]:
    if robot.name != "CART-2":
        return False, "recovery is restricted to CART-2"
    pose = robot.last_valid_pose
    if pose is None:
        return False, "current pose unavailable"
    s2 = DOCKING_GOALS["cart2"]
    position_error = math.hypot(float(pose["x"]) - s2.x, float(pose["y"]) - s2.y)
    yaw_error = _wrapped_yaw_error(float(pose.get("yaw", 0.0)), s2.yaw)
    if position_error > position_tolerance:
        return False, f"not near S2 (distance={position_error:.3f}m)"
    if yaw_error > yaw_tolerance:
        return False, f"S2 yaw mismatch ({yaw_error:.3f}rad)"
    # Never undock when S2 itself is the requested final goal (return-home mode).
    final_error = math.hypot(robot.final_goal.x - s2.x, robot.final_goal.y - s2.y)
    if final_error <= position_tolerance:
        return False, "final goal is S2; undock recovery is not applicable"
    return True, f"near S2 distance={position_error:.3f}m yawError={yaw_error:.3f}rad"


def _undock_start_pose(robot: RobotContext, distance_m: float) -> Goal:
    pose = robot.last_valid_pose
    if pose is None:
        raise TrafficControllerError(f"{robot.name}: current pose unavailable for undock")
    yaw = float(pose.get("yaw", 0.0))
    return Goal(
        float(pose["x"]) + float(distance_m) * math.cos(yaw),
        float(pose["y"]) + float(distance_m) * math.sin(yaw),
        yaw,
    )


def _undock_precheck_states(
    robot: RobotContext,
    other: RobotContext,
    http_timeout: float,
) -> None:
    for candidate in (robot, other):
        state = get_state(candidate, http_timeout)
        if state.get("navigating") is True:
            raise TrafficControllerError(
                f"{candidate.name}: navigating=true; undock recovery requires both robots stopped"
            )
        readiness = state.get("nav2Readiness") if isinstance(state, dict) else None
        readiness = readiness if isinstance(readiness, dict) else {}
        if readiness.get("ready") is not True:
            raise TrafficControllerError(
                f"{candidate.name}: Nav2 not ready for undock recovery; "
                f"failures={readiness.get('failures')!r}"
            )
        if candidate is robot:
            if readiness.get("tfValid") is not True or readiness.get("scanFresh") is not True:
                raise TrafficControllerError(
                    f"{candidate.name}: undock requires tfValid=true and scanFresh=true"
                )
            update_pose(candidate, state, time.monotonic())


def plan_robot_with_s2_undock_recovery(
    robot: RobotContext,
    other: RobotContext,
    args: argparse.Namespace,
    dry_run: bool,
) -> None:
    try:
        plan_robot(robot, args.plan_timeout)
        return
    except TrafficControllerError as first_error:
        if not args.s2_undock_recovery:
            raise
        eligible, reason = _cart2_s2_undock_eligible(
            robot,
            position_tolerance=args.undock_s2_position_tolerance,
            yaw_tolerance=args.undock_s2_yaw_tolerance,
        )
        if not eligible:
            raise TrafficControllerError(
                f"{robot.name} final plan failed and S2 undock is not eligible: "
                f"{reason}; original={first_error}"
            ) from first_error

        _undock_precheck_states(robot, other, args.http_timeout)
        print(f"[PLAN FAILED] {robot.name}: {first_error}")
        print(f"[S2 CHECK] {robot.name}: {reason}")
        print(f"[UNDOCK RECOVERY] {robot.name} eligible")
        print(
            f"[UNDOCK CONFIG] step={args.undock_step:.3f}m "
            f"max={args.undock_max_distance:.3f}m speed={args.undock_speed:.3f}m/s "
            f"retries={args.undock_retries}"
        )

        # Ask the robot-side motion primitive to perform all live scan/TF/odom
        # checks without moving. This also makes a dry-run useful after deployment.
        if not robot.mock:
            precheck = request_json(
                "POST",
                f"{robot.base_url}/nav/relative_move",
                {
                    "distanceM": min(args.undock_step, args.undock_max_distance),
                    "speedMps": args.undock_speed,
                    "dryRun": True,
                },
                timeout=max(args.http_timeout, 4.0),
            )
            print(
                f"[UNDOCK SAFETY] robot precheck success={bool(precheck.get('success'))} "
                f"clearance={precheck.get('clearanceM')}"
            )

        travelled = 0.0
        attempts = 0
        while attempts < args.undock_retries and travelled + 1e-9 < args.undock_max_distance:
            step = min(args.undock_step, args.undock_max_distance - travelled)
            attempts += 1
            if dry_run:
                predicted_distance = travelled + step
                predicted_start = _undock_start_pose(robot, predicted_distance)
                print(
                    f"[DRY-RUN UNDOCK] step {attempts}/{args.undock_retries}: "
                    f"would move {step:.3f}m forward; total={predicted_distance:.3f}m"
                )
                try:
                    points = plan_leg(
                        robot, robot.final_goal, args.plan_timeout, predicted_start
                    )
                except TrafficControllerError as exc:
                    print(
                        f"[DRY-RUN REPLAN] hypothetical +{predicted_distance:.3f}m "
                        f"still blocked: {exc}"
                    )
                    travelled = predicted_distance
                    continue
                robot.original_planned_path = points
                robot.phase = "PLANNED"
                print(
                    f"[DRY-RUN REPLAN OK] {robot.name}: hypothetical undock "
                    f"{predicted_distance:.3f}m -> {len(points)} points"
                )
                print("[DRY-RUN] no cmd_vel, /nav/goal, or /nav/stop was sent")
                return

            timeout_sec = max(2.5, step / args.undock_speed + 2.0)
            print(
                f"[UNDOCK STEP] {robot.name} {step:.3f}m forward "
                f"({attempts}/{args.undock_retries})"
            )
            result = request_json(
                "POST",
                f"{robot.base_url}/nav/relative_move",
                {
                    "distanceM": step,
                    "speedMps": args.undock_speed,
                    "timeoutSec": timeout_sec,
                    "dryRun": False,
                },
                timeout=timeout_sec + 3.0,
            )
            if result.get("success") is not True:
                raise TrafficControllerError(
                    f"{robot.name} undock step failed: {result.get('message', result)}"
                )
            travelled += max(0.0, float(result.get("movedM", step)))
            print(
                f"[UNDOCK STOPPED] {robot.name} moved={travelled:.3f}m; replanning"
            )
            time.sleep(0.15)
            state = get_state(robot, args.http_timeout)
            update_pose(robot, state, time.monotonic())
            try:
                plan_robot(robot, args.plan_timeout)
            except TrafficControllerError as exc:
                print(
                    f"[UNDOCK REPLAN BLOCKED] {robot.name} after {travelled:.3f}m: {exc}"
                )
                continue
            print(
                f"[UNDOCK COMPLETE] {robot.name} after {travelled:.3f}m; "
                f"plan={len(robot.original_planned_path)} points"
            )
            return

        raise PathBlockedError(
            f"{robot.name} remains trapped after undock recovery "
            f"({travelled:.3f}m, {attempts} steps)"
        )


def plan_leg(
    robot: RobotContext,
    goal: Goal,
    timeout_sec: float,
    start: Goal | None = None,
) -> list[tuple[float, float]]:
    body = goal_body(goal, timeout_sec)
    if start is not None:
        body.update({"startX": start.x, "startY": start.y, "startYaw": start.yaw})
    result = request_json(
        "POST", f"{robot.base_url}/nav/plan", body, timeout=timeout_sec + 3.0
    )
    points = path_points(result)
    if not result.get("success") or not points:
        raise TrafficControllerError(
            f"{robot.name} path leg planning failed: {result.get('message', result)}"
        )
    endpoint_error = math.hypot(points[-1][0] - goal.x, points[-1][1] - goal.y)
    if endpoint_error > 0.05:
        raise TrafficControllerError(
            f"{robot.name} path leg stopped {endpoint_error:.3f}m short of its "
            "requested goal; refusing an incomplete docking safety check"
        )
    return points


def validate_hold_plan(robot: RobotContext, timeout_sec: float) -> None:
    if robot.hold_goal is None:
        raise TrafficControllerError(f"{robot.name} hold goal is missing")
    result = request_json(
        "POST",
        f"{robot.base_url}/nav/plan",
        goal_body(robot.hold_goal, timeout_sec),
        timeout=timeout_sec + 3.0,
    )
    if not result.get("success") or not path_points(result):
        robot.phase = "FAILED"
        raise TrafficControllerError(
            f"{robot.name} HOLD plan failed: {result.get('message', result)}"
        )


def prepare_start_hold(robot: RobotContext, timeout_sec: float) -> None:
    """Use a stationary robot's current pose as its hold point without commanding it."""
    state = get_state(robot, timeout_sec)
    now = time.monotonic()
    if not update_pose(robot, state, now) or robot.last_valid_pose is None:
        robot.phase = "FAILED"
        raise TrafficControllerError(
            f"{robot.name} pose is null; START_HOLD is not safe"
        )
    if navigating_state(state, robot.name):
        robot.phase = "FAILED"
        raise TrafficControllerError(
            f"{robot.name} is navigating; START_HOLD requires navigating=false"
        )
    pose = robot.last_valid_pose
    robot.hold_goal = Goal(pose["x"], pose["y"], pose.get("yaw", 0.0))
    robot.start_hold = True
    robot.phase = "START_HOLD"


def get_state(robot: RobotContext, timeout_sec: float) -> dict[str, Any]:
    if robot.mock:
        if not robot.mock_docked:
            raise TrafficControllerError(
                f"{robot.name} mock state is unavailable; use --mock-cart1-docked"
            )
        return {
            "pose": {
                "x": robot.final_goal.x,
                "y": robot.final_goal.y,
                "yaw": robot.final_goal.yaw,
            },
            "navigating": False,
            "mock": True,
            "navigationAction": {"state": "SUCCEEDED", "goalId": "mock-docked"},
            "nav2Readiness": {"ready": True, "failures": []},
        }
    return request_json("GET", f"{robot.base_url}/nav/state", timeout=timeout_sec)


def update_pose(robot: RobotContext, state: dict[str, Any], now: float) -> bool:
    pose = state.get("pose") if isinstance(state, dict) else None
    if not isinstance(pose, dict):
        return False
    try:
        robot.last_valid_pose = {
            "x": float(pose["x"]),
            "y": float(pose["y"]),
            "yaw": float(pose.get("yaw", 0.0)),
        }
    except (KeyError, TypeError, ValueError):
        return False
    robot.last_pose_time = now
    robot.pose_history.append(
        (now, robot.last_valid_pose["x"], robot.last_valid_pose["y"], robot.last_valid_pose["yaw"])
    )
    robot.pose_history = [sample for sample in robot.pose_history if now - sample[0] <= 5.0]
    return True


def validate_localization_state(
    robot: RobotContext,
    state: dict[str, Any],
    now: float,
    robot_max_speed: float,
    localization_speed_factor: float = 3.0,
    localization_violation_count: int = 3,
    localization_hard_jump: float = 0.5,
) -> None:
    """Accept a live pose only while the minimum localization signals are healthy."""
    readiness = state.get("nav2Readiness") if isinstance(state, dict) else None
    readiness = readiness if isinstance(readiness, dict) else {}
    ready = readiness.get("ready")
    tf_valid = readiness.get("tfValid")
    scan_fresh = readiness.get("scanFresh")
    if ready is not True or tf_valid is not True or scan_fresh is not True:
        raise LocalizationLostError(
            f"{robot.name}: ready={ready!r} tfValid={tf_valid!r} "
            f"scanFresh={scan_fresh!r} failures={readiness.get('failures')!r}"
        )

    previous_pose = robot.last_valid_pose
    previous_time = robot.last_pose_time
    pose = state.get("pose") if isinstance(state, dict) else None
    if not isinstance(pose, dict):
        raise LocalizationLostError(f"{robot.name}: AMCL pose is unavailable")
    try:
        current_x = float(pose["x"])
        current_y = float(pose["y"])
    except (KeyError, TypeError, ValueError) as exc:
        raise LocalizationLostError(f"{robot.name}: AMCL pose is invalid") from exc

    if previous_pose is not None and previous_time is not None:
        elapsed = now - previous_time
        if elapsed > 0.0:
            displacement = math.hypot(
                current_x - previous_pose["x"], current_y - previous_pose["y"]
            )
            implied_speed = displacement / elapsed
            if displacement > localization_hard_jump:
                robot.localization_violation_count = 0
                print(
                    f"[LOCALIZATION HARD JUMP] {robot.name}: "
                    f"{displacement:.3f}m in {elapsed:.3f}s"
                )
                raise LocalizationLostError(
                    f"{robot.name}: AMCL hard jump {displacement:.3f}m exceeds "
                    f"{localization_hard_jump:.3f}m"
                )
            soft_speed_limit = robot_max_speed * localization_speed_factor
            if implied_speed > soft_speed_limit:
                robot.localization_violation_count += 1
                print(
                    f"[LOCALIZATION WARNING] {robot.name}: implied speed "
                    f"{implied_speed:.3f}m/s > soft limit "
                    f"{soft_speed_limit:.3f}m/s "
                    f"({robot.localization_violation_count}/"
                    f"{localization_violation_count})"
                )
                if robot.localization_violation_count >= localization_violation_count:
                    raise LocalizationLostError(
                        f"{robot.name}: AMCL soft-limit violation repeated "
                        f"{robot.localization_violation_count} times"
                    )
            else:
                robot.localization_violation_count = 0
    if not update_pose(robot, state, now):
        raise LocalizationLostError(f"{robot.name}: AMCL pose is unavailable")


def stop_all_after_localization_loss(
    robots: list[RobotContext], http_timeout: float
) -> None:
    print("[LOCALIZATION LOST]")
    print("[SAFETY FAULT] localization/TF/sensor state is invalid")
    print("[STOP BOTH]")
    for robot in robots:
        if robot.mock:
            continue
        try:
            request_json(
                "POST", f"{robot.base_url}/nav/stop", {}, timeout=http_timeout
            )
            print(f"[STOP] {robot.name}")
        except TrafficControllerError as exc:
            print(f"[STOP FAILED] {robot.name}: {exc}")


def transit_goal(robot: RobotContext) -> Goal:
    """Keep the final position, but approach transit waypoints without a final spin."""
    path = robot.release_planned_path or robot.original_planned_path
    if len(path) < 2:
        return robot.final_goal
    return Goal(robot.final_goal.x, robot.final_goal.y, path_heading(path, len(path) - 1))


def navigation_action_state(state: dict[str, Any]) -> str:
    action = state.get("navigationAction") if isinstance(state, dict) else None
    value = action.get("state") if isinstance(action, dict) else None
    return str(value).upper() if value else "UNKNOWN"


def docking_completion_ready(
    pose: dict[str, float] | None,
    goal: Goal,
    position_tolerance: float,
    yaw_tolerance: float,
    pose_stable: bool,
    action_state: str,
    navigating: bool,
) -> bool:
    """Pure safety gate used before a docked owner can release its waiter."""
    if pose is None:
        return False
    position_ok = distance_to_point(pose, (goal.x, goal.y)) <= position_tolerance
    yaw_ok = yaw_error(float(pose.get("yaw", 0.0)), goal.yaw) <= yaw_tolerance
    return bool(
        position_ok
        and yaw_ok
        and pose_stable
        and str(action_state).upper() == "SUCCEEDED"
        and not navigating
    )


def pose_history_stable(
    robot: RobotContext,
    now: float,
    duration: float = 2.0,
    position_variation: float = 0.02,
    yaw_variation: float = 0.05,
) -> bool:
    samples = [sample for sample in robot.pose_history if now - sample[0] <= duration]
    if len(samples) < 2 or samples[0][0] > now - duration + 0.15:
        return False
    anchor = samples[0]
    return all(
        math.hypot(sample[1] - anchor[1], sample[2] - anchor[2]) <= position_variation
        and yaw_error(sample[3], anchor[3]) <= yaw_variation
        for sample in samples[1:]
    )


def wait_for_stable_poses(
    robots: list[RobotContext],
    duration: float,
    timeout: float,
    poll_hz: float,
    http_timeout: float,
    robot_max_speed: float,
    localization_speed_factor: float,
    localization_violation_count: int,
    localization_hard_jump: float,
    position_variation: float = 0.02,
    yaw_variation: float = 0.05,
) -> None:
    pending = [robot for robot in robots if not robot.mock]
    if not pending:
        return
    for robot in pending:
        robot.pose_history.clear()
    deadline = time.monotonic() + timeout
    warned = False
    while time.monotonic() < deadline:
        now = time.monotonic()
        all_stable = True
        for robot in pending:
            state = get_state(robot, http_timeout)
            if navigating_state(state, robot.name):
                robot.pose_history.clear()
                all_stable = False
                continue
            validate_localization_state(
                robot, state, now, robot_max_speed,
                localization_speed_factor, localization_violation_count,
                localization_hard_jump,
            )
            if not pose_history_stable(
                robot, now, duration, position_variation, yaw_variation
            ):
                all_stable = False
        if all_stable:
            for robot in pending:
                print(f"[POSE STABLE] {robot.name}")
            return
        if not warned:
            print("[POSE UNSTABLE] waiting for localization convergence")
            warned = True
        time.sleep(1.0 / poll_hz)
    raise TrafficControllerError(
        f"pose stability timeout after {timeout:.1f}s; no planning or goal command"
    )


def navigating_state(state: dict[str, Any], robot_name: str) -> bool:
    """Return a trustworthy navigation flag; missing/invalid is never treated as idle."""
    navigating = state.get("navigating") if isinstance(state, dict) else None
    if not isinstance(navigating, bool):
        raise TrafficControllerError(
            f"{robot_name} navigating state is missing or invalid; no release"
        )
    return navigating


def release_time_replan(
    waiter: RobotContext,
    state: dict[str, Any],
    plan_timeout: float,
    replan_retries: int = 3,
    replan_interval: float = 1.0,
    http_timeout: float = 3.0,
    robot_max_speed: float | None = None,
    localization_speed_factor: float = 3.0,
    localization_violation_count: int = 3,
    localization_hard_jump: float = 0.5,
) -> list[tuple[float, float]]:
    """Validate live waiter state and plan its actual post-release route.

    The immutable initial paths remain the traffic-scheduling authority.  They
    are deliberately not reused as proof that the waiter can drive after the
    owner has cleared the reservation.
    """
    print(f"[RELEASE CHECK] {waiter.name}")
    attempts = max(1, int(replan_retries))
    last_error: TrafficControllerError | None = None
    current_state = state
    for attempt in range(1, attempts + 1):
        if attempt > 1:
            print(
                f"[REPLAN RETRY] temporary planning failure; "
                f"WAIT {replan_interval:.2f}s"
            )
            time.sleep(max(0.0, replan_interval))
            current_state = get_state(waiter, http_timeout)

        readiness = (
            current_state.get("nav2Readiness")
            if isinstance(current_state, dict)
            else None
        )
        readiness = readiness if isinstance(readiness, dict) else {}
        ready = readiness.get("ready")
        tf_valid = readiness.get("tfValid")
        scan_fresh = readiness.get("scanFresh")
        navigating = current_state.get("navigating")
        print(
            f"[RELEASE STATE] ready={str(ready).lower()} "
            f"tfValid={str(tf_valid).lower()} "
            f"scanFresh={str(scan_fresh).lower()} "
            f"navigating={str(navigating).lower()}"
        )
        if ready is not True or tf_valid is not True or scan_fresh is not True:
            raise LocalizationLostError(
                f"{waiter.name}: release readiness invalid: ready={ready!r} "
                f"tfValid={tf_valid!r} scanFresh={scan_fresh!r} "
                f"failures={readiness.get('failures')!r}"
            )
        if navigating_state(current_state, waiter.name):
            last_error = TrafficControllerError(
                f"{waiter.name} is unexpectedly navigating while held"
            )
            continue

        now = time.monotonic()
        if robot_max_speed is not None:
            validate_localization_state(
                waiter, current_state, now, robot_max_speed,
                localization_speed_factor, localization_violation_count,
                localization_hard_jump,
            )
        elif not update_pose(waiter, current_state, now):
            raise LocalizationLostError(
                f"{waiter.name}: AMCL pose unavailable at release"
            )
        pose = waiter.last_valid_pose
        if pose is None:
            raise LocalizationLostError(
                f"{waiter.name}: AMCL pose unavailable at release"
            )
        print(
            f"[RELEASE POSE] {waiter.name} "
            f"x={pose['x']:.3f} y={pose['y']:.3f} yaw={pose['yaw']:.3f}"
        )
        print(f"[REPLAN] {waiter.name} attempt {attempt}/{attempts}")
        try:
            path = plan_leg(waiter, waiter.final_goal, plan_timeout)
        except TrafficControllerError as exc:
            last_error = exc
            print(f"[TEMPORARY PLAN BLOCKED] {exc}")
            continue

        waiter.release_planned_path = path
        print(f"[REPLAN OK] pointCount={len(path)}")
        return path

    print(f"[PATH BLOCKED] {waiter.name} remains WAIT; goal was NOT sent")
    raise PathBlockedError(
        f"{waiter.name} release replan failed after {attempts} attempts: {last_error}"
    )


def send_goal_once(robot: RobotContext, goal: Goal, final: bool, timeout_sec: float) -> None:
    if robot.mock:
        raise TrafficControllerError(
            f"refusing to send a navigation command to mock {robot.name}"
        )
    already_sent = robot.final_command_sent if final else robot.command_sent
    if already_sent:
        return
    result = request_json(
        "POST",
        f"{robot.base_url}/nav/goal",
        goal_body(goal),
        timeout=timeout_sec,
    )
    if not result.get("success"):
        robot.phase = "FAILED"
        raise TrafficControllerError(
            f"{robot.name} goal command failed: {result.get('message', result)}"
        )
    robot.command_time = time.monotonic()
    if final:
        robot.final_command_sent = True
        robot.phase = "TO_FINAL"
    else:
        robot.command_sent = True
        robot.phase = "TO_HOLD"


def send_approach_once(robot: RobotContext, timeout_sec: float) -> None:
    if robot.approach_command_sent:
        return
    if robot.approach_goal is None:
        raise TrafficControllerError(f"{robot.name} approach goal is missing")
    result = request_json(
        "POST",
        f"{robot.base_url}/nav/goal",
        goal_body(robot.approach_goal),
        timeout=timeout_sec,
    )
    if not result.get("success"):
        robot.phase = "FAILED"
        raise TrafficControllerError(
            f"{robot.name} approach command failed: {result.get('message', result)}"
        )
    robot.approach_command_sent = True
    robot.approach_command_time = time.monotonic()
    robot.phase = "TO_APPROACH"


def wait_for_home_hold(
    waiter: RobotContext,
    poll_hz: float,
    pose_timeout: float,
    hold_tolerance: float,
    http_timeout: float,
    mission_timeout: float,
) -> None:
    """Complete CART-2 HOLD before CART-1 is allowed to move home."""
    if waiter.hold_goal is None:
        raise TrafficControllerError(f"{waiter.name} HOME HOLD goal is missing")
    started = time.monotonic()
    while True:
        now = time.monotonic()
        if now - started > mission_timeout:
            raise TrafficControllerError(f"{waiter.name} HOME HOLD timeout")
        state = get_state(waiter, http_timeout)
        if not update_pose(waiter, state, now) or waiter.last_pose_time is None:
            raise TrafficControllerError(f"{waiter.name} HOME HOLD pose is unavailable")
        if now - waiter.last_pose_time > pose_timeout:
            raise TrafficControllerError(f"{waiter.name} HOME HOLD pose timed out")
        navigating = navigating_state(state, waiter.name)
        action_state = navigation_action_state(state)
        at_hold = (
            distance_to_point(
                waiter.last_valid_pose, (waiter.hold_goal.x, waiter.hold_goal.y)
            ) <= hold_tolerance
        )
        if at_hold and not navigating and action_state == "SUCCEEDED":
            waiter.phase = "HOLDING"
            print(f"[STATE] {waiter.name}: HOLDING")
            return
        if action_state in ("ABORTED", "CANCELED"):
            waiter.phase = "FAILED"
            raise TrafficControllerError(
                f"{waiter.name} HOME HOLD navigation {action_state}"
            )
        if (
            waiter.command_time is not None
            and now - waiter.command_time > 2.0
            and not navigating
            and not at_hold
        ):
            waiter.phase = "FAILED"
            raise TrafficControllerError(
                f"{waiter.name} stopped before HOME HOLD; CART-1 remains stopped"
            )
        time.sleep(1.0 / poll_hz)


def print_goal(label: str, goal: Goal) -> None:
    print(f"{label}: x={goal.x:.3f}, y={goal.y:.3f}, yaw={goal.yaw:.3f} rad")


def path_distance_between(
    path: list[tuple[float, float]], start_index: int, end_index: int
) -> float:
    low, high = sorted((int(start_index), int(end_index)))
    return sum(
        math.hypot(path[index + 1][0] - path[index][0],
                   path[index + 1][1] - path[index][1])
        for index in range(low, high)
    )


def controller_loop(
    owner: RobotContext,
    waiter: RobotContext | None,
    release_index: int | None,
    poll_hz: float,
    pose_timeout: float,
    hold_tolerance: float,
    goal_tolerance: float,
    http_timeout: float,
    mission_timeout: float,
    release_margin: float = 0.20,
    terminal_docking: bool = False,
    plan_timeout: float = 10.0,
    footprint_half_length: float = NAV2_FOOTPRINT_HALF_LENGTH_M,
    footprint_half_width: float = NAV2_FOOTPRINT_HALF_WIDTH_M,
    docking_margin: float = NAV2_FOOTPRINT_PADDING_M,
    docking_yaw_tolerance: float = 0.15,
    docking_status_callback=None,
    docking_path_callback=None,
    robot_max_speed: float = 0.0,
    replan_retries: int = 3,
    replan_interval: float = 1.0,
    localization_speed_factor: float = 3.0,
    localization_violation_count: int = 3,
    localization_hard_jump: float = 0.5,
) -> None:
    started = time.monotonic()
    grace_sec = 2.0
    docking_result_logged = False
    docking_route: str | None = None
    last_docking_replan = -math.inf
    last_final_dock_replan = -math.inf
    docking_replan_interval = 2.0
    while True:
        now = time.monotonic()
        if now - started > mission_timeout:
            raise TrafficControllerError("mission timeout; reservation remains unreleased")

        contexts = [owner] + ([waiter] if waiter is not None else [])
        states: dict[str, dict[str, Any]] = {}
        try:
            for robot in contexts:
                state = get_state(robot, http_timeout)
                states[robot.name] = state
                validate_localization_state(
                    robot, state, now, robot_max_speed,
                    localization_speed_factor, localization_violation_count,
                    localization_hard_jump,
                )
        except LocalizationLostError as exc:
            stop_all_after_localization_loss(contexts, http_timeout)
            raise LocalizationLostError(str(exc)) from exc

        owner_state = states[owner.name]
        owner_navigating = navigating_state(owner_state, owner.name)
        owner_action_state = navigation_action_state(owner_state)
        owner_at_goal = (
            distance_to_point(owner.last_valid_pose, (owner.final_goal.x, owner.final_goal.y))
            <= goal_tolerance
        )
        owner_at_docking_yaw = (
            owner.last_valid_pose is not None
            and yaw_error(
                float(owner.last_valid_pose.get("yaw", 0.0)), owner.final_goal.yaw
            ) <= docking_yaw_tolerance
        )
        owner_pose_stable = pose_history_stable(owner, now)

        if waiter is None:
            if owner_at_goal and not owner_navigating:
                owner.phase = "COMPLETE"
                return
            if (
                owner.command_time is not None
                and now - owner.command_time > grace_sec
                and not owner_navigating
                and not owner_at_goal
            ):
                owner.phase = "FAILED"
                raise TrafficControllerError(f"{owner.name} navigation stopped before final goal")
            time.sleep(1.0 / poll_hz)
            continue

        waiter_state = states[waiter.name]
        waiter_navigating = navigating_state(waiter_state, waiter.name)
        if waiter.phase == "START_HOLD":
            if waiter.hold_goal is None:
                waiter.phase = "FAILED"
                raise TrafficControllerError(
                    f"{waiter.name} START_HOLD pose is missing; no release"
                )
            stayed_at_start = (
                distance_to_point(
                    waiter.last_valid_pose, (waiter.hold_goal.x, waiter.hold_goal.y)
                )
                <= hold_tolerance
            )
            if waiter_navigating or not stayed_at_start:
                waiter.phase = "FAILED"
                raise TrafficControllerError(
                    f"{waiter.name} moved during START_HOLD; reservation retained"
                )
        elif waiter.phase == "TO_HOLD" and waiter.hold_goal is not None:
            at_hold = (
                distance_to_point(
                    waiter.last_valid_pose, (waiter.hold_goal.x, waiter.hold_goal.y)
                )
                <= hold_tolerance
            )
            if at_hold and not waiter_navigating:
                waiter.phase = "HOLDING"
                print(f"[STATE] {waiter.name}: HOLDING")
            elif (
                waiter.command_time is not None
                and now - waiter.command_time > grace_sec
                and not waiter_navigating
            ):
                waiter.phase = "FAILED"
                raise TrafficControllerError(
                    f"{waiter.name} stopped before reaching HOLD; no release"
                )

        if terminal_docking:
            # A terminal conflict has no safe point beyond EXIT.  Only a verified,
            # stationary arrival at the configured dock releases the waiter.
            owner_cleared = docking_completion_ready(
                owner.last_valid_pose,
                owner.final_goal,
                goal_tolerance,
                docking_yaw_tolerance,
                owner_pose_stable,
                owner_action_state,
                owner_navigating,
            )
            if owner_cleared:
                owner.phase = "DOCKED"
                if not docking_result_logged:
                    print(f"[DOCKED] {owner.name}")
                    print(f"[HOME] {owner.name} PARKED")
                    docking_result_logged = True
                if (
                    waiter.phase in ("START_HOLD", "HOLDING")
                    and not waiter.final_command_sent
                    and now - last_docking_replan >= docking_replan_interval
                ):
                    actual_owner = owner.last_valid_pose or {}
                    actual_owner_goal = Goal(
                        float(actual_owner["x"]),
                        float(actual_owner["y"]),
                        float(actual_owner.get("yaw", 0.0)),
                    )
                    decision = select_docking_route(
                        actual_owner_goal,
                        waiter,
                        plan_timeout,
                        footprint_half_length,
                        footprint_half_width,
                        docking_margin,
                    )
                    last_docking_replan = now
                    if decision.get("safe"):
                        docking_route = str(decision["route"])
                        print(f"[DOCKING ROUTE] {docking_route} SAFE")
                        if docking_status_callback:
                            docking_status_callback(f"{docking_route} SAFE")
                        if docking_path_callback:
                            docking_path_callback(decision.get("path") or [])
                    else:
                        docking_route = None
                        print("[DOCKING BLOCKED] CART-2 remains WAIT")
                        if docking_status_callback:
                            docking_status_callback("BLOCKED")
        else:
            if release_index is None:
                raise TrafficControllerError("release index is missing")
            owner_cleared = has_passed_path_index(
                owner.original_planned_path,
                owner.last_valid_pose,
                release_index,
            )
        if (
            owner.command_time is not None
            and now - owner.command_time > grace_sec
            and not owner_navigating
            and not owner_cleared
        ):
            owner.phase = "FAILED"
            raise TrafficControllerError(
                f"{owner.name} navigation stopped before verified release "
                f"(action={owner_action_state}); reservation retained"
            )

        if (
            owner_cleared
            and (not terminal_docking or docking_route is not None)
            and waiter.phase in ("START_HOLD", "HOLDING")
            and not waiter.final_command_sent
        ):
            if terminal_docking:
                print(f"[RELEASE] {waiter.name} route={docking_route}")
            else:
                print(
                    f"[EXIT] {owner.name} passed conflict EXIT + "
                    f"{release_margin:.2f}m"
                )
                try:
                    release_time_replan(
                        waiter,
                        waiter_state,
                        plan_timeout,
                        replan_retries,
                        replan_interval,
                        http_timeout,
                        robot_max_speed,
                        localization_speed_factor,
                        localization_violation_count,
                        localization_hard_jump,
                    )
                except LocalizationLostError as exc:
                    stop_all_after_localization_loss(contexts, http_timeout)
                    raise LocalizationLostError(str(exc)) from exc
            waiter.phase = "RELEASED"
            if terminal_docking and docking_route == "ALTERNATE":
                send_approach_once(waiter, http_timeout)
                print(f"[COMMAND] {waiter.name} -> DOCK APPROACH")
            else:
                release_goal = (
                    waiter.final_goal if terminal_docking else transit_goal(waiter)
                )
                send_goal_once(waiter, release_goal, final=True, timeout_sec=http_timeout)
                if terminal_docking:
                    waiter.phase = "DOCKING"
                    print(f"[HOME] {waiter.name} -> S2")
                else:
                    print(f"[RELEASE] {waiter.name}")
                    print(f"[GO] {waiter.name}")
                print(f"[COMMAND] {waiter.name} -> final goal")

        if waiter.phase == "TO_APPROACH" and waiter.approach_goal is not None:
            at_approach = (
                distance_to_point(
                    waiter.last_valid_pose,
                    (waiter.approach_goal.x, waiter.approach_goal.y),
                ) <= goal_tolerance
            )
            if at_approach and not waiter_navigating:
                if now - last_final_dock_replan >= docking_replan_interval:
                    print(f"[APPROACH REACHED] {waiter.name}; replanning final leg")
                    actual_owner = owner.last_valid_pose or {}
                    actual_owner_goal = Goal(
                        float(actual_owner["x"]),
                        float(actual_owner["y"]),
                        float(actual_owner.get("yaw", 0.0)),
                    )
                    try:
                        current_final_path = plan_leg(
                            waiter, waiter.final_goal, plan_timeout
                        )
                        current_final_check = check_approved_docking_path(
                            actual_owner_goal,
                            current_final_path,
                            waiter.final_goal.yaw,
                            footprint_half_length,
                            footprint_half_width,
                            docking_margin,
                        )
                        final_safe = bool(current_final_check.get("safe"))
                        print_docking_check(waiter.name, current_final_check)
                    except TrafficControllerError as exc:
                        final_safe = False
                        current_final_path = []
                        print(f"[FINAL DOCK REPLAN CHECK] BLOCKED: {exc}")
                    last_final_dock_replan = now
                    if final_safe:
                        if docking_path_callback:
                            docking_path_callback(current_final_path)
                        send_goal_once(
                            waiter, waiter.final_goal, final=True, timeout_sec=http_timeout
                        )
                        waiter.phase = "DOCKING"
                        if docking_status_callback:
                            docking_status_callback("DIRECT SAFE")
                        print(f"[COMMAND] {waiter.name} -> S2 final docking goal")
                    else:
                        if docking_status_callback:
                            docking_status_callback("BLOCKED")
                        print(f"[DOCKING BLOCKED] {waiter.name} remains at APPROACH")
            elif (
                waiter.approach_command_time is not None
                and now - waiter.approach_command_time > grace_sec
                and not waiter_navigating
            ):
                waiter.phase = "FAILED"
                raise TrafficControllerError(
                    f"{waiter.name} stopped before DOCK APPROACH; no final docking release"
                )

        if owner_at_goal and not owner_navigating and not terminal_docking:
            owner.phase = "COMPLETE"
        if waiter.final_command_sent:
            waiter_action_state = navigation_action_state(waiter_state)
            waiter_at_goal = (
                distance_to_point(
                    waiter.last_valid_pose, (waiter.final_goal.x, waiter.final_goal.y)
                )
                <= goal_tolerance
            )
            if (
                waiter_at_goal
                and waiter_action_state == "SUCCEEDED"
                and not waiter_navigating
            ):
                waiter.phase = "COMPLETE"
            elif waiter_action_state in ("ABORTED", "CANCELED"):
                waiter.phase = "FAILED"
                raise TrafficControllerError(
                    f"{waiter.name} final navigation {waiter_action_state}"
                )
            elif (
                waiter.command_time is not None
                and now - waiter.command_time > grace_sec
                and not waiter_navigating
                and not waiter_at_goal
            ):
                waiter.phase = "FAILED"
                raise TrafficControllerError(
                    f"{waiter.name} stopped before successful final goal "
                    f"(action={waiter_action_state})"
                )
        owner_complete = owner.phase in ("COMPLETE", "DOCKED")
        if owner_complete and waiter.phase == "COMPLETE":
            if terminal_docking:
                print("[HOME] HOME_COMPLETE")
            return
        time.sleep(1.0 / poll_hz)


def clear_paths_loop(
    robots: list[RobotContext],
    poll_hz: float,
    pose_timeout: float,
    goal_tolerance: float,
    http_timeout: float,
    mission_timeout: float,
    robot_max_speed: float,
    localization_speed_factor: float,
    localization_violation_count: int,
    localization_hard_jump: float,
) -> None:
    """Monitor both independently-started robots without traffic release logic."""
    started = time.monotonic()
    grace_sec = 2.0
    while True:
        now = time.monotonic()
        if now - started > mission_timeout:
            raise TrafficControllerError("mission timeout")
        try:
            for robot in robots:
                if robot.phase == "COMPLETE":
                    continue
                state = get_state(robot, http_timeout)
                validate_localization_state(
                    robot, state, now, robot_max_speed,
                    localization_speed_factor, localization_violation_count,
                    localization_hard_jump,
                )
                at_goal = (
                    distance_to_point(
                        robot.last_valid_pose, (robot.final_goal.x, robot.final_goal.y)
                    )
                    <= goal_tolerance
                )
                navigating = navigating_state(state, robot.name)
                action_state = navigation_action_state(state)
                if at_goal and action_state == "SUCCEEDED" and not navigating:
                    robot.phase = "COMPLETE"
                    print(f"[STATE] {robot.name}: COMPLETE")
                elif action_state in ("ABORTED", "CANCELED"):
                    robot.phase = "FAILED"
                    raise TrafficControllerError(
                        f"{robot.name} navigation {action_state}"
                    )
                elif (
                    robot.command_time is not None
                    and now - robot.command_time > grace_sec
                    and not navigating
                ):
                    robot.phase = "FAILED"
                    raise TrafficControllerError(
                        f"{robot.name} navigation stopped before successful final goal "
                        f"(action={action_state})"
                    )
        except LocalizationLostError as exc:
            stop_all_after_localization_loss(robots, http_timeout)
            raise LocalizationLostError(str(exc)) from exc
        if all(robot.phase == "COMPLETE" for robot in robots):
            return
        time.sleep(1.0 / poll_hz)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Two-robot HOLD-point traffic controller")
    parser.add_argument("--cart1")
    parser.add_argument("--cart2", required=True)
    parser.add_argument(
        "--mock-cart1",
        action="store_true",
        help="use an HTTP-free CART-1 test double at the approved S1 goal",
    )
    parser.add_argument(
        "--mock-cart1-docked",
        action="store_true",
        help="inject mock CART-1 pose=S1 and navigating=false (implies --mock-cart1)",
    )
    for cart in ("cart1", "cart2"):
        parser.add_argument(f"--{cart}-goal-x", required=True, type=float)
        parser.add_argument(f"--{cart}-goal-y", required=True, type=float)
        parser.add_argument(f"--{cart}-goal-yaw", required=True, type=float, help="radians")
    parser.add_argument("--priority", choices=("cart1", "cart2"), default="cart1")
    parser.add_argument("--clearance", type=float, default=0.20)
    parser.add_argument("--hold-margin", type=float, default=0.35)
    parser.add_argument("--release-margin", type=float, default=0.20)
    parser.add_argument("--hold-tolerance", type=float, default=0.08)
    parser.add_argument("--goal-tolerance", type=float, default=0.10)
    parser.add_argument("--poll-hz", type=float, default=5.0)
    parser.add_argument(
        "--robot-max-speed",
        required=True,
        type=float,
        help=(
            "physical maximum linear speed in m/s; AMCL displacement is checked "
            "against this value and the actual polling interval"
        ),
    )
    parser.add_argument(
        "--localization-speed-factor",
        type=float,
        default=3.0,
        help="soft AMCL implied-speed limit multiplier (default: 3.0)",
    )
    parser.add_argument(
        "--localization-violation-count",
        type=int,
        default=3,
        help="consecutive soft-limit violations before safety fault (default: 3)",
    )
    parser.add_argument(
        "--localization-hard-jump",
        type=float,
        default=0.5,
        help="single-sample AMCL displacement causing immediate fault in meters",
    )
    parser.add_argument("--pose-timeout", type=float, default=3.0)
    parser.add_argument("--pose-stability-duration", type=float, default=2.5)
    parser.add_argument("--pose-stability-timeout", type=float, default=12.0)
    parser.add_argument("--mission-timeout", type=float, default=300.0)
    parser.add_argument("--plan-timeout", type=float, default=10.0)
    parser.add_argument(
        "--replan-retries",
        type=int,
        default=3,
        help="maximum release-time /nav/plan attempts while waiter remains WAIT",
    )
    parser.add_argument(
        "--replan-interval",
        type=float,
        default=1.0,
        help="seconds between release-time /nav/plan attempts",
    )
    parser.add_argument(
        "--s2-undock-recovery",
        action="store_true",
        help="allow CART-2 to leave the known trapped S2 start via short odom micro-steps",
    )
    parser.add_argument("--undock-step", type=float, default=0.03)
    parser.add_argument("--undock-max-distance", type=float, default=0.15)
    parser.add_argument("--undock-speed", type=float, default=0.02)
    parser.add_argument("--undock-retries", type=int, default=5)
    parser.add_argument("--undock-s2-position-tolerance", type=float, default=0.08)
    parser.add_argument("--undock-s2-yaw-tolerance", type=float, default=0.15)
    parser.add_argument("--http-timeout", type=float, default=3.0)
    parser.add_argument(
        "--docking", "--return-home",
        dest="docking",
        action="store_true",
        help="sequential return-home mode for the approved CART-1/S1 + CART-2/S2 pair",
    )
    parser.add_argument(
        "--docking-clearance",
        type=float,
        default=0.15,
        help="legacy center-clearance display value; normal traffic still uses --clearance",
    )
    parser.add_argument(
        "--docking-margin",
        type=float,
        default=NAV2_FOOTPRINT_PADDING_M,
        help="per-robot docking footprint padding (Nav2 default here: 0.03m)",
    )
    parser.add_argument(
        "--docking-footprint-half-length",
        type=float,
        default=NAV2_FOOTPRINT_HALF_LENGTH_M,
        help="robot footprint half-length used only for docking checks (default: 0.06m)",
    )
    parser.add_argument(
        "--docking-footprint-half-width",
        type=float,
        default=NAV2_FOOTPRINT_HALF_WIDTH_M,
        help="robot footprint half-width used only for docking checks (default: 0.06m)",
    )
    parser.add_argument(
        "--docking-yaw-tolerance",
        type=float,
        default=0.15,
        help="maximum dock orientation error in radians (default: 0.15)",
    )
    parser.add_argument(
        "--cart2-dock-approach-x", type=float,
        help="optional CART-2 alternate docking waypoint x",
    )
    parser.add_argument(
        "--cart2-dock-approach-y", type=float,
        help="optional CART-2 alternate docking waypoint y",
    )
    parser.add_argument(
        "--cart2-dock-approach-yaw", type=float, default=0.0,
        help="CART-2 pre-docking waypoint yaw (default: 0.0)",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", help="validate only (default)")
    mode.add_argument("--execute", action="store_true", help="send real /nav/goal commands")
    parser.add_argument("--serve", action="store_true", help="publish until Ctrl+C")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.robot_max_speed <= 0.0:
        print("[SAFE STOP] --robot-max-speed must be greater than zero", file=sys.stderr)
        return 2
    if (
        args.localization_speed_factor <= 0.0
        or args.localization_violation_count < 1
        or args.localization_hard_jump <= 0.0
    ):
        print(
            "[SAFE STOP] localization speed factor/hard jump must be > 0 and "
            "violation count must be >= 1",
            file=sys.stderr,
        )
        return 2
    if args.replan_retries < 1 or args.replan_interval < 0.0:
        print(
            "[SAFE STOP] --replan-retries must be >= 1 and "
            "--replan-interval must be >= 0",
            file=sys.stderr,
        )
        return 2
    if (
        args.undock_step <= 0.0
        or args.undock_max_distance <= 0.0
        or args.undock_speed <= 0.0
        or args.undock_speed > 0.08
        or args.undock_retries < 1
        or args.undock_s2_position_tolerance <= 0.0
        or args.undock_s2_yaw_tolerance <= 0.0
    ):
        print(
            "[SAFE STOP] invalid S2 undock parameters; require positive distances/"
            "tolerances, retries>=1, and 0<speed<=0.08m/s",
            file=sys.stderr,
        )
        return 2
    dry_run = not args.execute
    mock_cart1 = bool(args.mock_cart1 or args.mock_cart1_docked)
    if not mock_cart1 and not args.cart1:
        print("[SAFE STOP] --cart1 is required unless --mock-cart1 is used", file=sys.stderr)
        return 2
    if args.execute and mock_cart1 and not args.mock_cart1_docked:
        print(
            "[SAFE STOP] mock CART-1 execute requires --mock-cart1-docked",
            file=sys.stderr,
        )
        print("[SAFE STOP] No /nav/goal or /nav/stop was issued.", file=sys.stderr)
        return 2
    carts = {
        "cart1": RobotContext(
            "CART-1", (args.cart1 or "").rstrip("/"),
            Goal(args.cart1_goal_x, args.cart1_goal_y, args.cart1_goal_yaw),
            mock=mock_cart1,
            mock_docked=bool(args.mock_cart1_docked),
        ),
        "cart2": RobotContext(
            "CART-2", args.cart2.rstrip("/"),
            Goal(args.cart2_goal_x, args.cart2_goal_y, args.cart2_goal_yaw),
        ),
    }
    approach_values = (args.cart2_dock_approach_x, args.cart2_dock_approach_y)
    if (approach_values[0] is None) != (approach_values[1] is None):
        print(
            "[SAFE STOP] both --cart2-dock-approach-x and -y are required",
            file=sys.stderr,
        )
        return 2
    if approach_values[0] is not None:
        if not args.docking:
            print("[SAFE STOP] dock approach requires --docking", file=sys.stderr)
            return 2
        carts["cart2"].approach_goal = Goal(
            float(approach_values[0]),
            float(approach_values[1]),
            float(args.cart2_dock_approach_yaw),
        )
    if args.docking:
        try:
            if args.priority != "cart1":
                raise TrafficControllerError(
                    "approved docking pair requires --priority cart1 (CART-1 docks first)"
                )
            validate_docking_goals(carts)
            if args.docking_margin < 0.0:
                raise TrafficControllerError(
                    "--docking-margin must not be negative"
                )
            if (
                args.docking_footprint_half_length <= 0.0
                or args.docking_footprint_half_width <= 0.0
            ):
                raise TrafficControllerError(
                    "docking footprint half-length and half-width must be positive"
                )
            if args.docking_yaw_tolerance <= 0.0:
                raise TrafficControllerError(
                    "--docking-yaw-tolerance must be > 0"
                )
        except TrafficControllerError as exc:
            print(f"[SAFE STOP] {exc}", file=sys.stderr)
            print("[SAFE STOP] No /nav/goal or /nav/stop was issued.", file=sys.stderr)
            return 2
    authority: dict[str, Any] = {
        "mode": "dry-run" if dry_run else "execute",
        "priority": args.priority,
        "owner": None,
        "waiter": None,
        "conflict": False,
        "segments": [],
        "holdPoint": None,
        "terminalDocking": False,
        "dockingPathStatus": None,
        "dockingPathPrecheck": None,
        "dockApproach": None,
    }

    def authority_snapshot() -> dict[str, Any]:
        waiter_key = "cart2" if args.priority == "cart1" else "cart1"
        conflict_active = bool(authority["conflict"])
        released = conflict_active and carts[waiter_key].final_command_sent
        states = {"cart1": "GO", "cart2": "GO"}
        if conflict_active:
            if authority["terminalDocking"]:
                states[args.priority] = (
                    "DOCKED" if carts[args.priority].phase == "DOCKED" else "DOCKING"
                )
            else:
                states[args.priority] = "CLEAR" if released else "GO"
            if carts[waiter_key].phase == "TO_APPROACH":
                states[waiter_key] = "DOCK APPROACH"
            elif released:
                states[waiter_key] = (
                    "DOCKING" if authority["terminalDocking"] else "GO"
                )
            elif authority["terminalDocking"] and authority["dockingPathStatus"] == "BLOCKED":
                states[waiter_key] = "DOCKING BLOCKED"
            elif (
                authority["terminalDocking"]
                and isinstance(authority["dockingPathStatus"], str)
                and authority["dockingPathStatus"].endswith("SAFE")
            ):
                states[waiter_key] = authority["dockingPathStatus"]
            elif carts[waiter_key].phase == "START_HOLD":
                states[waiter_key] = "START HOLD"
            else:
                states[waiter_key] = "WAIT"
        return {
            "mode": authority["mode"],
            "priority": authority["priority"],
            "owner": authority["owner"],
            "waiter": authority["waiter"],
            "reservationReleased": released,
            "conflict": conflict_active,
            "terminalDocking": authority["terminalDocking"],
            "dockingPathStatus": authority["dockingPathStatus"],
            "dockingPathPrecheck": authority["dockingPathPrecheck"],
            "cart1": {"phase": carts["cart1"].phase, "trafficState": states["cart1"]},
            "cart2": {"phase": carts["cart2"].phase, "trafficState": states["cart2"]},
            "holdPoint": authority["holdPoint"],
            "segments": authority["segments"],
        }

    ros_runtime = RosPublisherRuntime(authority_snapshot)
    if mock_cart1:
        print("[TEST MODE] CART-1 is MOCK")
    if args.mock_cart1_docked:
        print("[MOCK DOCKED] CART-1 at S1")
    print(f"[MODE] {'DRY-RUN: no /nav/goal or /nav/stop will be called' if dry_run else 'EXECUTE: real robots may move'}")
    print("[MVP] Conflict/HOLD/EXIT use immutable paths from the first final-goal /nav/plan.")
    for robot in carts.values():
        print_goal(f"[FINAL] {robot.name}", robot.final_goal)

    try:
        wait_for_stable_poses(
            list(carts.values()),
            args.pose_stability_duration,
            args.pose_stability_timeout,
            max(2.0, args.poll_hz),
            args.http_timeout,
            args.robot_max_speed,
            args.localization_speed_factor,
            args.localization_violation_count,
            args.localization_hard_jump,
        )
        for key, robot in carts.items():
            other = carts["cart2" if key == "cart1" else "cart1"]
            plan_robot_with_s2_undock_recovery(robot, other, args, dry_run)
            print(f"[PLAN] {robot.name}: {len(robot.original_planned_path)} points")
        if carts["cart2"].approach_goal is not None:
            approach = carts["cart2"].approach_goal
            print_goal("[ALTERNATE CANDIDATE] waypoint", approach)
            authority["dockApproach"] = {
                "x": approach.x, "y": approach.y, "yaw": approach.yaw
            }
            print("[ALTERNATE CANDIDATE] deferred until a post-DOCKED direct replan is BLOCKED")
        ros_runtime.node.publish_reference("cart1", carts["cart1"].original_planned_path)
        ros_runtime.node.publish_reference("cart2", carts["cart2"].original_planned_path)

        conflict = find_path_conflicts(
            carts["cart1"].original_planned_path,
            carts["cart2"].original_planned_path,
            args.clearance,
            max_samples=200,
        )
        segments = conflict.get("segments") or []
        authority["conflict"] = bool(segments)
        authority["segments"] = segments
        print(
            f"[PATH CONFLICT] CART-1 <-> CART-2 segments={len(segments)} "
            f"clearance={args.clearance:.3f}m"
            if segments
            else f"[PATHS CLEAR] clearance={args.clearance:.3f}m"
        )
        for number, segment in enumerate(segments, 1):
            entry, exit_point = segment["entry"], segment["exit"]
            print(
                f"  segment {number}: ENTRY=({entry['x']:.3f},{entry['y']:.3f}) "
                f"EXIT=({exit_point['x']:.3f},{exit_point['y']:.3f})"
            )

        owner: RobotContext
        waiter: RobotContext | None
        release_index: int | None = None
        terminal_docking = False
        docking_path_check: dict[str, Any] | None = None
        if args.docking:
            # Approved return-home mode is sequential even when the two initial
            # global paths happen not to conflict.  CART-2 stays at its current
            # pose; only CART-1 may move until it is verified PARKED at S1.
            owner = carts["cart1"]
            waiter = carts["cart2"]
            authority["owner"] = owner.name
            authority["waiter"] = waiter.name
            terminal_docking = True
            authority["terminalDocking"] = True
            owner.phase = "DOCKING"
            prepare_start_hold(waiter, args.http_timeout)
            authority["holdPoint"] = {
                "robot": waiter.name,
                "x": waiter.hold_goal.x,
                "y": waiter.hold_goal.y,
                "yaw": waiter.hold_goal.yaw,
                "type": "START_HOLD",
            }
            docking_path_check = check_approved_docking_path(
                owner.final_goal,
                waiter.original_planned_path,
                waiter.final_goal.yaw,
                args.docking_footprint_half_length,
                args.docking_footprint_half_width,
                args.docking_margin,
            )
            authority["dockingPathPrecheck"] = (
                "SAFE" if docking_path_check.get("safe") else "BLOCKED"
            )
            print(
                "[RETURN HOME] approved CART-1->S1 + CART-2->S2 pair; "
                "sequential motion is mandatory"
            )
            print(
                f"[DOCKING FOOTPRINT] half_length="
                f"{args.docking_footprint_half_length:.3f}m "
                f"half_width={args.docking_footprint_half_width:.3f}m "
                f"margin={args.docking_margin:.3f}m"
            )
            print(f"[TRAFFIC] owner={owner.name} waiter={waiter.name}")
            print(f"[WAIT] {waiter.name}")
            print_goal(f"[START HOLD POSE] {waiter.name}", waiter.hold_goal)
            print(f"[HOME] {waiter.name} HOLD")
            print(f"[HOME] {owner.name} -> S1")
            print("[REPLAN POLICY] owner PARKED -> DIRECT REPLAN; alternate only on collision")
        elif not segments:
            owner = carts["cart1"]
            waiter = carts["cart2"]
            print("[TRAFFIC] paths clear; both robots use final goals")
        else:
            owner = carts[args.priority]
            waiter_key = "cart2" if args.priority == "cart1" else "cart1"
            waiter = carts[waiter_key]
            authority["owner"] = owner.name
            authority["waiter"] = waiter.name
            hold_segment = min(
                segments,
                key=lambda item: int(item[f"{waiter_key}_start_index"]),
            )
            release_segment = max(
                segments,
                key=lambda item: int(item[f"{args.priority}_end_index"]),
            )
            waiter_index_key = f"{waiter_key}_start_index"
            entry_index = int(hold_segment[waiter_index_key])
            hold_index = retreat_path_index(
                waiter.original_planned_path,
                entry_index,
                args.hold_margin,
            )
            actual_hold_margin = path_distance_between(
                waiter.original_planned_path, hold_index, entry_index
            )
            if owner.mock_docked:
                # The mock owner is already stationary at S1.  Keep the real
                # waiter at its current pose until the unchanged docking safety
                # checks release it directly toward S2.
                hold_index = 0
                prepare_start_hold(waiter, args.http_timeout)
                print(f"[START HOLD] {waiter.name} will remain at its current pose")
            elif actual_hold_margin + 1e-6 < args.hold_margin:
                if hold_index != 0:
                    raise TrafficControllerError(
                        f"{waiter.name} path has only {actual_hold_margin:.3f}m before "
                        f"ENTRY; requested HOLD margin is {args.hold_margin:.3f}m"
                    )
                prepare_start_hold(waiter, args.http_timeout)
                print(f"[START HOLD] {waiter.name} will remain at its current pose")
            else:
                hold_x, hold_y = waiter.original_planned_path[hold_index]
                waiter.hold_goal = Goal(
                    hold_x, hold_y, path_heading(waiter.original_planned_path, hold_index)
                )
            authority["holdPoint"] = {
                "robot": waiter.name,
                "x": waiter.hold_goal.x,
                "y": waiter.hold_goal.y,
                "yaw": waiter.hold_goal.yaw,
                "type": "START_HOLD" if waiter.start_hold else "HOLD",
            }
            exit_index = int(release_segment[f"{args.priority}_end_index"])
            release_index = advance_path_index(
                owner.original_planned_path,
                exit_index,
                args.release_margin,
            )
            actual_release_margin = path_distance_between(
                owner.original_planned_path, exit_index, release_index
            )
            if actual_release_margin + 1e-6 < args.release_margin:
                conflict_reaches_final = exit_index == len(owner.original_planned_path) - 1
                if args.docking and conflict_reaches_final:
                    terminal_docking = True
                    authority["terminalDocking"] = True
                    owner.phase = "DOCKING"
                    docking_path_check = check_approved_docking_path(
                        owner.final_goal,
                        waiter.original_planned_path[hold_index:],
                        waiter.final_goal.yaw,
                        args.docking_footprint_half_length,
                        args.docking_footprint_half_width,
                        args.docking_margin,
                    )
                    authority["dockingPathPrecheck"] = (
                        "SAFE" if docking_path_check.get("safe") else "BLOCKED"
                    )
                    print(
                        f"[TERMINAL CONFLICT] {owner.name} conflict reaches final dock; "
                        "release will require DOCKED state"
                    )
                    print(
                        "[DOCKING PAIR] APPROVED: CART-1->S1 + CART-2->S2; "
                        "simultaneous parking assumes validated non-overlapping footprints"
                    )
                    print(
                        f"[DOCKING FOOTPRINT] half_length="
                        f"{args.docking_footprint_half_length:.3f}m "
                        f"half_width={args.docking_footprint_half_width:.3f}m "
                        f"margin={args.docking_margin:.3f}m "
                        f"minimum={float(docking_path_check.get('minimumDistanceM', math.inf)):.3f}m "
                        f"endpoint={float(docking_path_check.get('endpointDistanceM', math.inf)):.3f}m"
                    )
                    print(
                        f"[INITIAL DIRECT PATH CHECK] "
                        f"{'SAFE' if docking_path_check.get('safe') else 'BLOCKED'}: "
                        f"{docking_path_check.get('reason', 'unknown reason')}"
                    )
                    print(
                        "[REPLAN POLICY] owner DOCKED -> DIRECT REPLAN; "
                        "alternate is considered only if direct is BLOCKED"
                    )
                else:
                    reason = (
                        "terminal conflict requires --docking"
                        if conflict_reaches_final
                        else "conflict does not reach the final path sample"
                    )
                    raise TrafficControllerError(
                        f"{owner.name} path has only {actual_release_margin:.3f}m after "
                        f"EXIT; requested release margin is {args.release_margin:.3f}m; "
                        f"{reason}"
                    )
            print(f"[TRAFFIC] owner={owner.name} waiter={waiter.name}")
            print(f"[WAIT] {waiter.name}")
            if waiter.start_hold:
                print_goal(f"[START HOLD POSE] {waiter.name}", waiter.hold_goal)
            else:
                print_goal(f"[HOLD] {waiter.name}", waiter.hold_goal)
            if terminal_docking:
                print(f"[HOME] {waiter.name} HOLD")
                print(f"[HOME] {owner.name} -> S1")
                print(
                    f"[MARGINS] hold={args.hold_margin:.3f}m "
                    "terminal_release=DOCKED"
                )
            else:
                print(
                    f"[MARGINS] hold={args.hold_margin:.3f}m "
                    f"release={args.release_margin:.3f}m release_index={release_index}"
                )
            if not waiter.start_hold:
                validate_hold_plan(waiter, args.plan_timeout)
                print(f"[VALID] {waiter.name} HOLD point is Nav2-plannable")

        dry_run_docking_decision: dict[str, Any] | None = None
        if dry_run and terminal_docking and waiter is not None:
            print(f"[HOME] {owner.name} PARKED")
            print("[DRY-RUN REPLAN] simulating the decision after owner DOCKED")
            dry_run_docking_decision = select_docking_route(
                owner.final_goal,
                waiter,
                args.plan_timeout,
                args.docking_footprint_half_length,
                args.docking_footprint_half_width,
                args.docking_margin,
            )
            if dry_run_docking_decision.get("safe"):
                route = str(dry_run_docking_decision["route"])
                authority["dockingPathStatus"] = f"{route} SAFE"
                selected_path = dry_run_docking_decision.get("path") or []
                if selected_path:
                    ros_runtime.node.publish_reference("cart2", selected_path)
            else:
                authority["dockingPathStatus"] = "BLOCKED"
            if dry_run_docking_decision.get("safe"):
                if dry_run_docking_decision.get("route") == "ALTERNATE":
                    print(f"[HOME] {waiter.name} -> APPROACH -> S2")
                else:
                    print(f"[HOME] {waiter.name} -> S2")
                print("[HOME] HOME_COMPLETE")

        ros_runtime.node.publish_state(force=True)

        for robot in carts.values():
            if robot.mock:
                print(f"[WOULD NOT SEND] {robot.name} -> MOCK robot; no HTTP command")
                continue
            if (segments or terminal_docking) and robot is waiter and robot.start_hold:
                print(f"[WOULD NOT SEND] {robot.name} -> waiting at start")
                continue
            planned_goal = (
                robot.hold_goal
                if (segments or terminal_docking)
                and robot is waiter
                and robot.hold_goal is not None
                else (
                    robot.final_goal
                    if terminal_docking
                    else transit_goal(robot)
                )
            )
            suffix = (
                "HOME GOAL"
                if terminal_docking and planned_goal is robot.final_goal
                else "TRANSIT GOAL"
                if not terminal_docking and planned_goal is not robot.hold_goal
                else "HOLD POINT"
            )
            print_goal(f"[WOULD SEND] {robot.name} -> {suffix}", planned_goal)

        if (segments or terminal_docking) and waiter is not None:
            if terminal_docking:
                print(
                    f"[RELEASE] {owner.name} must reach its docking goal, "
                    "stop navigating, and be within goal tolerance"
                )
                if dry_run_docking_decision and dry_run_docking_decision.get("safe"):
                    route = dry_run_docking_decision.get("route")
                    if route == "ALTERNATE":
                        print(f"[AFTER DOCKED] [RELEASE] {waiter.name} -> APPROACH, then S2")
                    else:
                        print(f"[AFTER DOCKED] [DIRECT GO] {waiter.name} -> S2")
                else:
                    print(f"[AFTER DOCKED] [DOCKING BLOCKED] {waiter.name} remains WAIT")
            else:
                print(
                    f"[RELEASE] {owner.name} must pass EXIT + "
                    f"{args.release_margin:.2f}m"
                )
            if waiter.start_hold and not terminal_docking:
                print(f"[AFTER RELEASE] {waiter.name} -> FINAL GOAL")
                print(
                    "[DRY-RUN] release-time replan is not executed because "
                    "the owner has not physically moved"
                )
                print(
                    f"[DRY-RUN] after EXIT + {args.release_margin:.2f}m: "
                    f"{waiter.name} would replan current pose -> final goal"
                )
                print(f"[DRY-RUN] {waiter.name} would be released after plan success")

        if dry_run:
            print("[DONE] DRY-RUN complete; no robot navigation command was sent")
            if args.serve:
                print("[SERVE] Publishing authoritative state at 1 Hz; Ctrl+C to stop")
                while rclpy.ok():
                    time.sleep(0.5)
            return 0

        # Restart safety: never overwrite an already-running mission.
        now = time.monotonic()
        for robot in carts.values():
            state = get_state(robot, args.http_timeout)
            validate_localization_state(
                robot, state, now, args.robot_max_speed,
                args.localization_speed_factor,
                args.localization_violation_count,
                args.localization_hard_jump,
            )
            if navigating_state(state, robot.name):
                raise TrafficControllerError(
                    f"{robot.name} is already navigating; refusing automatic resume/re-send"
                )
            if robot.last_valid_pose is None:
                raise TrafficControllerError(f"{robot.name} pose is null; no goal sent")
            if (
                robot.start_hold
                and robot.hold_goal is not None
                and distance_to_point(
                    robot.last_valid_pose, (robot.hold_goal.x, robot.hold_goal.y)
                ) > args.hold_tolerance
            ):
                raise TrafficControllerError(
                    f"{robot.name} moved before START_HOLD began; no goal sent"
                )

        if not segments and not terminal_docking:
            for robot in carts.values():
                if robot.mock:
                    robot.phase = "COMPLETE"
                    print(f"[MOCK] {robot.name}: no HTTP goal command sent")
                    continue
                send_goal_once(
                    robot, transit_goal(robot), final=True,
                    timeout_sec=args.http_timeout,
                )
                print(f"[COMMAND] {robot.name} -> final goal")
            clear_paths_loop(
                list(carts.values()), args.poll_hz, args.pose_timeout,
                args.goal_tolerance, args.http_timeout, args.mission_timeout,
                args.robot_max_speed,
                args.localization_speed_factor,
                args.localization_violation_count,
                args.localization_hard_jump,
            )
        else:
            assert waiter is not None and waiter.hold_goal is not None
            if terminal_docking:
                # Return-home is deliberately sequential.  CART-1 is not allowed
                # to move until CART-2 is stationary at its HOLD state.
                print(f"[HOME] {waiter.name} HOLD")
                if waiter.start_hold:
                    print(f"[START HOLD] {waiter.name} remains stationary; no goal sent")
                else:
                    send_goal_once(
                        waiter, waiter.hold_goal, final=False,
                        timeout_sec=args.http_timeout,
                    )
                    print(f"[COMMAND] {waiter.name} -> HOLD point")
                    wait_for_home_hold(
                        waiter, args.poll_hz, args.pose_timeout,
                        args.hold_tolerance, args.http_timeout,
                        args.mission_timeout,
                    )
                if owner.mock:
                    if not owner.mock_docked:
                        raise TrafficControllerError(
                            f"{owner.name} mock is not docked; waiter remains WAIT"
                        )
                    owner.phase = "DOCKED"
                    print(f"[MOCK DOCKED] {owner.name} at S1; no HTTP command sent")
                else:
                    print(f"[HOME] {owner.name} -> S1")
                    send_goal_once(
                        owner, owner.final_goal, final=True,
                        timeout_sec=args.http_timeout,
                    )
                    owner.phase = "DOCKING"
                    print(f"[COMMAND] {owner.name} -> final goal")
            else:
                # Preserve the proven normal GO/WAIT behavior exactly: owner GO
                # and waiter START_HOLD/HOLD are dispatched as before.
                if owner.mock:
                    if not owner.mock_docked:
                        raise TrafficControllerError(
                            f"{owner.name} mock is not docked; waiter remains WAIT"
                        )
                    owner.phase = "DOCKED"
                    print(f"[MOCK DOCKED] {owner.name} at S1; no HTTP command sent")
                else:
                    send_goal_once(
                        owner, transit_goal(owner), final=True,
                        timeout_sec=args.http_timeout,
                    )
                    print(f"[GO] {owner.name}")
                    print(f"[COMMAND] {owner.name} -> final goal")
                if waiter.start_hold:
                    print(f"[START HOLD] {waiter.name} remains stationary; no goal sent")
                else:
                    send_goal_once(
                        waiter, waiter.hold_goal, final=False,
                        timeout_sec=args.http_timeout,
                    )
                    print(f"[COMMAND] {waiter.name} -> HOLD point")
            controller_loop(
                owner, waiter, release_index, args.poll_hz, args.pose_timeout,
                args.hold_tolerance, args.goal_tolerance, args.http_timeout,
                args.mission_timeout,
                args.release_margin,
                terminal_docking,
                args.plan_timeout,
                args.docking_footprint_half_length,
                args.docking_footprint_half_width,
                args.docking_margin,
                args.docking_yaw_tolerance,
                lambda status: authority.__setitem__("dockingPathStatus", status),
                lambda path: ros_runtime.node.publish_reference("cart2", path),
                args.robot_max_speed,
                args.replan_retries,
                args.replan_interval,
                args.localization_speed_factor,
                args.localization_violation_count,
                args.localization_hard_jump,
            )
        print("[DONE] mission complete")
        return 0
    except (TrafficControllerError, ValueError) as exc:
        for robot in carts.values():
            # A stationary waiter keeps the reservation if its owner fails.  Its
            # START_HOLD/HOLDING state is safety-significant and must remain visible.
            if robot.phase not in ("COMPLETE", "DOCKED", "START_HOLD", "HOLDING"):
                robot.phase = "FAILED"
        if isinstance(exc, PathBlockedError):
            print(f"[PATH BLOCKED] {exc}", file=sys.stderr)
            print(
                "[WAIT] retry budget exhausted; no emergency stop was issued.",
                file=sys.stderr,
            )
        else:
            print(f"[SAFE STOP] {exc}", file=sys.stderr)
        if isinstance(exc, LocalizationLostError) and not dry_run:
            print(
                "[SAFE STOP] Localization watchdog blocked release/new goals; "
                "/nav/stop was requested for both robots.",
                file=sys.stderr,
            )
        elif not isinstance(exc, PathBlockedError):
            print(
                "[SAFE STOP] No automatic release/resume or /nav/stop was issued.",
                file=sys.stderr,
            )
        return 2
    finally:
        ros_runtime.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n[STOP] interrupted; no automatic resume or /nav/stop was issued", file=sys.stderr)
        raise SystemExit(130)
