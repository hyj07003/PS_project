#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any

from traffic_common import find_path_conflicts, path_points


@dataclass
class RobotGoal:
    name: str
    base_url: str
    x: float
    y: float
    yaw_rad: float


def request_json(
    method: str,
    url: str,
    body: dict[str, Any] | None = None,
    timeout: float = 15.0,
) -> dict[str, Any]:
    data = None
    headers = {"Accept": "application/json"}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as res:
            raw = res.read().decode("utf-8")
            return json.loads(raw) if raw else {}
    except urllib.error.HTTPError as exc:
        raw = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"HTTP {exc.code} {url}: {raw[:300]}") from exc
    except urllib.error.URLError as exc:
        raise RuntimeError(f"cannot reach {url}: {exc.reason}") from exc


def compute_plan(robot: RobotGoal, timeout_sec: float) -> dict[str, Any]:
    print(f"[PLAN] {robot.name}: goal=({robot.x:.2f}, {robot.y:.2f}, {math.degrees(robot.yaw_rad):.1f}deg)")
    result = request_json(
        "POST",
        f"{robot.base_url}/nav/plan",
        {"x": robot.x, "y": robot.y, "yaw": robot.yaw_rad, "timeoutSec": timeout_sec},
        timeout=timeout_sec + 5.0,
    )
    if not result.get("success"):
        raise RuntimeError(f"{robot.name} planning failed: {result.get('message', result)}")
    count = int((result.get("path") or {}).get("count") or 0)
    print(f"[PLAN] {robot.name}: {count} path points")
    return result


def navigate_wait(robot: RobotGoal, timeout_sec: float) -> dict[str, Any]:
    print(f"[GO] {robot.name}")
    result = request_json(
        "POST",
        f"{robot.base_url}/nav/goal_wait",
        {"x": robot.x, "y": robot.y, "yaw": robot.yaw_rad, "timeoutSec": timeout_sec},
        timeout=timeout_sec + 15.0,
    )
    if result.get("success") or result.get("status") == "SUCCEEDED":
        print(f"[ARRIVED] {robot.name}")
    else:
        print(f"[FAILED] {robot.name}: {result}")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Two-Pinky first traffic scenario: pre-plan both paths, detect overlap, "
            "Robot1 GO / Robot2 WAIT, then RELEASE."
        )
    )
    parser.add_argument("--robot1", required=True, help="e.g. http://192.168.0.31:4200")
    parser.add_argument("--robot2", required=True, help="e.g. http://192.168.0.32:4200")
    parser.add_argument("--r1-x", required=True, type=float)
    parser.add_argument("--r1-y", required=True, type=float)
    parser.add_argument("--r1-yaw-deg", type=float, default=0.0)
    parser.add_argument("--r2-x", required=True, type=float)
    parser.add_argument("--r2-y", required=True, type=float)
    parser.add_argument("--r2-yaw-deg", type=float, default=0.0)
    parser.add_argument("--clearance", type=float, default=0.30, help="path safety corridor [m]")
    parser.add_argument("--plan-timeout", type=float, default=10.0)
    parser.add_argument("--nav-timeout", type=float, default=180.0)
    parser.add_argument(
        "--plan-only",
        action="store_true",
        help="only compute/compare paths; do not move either robot",
    )
    args = parser.parse_args()

    r1 = RobotGoal(
        "Robot1", args.robot1.rstrip("/"), args.r1_x, args.r1_y, math.radians(args.r1_yaw_deg)
    )
    r2 = RobotGoal(
        "Robot2", args.robot2.rstrip("/"), args.r2_x, args.r2_y, math.radians(args.r2_yaw_deg)
    )

    print("[CHECK] Pinky HTTP health")
    for robot in (r1, r2):
        health = request_json("GET", f"{robot.base_url}/health", timeout=3.0)
        print(
            f"  {robot.name}: ok={health.get('ok')} device={health.get('deviceCode')} "
            f"backend={health.get('backend')}"
        )

    plan1 = compute_plan(r1, args.plan_timeout)
    plan2 = compute_plan(r2, args.plan_timeout)
    p1 = path_points(plan1)
    p2 = path_points(plan2)
    conflict = find_path_conflicts(p1, p2, args.clearance)

    min_dist = conflict.get("minDistanceM")
    min_text = "n/a" if min_dist is None else f"{min_dist:.3f}m"
    print(
        f"[CONFLICT] overlap={conflict['conflict']} clearance={args.clearance:.2f}m "
        f"min_distance={min_text}"
    )
    if conflict["samples"]:
        first = conflict["samples"][0]
        print(f"[CONFLICT] example=({first['x']:.2f}, {first['y']:.2f})")

    if args.plan_only:
        print("[DONE] plan-only: robots were not moved")
        return 0

    if conflict["conflict"]:
        print("[RESERVE] Robot1 path reserved")
        print("[WAIT] Robot2: Robot1 reservation overlaps its path")
        r1_result = navigate_wait(r1, args.nav_timeout)
        if not (r1_result.get("success") or r1_result.get("status") == "SUCCEEDED"):
            print("[STOP] Robot1 failed; Robot2 remains blocked for safety")
            return 2
        print("[RELEASE] Robot1 reservation released")
        r2_result = navigate_wait(r2, args.nav_timeout)
        return 0 if (r2_result.get("success") or r2_result.get("status") == "SUCCEEDED") else 3

    print("[FREE] paths do not overlap: both robots may GO")
    results: dict[str, dict[str, Any]] = {}

    def run(robot: RobotGoal) -> None:
        results[robot.name] = navigate_wait(robot, args.nav_timeout)

    t1 = threading.Thread(target=run, args=(r1,))
    t2 = threading.Thread(target=run, args=(r2,))
    t1.start()
    t2.start()
    t1.join()
    t2.join()
    ok = all(r.get("success") or r.get("status") == "SUCCEEDED" for r in results.values())
    return 0 if ok else 4


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n[STOP] user interrupted")
        raise SystemExit(130)
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        raise SystemExit(1)
