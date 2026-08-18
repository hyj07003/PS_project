#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import yaml


def load_controller_safety_config(path: str | Path) -> dict[str, Any]:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    params = ((data.get("controller_server") or {}).get("ros__parameters") or {})
    progress_ids = list(params.get("progress_checker_plugins") or [])
    goal_ids = list(params.get("goal_checker_plugins") or [])
    progress_id = progress_ids[0] if progress_ids else None
    goal_id = goal_ids[0] if goal_ids else None
    progress = params.get(progress_id) or {} if progress_id else {}
    goal = params.get(goal_id) or {} if goal_id else {}
    follow = params.get("FollowPath") or {}
    return {
        "controllerPlugin": follow.get("plugin"),
        "progressCheckerId": progress_id,
        "progressCheckerPlugin": progress.get("plugin"),
        "requiredMovementRadius": progress.get("required_movement_radius"),
        "requiredMovementAngle": progress.get("required_movement_angle"),
        "movementTimeAllowance": progress.get("movement_time_allowance"),
        "goalCheckerId": goal_id,
        "goalCheckerPlugin": goal.get("plugin"),
        "xyGoalTolerance": goal.get("xy_goal_tolerance"),
        "yawGoalTolerance": goal.get("yaw_goal_tolerance"),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Read Nav2 controller safety settings")
    parser.add_argument("yaml_path")
    args = parser.parse_args()
    for key, value in load_controller_safety_config(args.yaml_path).items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
