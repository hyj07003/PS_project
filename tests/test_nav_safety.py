from __future__ import annotations

import math
import sys
import tempfile
import time
import unittest
from unittest.mock import patch
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from pinky.modules.backends.mock import MockBackend
from nav2_safety_config import load_controller_safety_config
from traffic_controller import (
    Goal,
    RobotContext,
    TrafficControllerError,
    _oriented_boxes_overlap,
    _cart2_s2_undock_eligible,
    _undock_start_pose,
    check_approved_docking_path,
    docking_completion_ready,
    pose_history_stable,
    release_time_replan,
    transit_goal,
    validate_localization_state,
)


class NavSafetyTests(unittest.TestCase):
    def test_progress_checker_config_parsing(self) -> None:
        content = """
controller_server:
  ros__parameters:
    controller_plugins: [FollowPath]
    FollowPath:
      plugin: nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
    progress_checker_plugins: [progress_checker]
    progress_checker:
      plugin: nav2_controller::PoseProgressChecker
      required_movement_radius: 0.05
      required_movement_angle: 0.10
      movement_time_allowance: 12.0
    goal_checker_plugins: [general_goal_checker]
    general_goal_checker:
      plugin: nav2_controller::SimpleGoalChecker
      xy_goal_tolerance: 0.05
      yaw_goal_tolerance: 0.25
"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "nav2.yaml"
            path.write_text(content, encoding="utf-8")
            config = load_controller_safety_config(path)
        self.assertEqual(config["progressCheckerPlugin"], "nav2_controller::PoseProgressChecker")
        self.assertEqual(config["requiredMovementRadius"], 0.05)
        self.assertEqual(config["requiredMovementAngle"], 0.10)
        self.assertEqual(config["movementTimeAllowance"], 12.0)
        self.assertEqual(config["xyGoalTolerance"], 0.05)

    def test_mock_action_transition(self) -> None:
        backend = MockBackend()
        backend.start()
        self.assertEqual(backend.get_navigation_action()["state"], "UNKNOWN")
        backend.navigate_to(0.1, 0.0, 0.0)
        self.assertEqual(backend.get_navigation_action()["state"], "EXECUTING")
        deadline = time.time() + 3.0
        while time.time() < deadline and backend.get_navigation_action()["state"] != "SUCCEEDED":
            time.sleep(0.05)
        self.assertEqual(backend.get_navigation_action()["state"], "SUCCEEDED")
        backend.stop()

    def test_mock_relative_move_dry_run_and_move(self) -> None:
        backend = MockBackend()
        backend.start()
        before = backend.get_nav_pose()
        dry = backend.relative_move(0.03, 0.02, dry_run=True)
        self.assertTrue(dry["success"])
        self.assertTrue(dry["dryRun"])
        self.assertEqual(before, backend.get_nav_pose())
        moved = backend.relative_move(0.03, 0.02)
        self.assertTrue(moved["success"])
        after = backend.get_nav_pose()
        self.assertAlmostEqual(
            math.hypot(after["x"] - before["x"], after["y"] - before["y"]),
            0.03,
            places=6,
        )

    def test_cart2_s2_undock_eligibility(self) -> None:
        robot = RobotContext("CART-2", "mock", Goal(0.8, -2.34, 0.0))
        robot.last_valid_pose = {
            "x": 0.038474577957370054,
            "y": -0.1911947634013857,
            "yaw": -0.02685422279113792,
        }
        eligible, reason = _cart2_s2_undock_eligible(
            robot, position_tolerance=0.08, yaw_tolerance=0.20
        )
        self.assertTrue(eligible, reason)
        predicted = _undock_start_pose(robot, 0.03)
        self.assertGreater(predicted.x, robot.last_valid_pose["x"])

        home_robot = RobotContext(
            "CART-2",
            "mock",
            Goal(
                0.038474577957370054,
                -0.1911947634013857,
                -0.02685422279113792,
            ),
        )
        home_robot.last_valid_pose = dict(robot.last_valid_pose)
        eligible, _ = _cart2_s2_undock_eligible(
            home_robot, position_tolerance=0.08, yaw_tolerance=0.20
        )
        self.assertFalse(eligible)

    def test_pose_stability(self) -> None:
        robot = RobotContext("CART-X", "mock", Goal(0.0, 0.0, 0.0))
        now = time.monotonic()
        robot.pose_history = [(now - 1.9, 0.0, 0.0, 0.0), (now, 0.01, 0.0, 0.02)]
        self.assertTrue(pose_history_stable(robot, now, duration=2.0))
        robot.pose_history[-1] = (now, 0.04, 0.0, 0.02)
        self.assertFalse(pose_history_stable(robot, now, duration=2.0))

    def test_docked_gate_and_waiter_release_blocking(self) -> None:
        goal = Goal(1.0, 2.0, 0.1)
        pose = {"x": 1.01, "y": 2.0, "yaw": 0.11}
        self.assertTrue(docking_completion_ready(pose, goal, 0.03, 0.10, True, "SUCCEEDED", False))
        for state in ("UNKNOWN", "ACCEPTED", "EXECUTING", "ABORTED", "CANCELED"):
            self.assertFalse(docking_completion_ready(pose, goal, 0.03, 0.10, True, state, False))
        self.assertFalse(docking_completion_ready(pose, goal, 0.03, 0.10, False, "SUCCEEDED", False))
        self.assertFalse(docking_completion_ready(pose, goal, 0.03, 0.10, True, "SUCCEEDED", True))
        self.assertFalse(docking_completion_ready(None, goal, 0.03, 0.10, True, "SUCCEEDED", False))

    def test_approved_home_footprints_fit_at_final_poses(self) -> None:
        s1 = Goal(0.036703343955750284, 0.0005066978948139312, 0.009148818566518708)
        s2 = Goal(0.038474577957370054, -0.1911947634013857, -0.026854222309465525)
        self.assertFalse(_oriented_boxes_overlap(s1, s2, 0.06, 0.06, 0.03))

    def test_docking_path_blocks_real_footprint_overlap(self) -> None:
        parked = Goal(0.0, 0.0, 0.0)
        path = [(0.0, -0.30), (0.0, -0.115), (0.0, -0.20)]
        result = check_approved_docking_path(parked, path, 0.0, 0.06, 0.06, 0.03)
        self.assertFalse(result["safe"])
        self.assertTrue(result["footprintCollision"])

    def test_docking_swept_path_detects_between_sparse_points(self) -> None:
        parked = Goal(0.0, 0.0, 0.0)
        path = [(-0.30, -0.17), (0.30, -0.17)]
        result = check_approved_docking_path(parked, path, 0.0, 0.06, 0.06, 0.03)
        self.assertFalse(result["safe"])
        self.assertTrue(result["footprintCollision"])

    def test_docking_swept_path_checks_final_rotation(self) -> None:
        parked = Goal(0.0, 0.0, 0.0)
        path = [(-0.30, 0.19), (0.0, 0.19)]
        result = check_approved_docking_path(
            parked, path, math.pi / 2.0, 0.06, 0.06, 0.03
        )
        self.assertFalse(result["safe"])
        self.assertTrue(result["footprintCollision"])

    def test_release_time_replan_uses_live_waiter_state(self) -> None:
        waiter = RobotContext("CART-1", "mock", Goal(1.0, 2.0, 0.1))
        waiter.original_planned_path = [(0.0, 0.0), (9.0, 9.0)]
        state = {
            "pose": {"x": 0.2, "y": 0.3, "yaw": 0.0},
            "navigating": False,
            "nav2Readiness": {
                "ready": True,
                "tfValid": True,
                "scanFresh": True,
                "failures": [],
            },
        }
        live_path = [(0.2, 0.3), (1.0, 2.0)]
        with patch("traffic_controller.plan_leg", return_value=live_path) as planner:
            result = release_time_replan(waiter, state, 10.0)
        planner.assert_called_once_with(waiter, waiter.final_goal, 10.0)
        self.assertEqual(result, live_path)
        self.assertEqual(waiter.release_planned_path, live_path)
        self.assertEqual(waiter.original_planned_path, [(0.0, 0.0), (9.0, 9.0)])

    def test_release_time_replan_blocks_invalid_live_tf(self) -> None:
        waiter = RobotContext("CART-1", "mock", Goal(1.0, 2.0, 0.1))
        state = {
            "pose": {"x": 0.2, "y": 0.3, "yaw": 0.0},
            "navigating": False,
            "nav2Readiness": {
                "ready": True,
                "tfValid": False,
                "scanFresh": True,
                "failures": ["map->base invalid"],
            },
        }
        with self.assertRaisesRegex(TrafficControllerError, "readiness invalid"):
            release_time_replan(waiter, state, 10.0)

    def test_release_time_replan_retries_temporary_plan_failure(self) -> None:
        waiter = RobotContext("CART-1", "mock", Goal(1.0, 2.0, 0.1))
        state = {
            "pose": {"x": 0.2, "y": 0.3, "yaw": 0.0},
            "navigating": False,
            "nav2Readiness": {
                "ready": True,
                "tfValid": True,
                "scanFresh": True,
                "failures": [],
            },
        }
        live_path = [(0.2, 0.3), (1.0, 2.0)]
        with (
            patch("traffic_controller.get_state", return_value=state),
            patch(
                "traffic_controller.plan_leg",
                side_effect=[TrafficControllerError("temporary"), live_path],
            ) as planner,
        ):
            result = release_time_replan(
                waiter, state, 10.0, replan_retries=3, replan_interval=0.0
            )
        self.assertEqual(result, live_path)
        self.assertEqual(planner.call_count, 2)

    def test_transit_goal_uses_path_approach_yaw(self) -> None:
        robot = RobotContext("CART-1", "mock", Goal(1.0, 1.0, math.pi))
        robot.original_planned_path = [(0.0, 0.0), (1.0, 1.0)]
        result = transit_goal(robot)
        self.assertAlmostEqual(result.x, 1.0)
        self.assertAlmostEqual(result.y, 1.0)
        self.assertAlmostEqual(result.yaw, math.pi / 4.0)

    @staticmethod
    def _localization_state(x: float) -> dict:
        return {
            "pose": {"x": x, "y": 0.0, "yaw": 0.0},
            "nav2Readiness": {
                "ready": True, "tfValid": True, "scanFresh": True,
            },
        }

    def test_localization_watchdog_allows_normal_motion(self) -> None:
        robot = RobotContext("CART-1", "mock", Goal(1.0, 1.0, 0.0))
        validate_localization_state(robot, self._localization_state(0.0), 10.0, 0.25)
        validate_localization_state(robot, self._localization_state(0.04), 10.2, 0.25)
        self.assertEqual(robot.localization_violation_count, 0)

    def test_localization_watchdog_allows_one_soft_violation(self) -> None:
        robot = RobotContext("CART-1", "mock", Goal(1.0, 1.0, 0.0))
        validate_localization_state(robot, self._localization_state(0.0), 10.0, 0.25)
        validate_localization_state(robot, self._localization_state(0.10), 10.1, 0.25)
        self.assertEqual(robot.localization_violation_count, 1)

    def test_localization_watchdog_resets_two_soft_violations_on_normal(self) -> None:
        robot = RobotContext("CART-1", "mock", Goal(1.0, 1.0, 0.0))
        validate_localization_state(robot, self._localization_state(0.0), 10.0, 0.25)
        validate_localization_state(robot, self._localization_state(0.10), 10.1, 0.25)
        validate_localization_state(robot, self._localization_state(0.20), 10.2, 0.25)
        validate_localization_state(robot, self._localization_state(0.22), 10.4, 0.25)
        self.assertEqual(robot.localization_violation_count, 0)

    def test_localization_watchdog_faults_after_three_soft_violations(self) -> None:
        robot = RobotContext("CART-1", "mock", Goal(1.0, 1.0, 0.0))
        validate_localization_state(robot, self._localization_state(0.0), 10.0, 0.25)
        validate_localization_state(robot, self._localization_state(0.10), 10.1, 0.25)
        validate_localization_state(robot, self._localization_state(0.20), 10.2, 0.25)
        with self.assertRaisesRegex(TrafficControllerError, "repeated 3 times"):
            validate_localization_state(
                robot, self._localization_state(0.30), 10.3, 0.25
            )

    def test_localization_watchdog_faults_on_one_hard_jump(self) -> None:
        robot = RobotContext("CART-1", "mock", Goal(1.0, 1.0, 0.0))
        validate_localization_state(robot, self._localization_state(0.0), 10.0, 0.25)
        with self.assertRaisesRegex(TrafficControllerError, "hard jump"):
            validate_localization_state(
                robot, self._localization_state(0.51), 10.2, 0.25
            )


if __name__ == "__main__":
    unittest.main()
