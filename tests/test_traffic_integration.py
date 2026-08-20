"""Two-robot traffic integration scenarios (dry-run, no live Pinky required)."""

from __future__ import annotations

import os
import sys
import threading
import time
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "server" / "apps" / "controller-server"))
if "flask_cors" not in sys.modules:
    sys.modules["flask_cors"] = types.SimpleNamespace(CORS=lambda *a, **k: None)

from app.services.traffic import NavGoal, TrafficCoordinator  # noqa: E402
from app.waypoints import get_waypoint  # noqa: E402


class OverlapCartPort:
    """Two carts on parallel lanes with a shared conflict corridor."""

    def __init__(self) -> None:
        self._poses = {
            "cart-1": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "cart-2": {"x": 0.0, "y": 0.1, "yaw": 0.0},
        }
        self._navigating: dict[str, bool] = {}
        self._progress: dict[str, float] = {"cart-1": 0.0, "cart-2": 0.0}

    def plan_pose(self, device_code, x, y, yaw=0.0, timeout_sec=10.0):
        del timeout_sec, yaw
        start = self._poses[device_code]
        return {
            "success": True,
            "path": {
                "poses": [
                    {"x": start["x"], "y": start["y"]},
                    {"x": float(x), "y": float(y)},
                ],
            },
        }

    def get_pose(self, device_code):
        code = device_code
        start = self._poses[code]
        progress = self._progress.get(code, 0.0)
        return {
            "x": start["x"] + progress,
            "y": start["y"],
            "yaw": 0.0,
        }

    def get_nav_state_full(self, device_code):
        return {
            "pose": self.get_pose(device_code),
            "navigating": bool(self._navigating.get(device_code)),
        }

    def get_active_path(self, device_code):
        plan = self.plan_pose(device_code, 2.0, self._poses[device_code]["y"])
        return plan

    def is_reachable(self, device_code):
        del device_code
        return True

    def advance(self, device_code: str, delta: float) -> None:
        self._progress[device_code] = min(2.0, self._progress.get(device_code, 0.0) + delta)


class TrafficIntegrationTests(unittest.TestCase):
    def setUp(self) -> None:
        self._env_backup = dict(os.environ)

    def tearDown(self) -> None:
        os.environ.clear()
        os.environ.update(self._env_backup)

    def test_two_robot_mission_fifo_blocks_then_releases(self) -> None:
        os.environ["TRAFFIC_ENABLED"] = "1"
        port = OverlapCartPort()
        coord = TrafficCoordinator(port, robot_codes=["cart-1", "cart-2"])
        coord.register_mission("cart-1", 10)
        coord.register_mission("cart-2", 20)

        events: list[str] = []
        lock = threading.Lock()

        def cart1_leg() -> None:
            port._navigating["cart-1"] = True
            coord.acquire_nav_leg("cart-1", NavGoal(2.0, 0.0, 0.0), 10, "W1")
            with lock:
                events.append("cart-1-granted")
            for _ in range(5):
                port.advance("cart-1", 0.5)
                time.sleep(0.05)
            port._navigating["cart-1"] = False
            coord.release_nav_leg("cart-1")
            with lock:
                events.append("cart-1-released")

        def cart2_leg() -> None:
            time.sleep(0.05)
            port._navigating["cart-2"] = True
            coord.acquire_nav_leg("cart-2", NavGoal(2.0, 0.1, 0.0), 20, "W2")
            with lock:
                events.append("cart-2-granted")
            port._navigating["cart-2"] = False
            coord.release_nav_leg("cart-2")

        t1 = threading.Thread(target=cart1_leg)
        t2 = threading.Thread(target=cart2_leg)
        t1.start()
        t2.start()
        t1.join(timeout=10.0)
        t2.join(timeout=10.0)

        self.assertEqual(events[0], "cart-1-granted")
        self.assertIn("cart-1-released", events)
        self.assertEqual(events[-1], "cart-2-granted")

    def test_traffic_disabled_skips_coordination(self) -> None:
        os.environ["TRAFFIC_ENABLED"] = "0"
        port = OverlapCartPort()
        coord = TrafficCoordinator(port, robot_codes=["cart-1", "cart-2"])
        self.assertFalse(coord.enabled())
        coord.acquire_nav_leg("cart-1", NavGoal(1.0, 0.0, 0.0), 1, "W1")
        coord.acquire_nav_leg("cart-2", NavGoal(1.0, 0.1, 0.0), 2, "W2")

    def test_snapshot_reports_robot_phases(self) -> None:
        os.environ["TRAFFIC_ENABLED"] = "1"
        port = OverlapCartPort()
        coord = TrafficCoordinator(port, robot_codes=["cart-1", "cart-2"])
        coord.register_mission("cart-1", 1)
        port._navigating["cart-1"] = True
        coord.acquire_nav_leg("cart-1", NavGoal(2.0, 0.0, 0.0), 1, "W1")
        snap = coord.snapshot()
        self.assertTrue(snap["enabled"])
        self.assertIn("cart-1", snap["robots"])
        self.assertEqual(snap["robots"]["cart-1"]["phase"], "NAVIGATING")
        coord.release_nav_leg("cart-1")

    def test_remaining_overlap_defers_for_waiter(self) -> None:
        os.environ["TRAFFIC_ENABLED"] = "1"
        port = OverlapCartPort()
        coord = TrafficCoordinator(port, robot_codes=["cart-1", "cart-2"])
        coord.register_mission("cart-1", 10)
        coord.register_mission("cart-2", 20)
        coord.update_remaining("cart-1", ["W3", "W1", "C", "P"])
        coord.update_remaining("cart-2", ["W3", "W5", "C", "P"])
        defer = coord.conflicting_waypoints("cart-2", ["W3", "W5"])
        self.assertIn("W3", defer)
        self.assertNotIn("W5", defer)


if __name__ == "__main__":
    unittest.main()
