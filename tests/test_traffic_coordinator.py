from __future__ import annotations

import importlib.util
import os
import threading
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _load_module(name: str, rel_path: str):
    path = ROOT / rel_path
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


traffic_paths = _load_module(
    "traffic_paths",
    "server/apps/controller-server/app/services/traffic_paths.py",
)

# traffic.py depends on package imports; stub optional deps then load app.services.traffic.
import sys
import types

sys.path.insert(0, str(ROOT / "server" / "apps" / "controller-server"))
if "flask_cors" not in sys.modules:
    sys.modules["flask_cors"] = types.SimpleNamespace(CORS=lambda *a, **k: None)
from app.services.traffic import NavGoal, TrafficCoordinator, TrafficTimeoutError  # noqa: E402


class FakeCartPort:
    def __init__(self) -> None:
        self._poses = {
            "cart-1": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "cart-2": {"x": 0.0, "y": 0.1, "yaw": 0.0},
        }
        self._paths = {
            "cart-1": [(0.0, 0.0), (2.0, 0.0)],
            "cart-2": [(0.0, 0.1), (2.0, 0.1)],
        }
        self._navigating: dict[str, bool] = {}

    def plan_pose(self, device_code, x, y, yaw=0.0, timeout_sec=10.0):
        del timeout_sec
        pose = self._poses[device_code]
        return {
            "success": True,
            "path": {
                "poses": [
                    {"x": pose["x"], "y": pose["y"]},
                    {"x": float(x), "y": float(y)},
                ],
            },
        }

    def get_pose(self, device_code):
        return dict(self._poses[device_code])

    def get_nav_state_full(self, device_code):
        return {
            "pose": dict(self._poses[device_code]),
            "navigating": bool(self._navigating.get(device_code)),
        }

    def get_active_path(self, device_code):
        points = self._paths[device_code]
        return {
            "success": True,
            "path": {"poses": [{"x": x, "y": y} for x, y in points]},
        }

    def is_reachable(self, device_code):
        del device_code
        return True


class TrafficCoordinatorTests(unittest.TestCase):
    def test_fifo_owner_granted_first(self) -> None:
        port = FakeCartPort()
        coord = TrafficCoordinator(port, robot_codes=["cart-1", "cart-2"])
        coord.register_mission("cart-1", 1)
        coord.register_mission("cart-2", 2)
        granted: list[str] = []

        def owner() -> None:
            port._navigating["cart-1"] = True
            coord.acquire_nav_leg("cart-1", NavGoal(2.0, 0.0, 0.0), 1, "W1")
            granted.append("cart-1")
            time.sleep(0.2)
            port._navigating["cart-1"] = False
            coord.release_nav_leg("cart-1")

        def waiter() -> None:
            time.sleep(0.05)
            port._navigating["cart-2"] = True
            coord.acquire_nav_leg("cart-2", NavGoal(2.0, 0.1, 0.0), 2, "W2")
            granted.append("cart-2")
            port._navigating["cart-2"] = False
            coord.release_nav_leg("cart-2")

        t1 = threading.Thread(target=owner)
        t2 = threading.Thread(target=waiter)
        t1.start()
        t2.start()
        t1.join(timeout=5.0)
        t2.join(timeout=5.0)
        self.assertEqual(granted[0], "cart-1")
        self.assertIn("cart-2", granted)

    def test_return_home_fifo(self) -> None:
        port = FakeCartPort()
        coord = TrafficCoordinator(port, robot_codes=["cart-1", "cart-2"])
        order: list[str] = []

        def first() -> None:
            coord.acquire_return_home("cart-1")
            order.append("cart-1-start")
            time.sleep(0.1)
            coord.release_return_home("cart-1")
            order.append("cart-1-end")

        def second() -> None:
            time.sleep(0.02)
            coord.acquire_return_home("cart-2")
            order.append("cart-2")
            coord.release_return_home("cart-2")

        t1 = threading.Thread(target=first)
        t2 = threading.Thread(target=second)
        t1.start()
        t2.start()
        t1.join(timeout=5.0)
        t2.join(timeout=5.0)
        self.assertEqual(order[0], "cart-1-start")
        self.assertEqual(order[-1], "cart-2")

    def test_solo_mission_grants_when_plan_fails(self) -> None:
        port = FakeCartPort()
        port.plan_pose = lambda *a, **k: {"success": False, "message": "planner down"}
        coord = TrafficCoordinator(port, robot_codes=["cart-1", "cart-2"])
        coord.register_mission("cart-1", 1)
        coord.acquire_nav_leg("cart-1", NavGoal(1.0, 0.0, 0.0), 1, "W1")
        snap = coord.snapshot()
        self.assertEqual(snap["robots"]["cart-1"]["phase"], "NAVIGATING")
        coord.release_nav_leg("cart-1")
        coord.unregister_mission("cart-1")
        self.assertIsNone(coord.snapshot()["robots"]["cart-1"]["missionId"])

    def test_plan_failure_does_not_block_conflicting_owner(self) -> None:
        port = FakeCartPort()
        port.plan_pose = lambda *a, **k: {"success": False, "message": "no path"}
        coord = TrafficCoordinator(port, robot_codes=["cart-1", "cart-2"])
        coord.register_mission("cart-1", 1)
        coord.register_mission("cart-2", 2)
        coord.acquire_nav_leg("cart-1", NavGoal(2.0, 0.0, 0.0), 1, "W1")
        self.assertEqual(coord.snapshot()["robots"]["cart-1"]["phase"], "NAVIGATING")
        coord.release_nav_leg("cart-1")

    def test_interrupt_robot_unblocks_acquire(self) -> None:
        port = FakeCartPort()
        coord = TrafficCoordinator(port, robot_codes=["cart-1", "cart-2"])
        coord.register_mission("cart-1", 1)
        coord.register_mission("cart-2", 2)
        coord.acquire_nav_leg("cart-1", NavGoal(2.0, 0.0, 0.0), 1, "W1")
        port._navigating["cart-1"] = True
        errors: list[str] = []

        def waiter() -> None:
            try:
                coord.acquire_nav_leg("cart-2", NavGoal(2.0, 0.1, 0.0), 2, "W2")
            except TrafficTimeoutError as exc:
                errors.append(str(exc))

        t = threading.Thread(target=waiter)
        t.start()
        time.sleep(0.15)
        coord.interrupt_robot("cart-2")
        t.join(timeout=2.0)
        self.assertFalse(t.is_alive())
        self.assertTrue(errors)
        coord.release_nav_leg("cart-1")

    def test_claim_release_waypoint_zone(self) -> None:
        port = FakeCartPort()
        coord = TrafficCoordinator(port, robot_codes=["cart-1", "cart-2"])
        coord.register_mission("cart-1", 1)
        coord.claim_waypoint_zone("cart-1", "W5")
        snap = coord.snapshot()
        self.assertEqual(snap["robots"]["cart-1"]["occupiedWaypoint"], "W5")
        self.assertFalse(coord.try_claim_waypoint_zone("cart-2", "W5"))
        self.assertFalse(coord.waypoint_access_granted("cart-2", "W5"))
        zones = coord.occupied_zones(exclude_device="cart-1")
        self.assertEqual(len(zones), 0)
        zones_other = coord.occupied_zones(exclude_device="cart-2")
        self.assertEqual(len(zones_other), 1)
        coord.release_waypoint_zone("cart-1")
        self.assertIsNone(coord.snapshot()["robots"]["cart-1"]["occupiedWaypoint"])
        self.assertTrue(coord.try_claim_waypoint_zone("cart-2", "W5"))

    def test_path_blocked_by_occupied_zone(self) -> None:
        os.environ["TRAFFIC_ENABLED"] = "1"
        port = FakeCartPort()

        def plan_through_w5(device_code, x, y, yaw=0.0, timeout_sec=10.0):
            del timeout_sec, yaw, x, y
            return {
                "success": True,
                "path": {
                    "poses": [
                        {"x": 0.0, "y": 0.0},
                        {"x": 0.46, "y": -1.07},
                    ],
                },
            }

        port.plan_pose = plan_through_w5
        coord = TrafficCoordinator(port, robot_codes=["cart-1", "cart-2"])
        coord.register_mission("cart-1", 1)
        coord.register_mission("cart-2", 2)
        coord.claim_waypoint_zone("cart-1", "W5")
        granted: list[str] = []

        def waiter() -> None:
            from app.waypoints import get_waypoint

            w5 = get_waypoint("W5")
            coord.acquire_nav_leg("cart-2", NavGoal(w5.x, w5.y, w5.yaw), 2, "W5")
            granted.append("cart-2")

        t = threading.Thread(target=waiter)
        t.start()
        time.sleep(0.3)
        self.assertEqual(granted, [])
        coord.release_waypoint_zone("cart-1")
        t.join(timeout=5.0)
        self.assertIn("cart-2", granted)

    def test_acquire_waypoint_access(self) -> None:
        port = FakeCartPort()
        coord = TrafficCoordinator(port, robot_codes=["cart-1", "cart-2"])
        coord.register_mission("cart-1", 1)
        coord.register_mission("cart-2", 2)
        coord.claim_waypoint_zone("cart-1", "W3")
        self.assertFalse(coord.waypoint_access_granted("cart-2", "W3"))

        def release_later() -> None:
            time.sleep(0.15)
            coord.release_waypoint_zone("cart-1")

        t = threading.Thread(target=release_later)
        t.start()
        coord.acquire_waypoint_access("cart-2", "W3", timeout_sec=5.0)
        t.join(timeout=2.0)
        self.assertTrue(coord.waypoint_access_granted("cart-2", "W3"))

    def test_pack_wait_until_other_arrives_home(self) -> None:
        port = FakeCartPort()
        coord = TrafficCoordinator(port, robot_codes=["cart-1", "cart-2"])
        coord.register_mission("cart-1", 1)
        coord.register_mission("cart-2", 2)
        coord.mark_returning_home("cart-1")
        self.assertFalse(coord.waypoint_access_granted("cart-2", "P"))
        self.assertTrue(coord.waypoint_access_granted("cart-2", "W1"))

        granted: list[str] = []

        def waiter() -> None:
            coord.acquire_waypoint_access("cart-2", "P", timeout_sec=5.0)
            granted.append("cart-2")

        t = threading.Thread(target=waiter)
        t.start()
        time.sleep(0.2)
        self.assertEqual(granted, [])
        coord.release_return_home("cart-1")
        t.join(timeout=3.0)
        self.assertEqual(granted, ["cart-2"])
        self.assertTrue(coord.waypoint_access_granted("cart-2", "P"))

    def test_remaining_overlap_owner_not_waiter(self) -> None:
        port = FakeCartPort()
        coord = TrafficCoordinator(port, robot_codes=["cart-1", "cart-2"])
        coord.register_mission("cart-1", 10)
        time.sleep(0.01)
        coord.register_mission("cart-2", 20)
        coord.update_remaining("cart-1", ["W3", "C", "P"])
        coord.update_remaining("cart-2", ["W3", "W5", "C", "P"])
        self.assertTrue(coord.waypoint_access_granted("cart-1", "W3"))
        self.assertFalse(coord.waypoint_access_granted("cart-2", "W3"))


if __name__ == "__main__":
    unittest.main()
