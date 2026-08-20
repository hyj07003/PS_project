from __future__ import annotations

import importlib.util
import os
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "server" / "apps" / "controller-server"))

spec = importlib.util.spec_from_file_location(
    "waypoints",
    ROOT / "server" / "apps" / "controller-server" / "app" / "waypoints.py",
)
assert spec and spec.loader
waypoints = importlib.util.module_from_spec(spec)
sys.modules["waypoints"] = waypoints
spec.loader.exec_module(waypoints)


class WaypointZoneTests(unittest.TestCase):
    def setUp(self) -> None:
        self._env_backup = dict(os.environ)

    def tearDown(self) -> None:
        os.environ.clear()
        os.environ.update(self._env_backup)

    def test_w7_staging_waypoint_defined(self) -> None:
        w7 = waypoints.get_waypoint("W7")
        self.assertEqual(w7.label, "충돌 대기")
        self.assertAlmostEqual(w7.x, 0.090)
        self.assertAlmostEqual(w7.y, -0.498)
        self.assertAlmostEqual(w7.yaw, 0.0)

    def test_zone_center_by_id_not_shared_xy(self) -> None:
        w6 = waypoints.waypoint_zone_center("W6")
        c = waypoints.waypoint_zone_center("C")
        self.assertEqual(w6, c)
        self.assertTrue(waypoints.is_zone_occupiable("W6"))
        self.assertTrue(waypoints.is_zone_occupiable("C"))
        self.assertFalse(waypoints.is_zone_occupiable("W7"))
        self.assertFalse(waypoints.is_zone_occupiable("S1"))

    def test_zone_radius_from_env(self) -> None:
        os.environ["TRAFFIC_ZONE_RADIUS_M"] = "0.55"
        self.assertAlmostEqual(waypoints.waypoint_zone_radius_m("W1"), 0.55)

    def test_conflict_aware_tour_defers_shelves(self) -> None:
        start = waypoints.get_waypoint("S1")
        order = waypoints.conflict_aware_tour_order(
            start,
            ["W3", "W1", "W5"],
            defer_ids={"W3", "W5"},
        )
        ids = [wp.id for wp in order]
        self.assertEqual(ids[0], "W1")
        self.assertEqual(set(ids[1:]), {"W3", "W5"})

    def test_nearest_neighbor_unchanged_without_defer(self) -> None:
        start = waypoints.get_waypoint("S1")
        ids_a = [wp.id for wp in waypoints.nearest_neighbor_order(start, ["W3", "W1"])]
        ids_b = [
            wp.id
            for wp in waypoints.conflict_aware_tour_order(start, ["W3", "W1"], defer_ids=set())
        ]
        self.assertEqual(ids_a, ids_b)


if __name__ == "__main__":
    unittest.main()
