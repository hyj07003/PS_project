from __future__ import annotations

import importlib.util
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / "server" / "apps" / "controller-server" / "app" / "services" / "traffic_paths.py"
spec = importlib.util.spec_from_file_location("traffic_paths", MODULE)
assert spec and spec.loader
traffic_paths = importlib.util.module_from_spec(spec)
spec.loader.exec_module(traffic_paths)

path_points = traffic_paths.path_points
find_path_conflicts = traffic_paths.find_path_conflicts
retreat_path_index = traffic_paths.retreat_path_index
advance_path_index = traffic_paths.advance_path_index
path_distance_between = traffic_paths.path_distance_between
has_passed_path_index = traffic_paths.has_passed_path_index
path_intersects_disc = traffic_paths.path_intersects_disc
path_intersects_zones = traffic_paths.path_intersects_zones


class TrafficPathsTests(unittest.TestCase):
    def test_path_points_from_plan_response(self) -> None:
        payload = {
            "success": True,
            "path": {
                "poses": [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 0.0}],
            },
        }
        self.assertEqual(path_points(payload), [(0.0, 0.0), (1.0, 0.0)])

    def test_find_path_conflicts_detects_overlap(self) -> None:
        path1 = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
        path2 = [(1.0, 0.05), (2.0, 0.05)]
        result = find_path_conflicts(path1, path2, clearance_m=0.20)
        self.assertTrue(result["conflict"])
        self.assertTrue(result["segments"])
        seg = result["segments"][0]
        self.assertIn("cart1_start_index", seg)
        self.assertIn("cart2_end_index", seg)

    def test_find_path_conflicts_clear_paths(self) -> None:
        path1 = [(0.0, 0.0), (1.0, 0.0)]
        path2 = [(0.0, 2.0), (1.0, 2.0)]
        result = find_path_conflicts(path1, path2, clearance_m=0.20)
        self.assertFalse(result["conflict"])
        self.assertEqual(result["segments"], [])

    def test_retreat_and_advance_indices(self) -> None:
        path = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.5, 0.0)]
        hold = retreat_path_index(path, 2, 0.40)
        self.assertEqual(hold, 1)
        release = advance_path_index(path, 1, 0.40)
        self.assertEqual(release, 2)
        self.assertAlmostEqual(path_distance_between(path, hold, 2), 0.5, places=6)

    def test_has_passed_path_index(self) -> None:
        path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
        pose_far = {"x": 0.4, "y": 0.0, "yaw": 0.0}
        pose_near_end = {"x": 1.8, "y": 0.0, "yaw": 0.0}
        self.assertFalse(has_passed_path_index(path, pose_far, 2))
        self.assertTrue(has_passed_path_index(path, pose_near_end, 1))

    def test_path_intersects_disc(self) -> None:
        path = [(0.0, 0.0), (2.0, 0.0)]
        self.assertTrue(path_intersects_disc(path, 1.0, 0.05, 0.10))
        self.assertFalse(path_intersects_disc(path, 1.0, 0.50, 0.10))

    def test_path_intersects_zones(self) -> None:
        path = [(0.0, 0.0), (2.0, 0.0)]
        zones = [(1.0, 0.50, 0.10), (3.0, 0.0, 0.10)]
        self.assertFalse(path_intersects_zones(path, zones))
        zones_block = [(1.0, 0.05, 0.10)]
        self.assertTrue(path_intersects_zones(path, zones_block))


if __name__ == "__main__":
    unittest.main()
