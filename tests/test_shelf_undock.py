from __future__ import annotations

import importlib.util
import os
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


def _load_waypoints():
    import sys

    path = ROOT / "server" / "apps" / "controller-server" / "app" / "waypoints.py"
    spec = importlib.util.spec_from_file_location("waypoints_mod", path)
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    sys.modules["waypoints_mod"] = mod
    spec.loader.exec_module(mod)
    return mod


wp = _load_waypoints()


class ShelfUndockDistanceTests(unittest.TestCase):
    def test_checkout_does_not_undock(self) -> None:
        self.assertFalse(wp.shelf_undock_after_aruco("C"))
        self.assertEqual(wp.shelf_undock_distance_m("C", 0.40), 0.0)
        self.assertTrue(wp.shelf_undock_after_aruco("P"))

    def test_home_does_not_undock(self) -> None:
        self.assertFalse(wp.shelf_undock_after_aruco("S1"))
        self.assertEqual(wp.shelf_undock_distance_m("S1", 0.20), 0.0)

    def test_uses_pre_approach_marker_range(self) -> None:
        self.assertAlmostEqual(wp.shelf_undock_distance_m("W5", 0.32), 0.32, places=3)

    def test_missing_approach_is_zero(self) -> None:
        self.assertEqual(wp.shelf_undock_distance_m("W5", None), 0.0)

    def test_max_cap(self) -> None:
        with patch.dict(os.environ, {"PICK_SHELF_UNDOCK_MAX_M": "0.30"}, clear=False):
            self.assertAlmostEqual(wp.shelf_undock_distance_m("W1", 0.50), 0.30, places=3)


if __name__ == "__main__":
    unittest.main()
