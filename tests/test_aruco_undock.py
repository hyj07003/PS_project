from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _load_aruco_dock():
    path = ROOT / "pinky" / "modules" / "aruco_dock.py"
    spec = importlib.util.spec_from_file_location("aruco_dock_mod", path)
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    sys.modules["aruco_dock_mod"] = mod
    spec.loader.exec_module(mod)
    return mod


ad = _load_aruco_dock()


class ArucoUndockTests(unittest.TestCase):
    def test_mock_undock(self) -> None:
        result = ad.run_aruco_undock(
            5,
            0.15,
            lambda _lx, _az: None,
            mock=True,
        )
        self.assertTrue(result.get("success"))
        self.assertAlmostEqual(float(result["targetRangeM"]), 0.15, places=3)


if __name__ == "__main__":
    unittest.main()
