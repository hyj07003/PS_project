from __future__ import annotations

import sys
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "server" / "apps" / "controller-server"))
if "flask_cors" not in sys.modules:
    sys.modules["flask_cors"] = types.SimpleNamespace(CORS=lambda *a, **k: None)

from app.adapters import OmxHttpStationAdapter  # noqa: E402


class OmxAdapterTests(unittest.TestCase):
    def test_pick_done_with_progress(self) -> None:
        adapter = OmxHttpStationAdapter(url="http://omx.local", poll_sec=0.01)
        events: list[tuple[int, int]] = []
        calls = {"n": 0}

        def fake_post(path, body, timeout=10.0):
            del timeout
            if path == "/pick":
                self.assertEqual(body["slug"], "milk")
                return {"success": True, "status": "RUNNING", "done": 0, "total": 2}
            raise AssertionError(path)

        def fake_get(path, timeout=5.0):
            del timeout
            self.assertEqual(path, "/pick/state")
            calls["n"] += 1
            if calls["n"] == 1:
                return {"success": True, "status": "RUNNING", "done": 1, "total": 2}
            return {"success": True, "status": "DONE", "done": 2, "total": 2}

        adapter._post = fake_post  # type: ignore[method-assign]
        adapter._get = fake_get  # type: ignore[method-assign]

        result = adapter.pick(
            device_code="cart-1",
            slug="milk",
            quantity=2,
            order_id=1,
            timeout_sec=2.0,
            on_progress=lambda d, t: events.append((d, t)),
        )
        self.assertEqual(result, "DONE")
        self.assertIn((1, 2), events)
        self.assertIn((2, 2), events)

    def test_pick_unreachable_force_success(self) -> None:
        adapter = OmxHttpStationAdapter(url="http://omx.local", poll_sec=0.01)

        def fake_post(path, body, timeout=10.0):
            del path, body, timeout
            raise OSError("connection refused")

        adapter._post = fake_post  # type: ignore[method-assign]
        result = adapter.pick(
            device_code="cart-1",
            slug="milk",
            quantity=1,
            order_id=1,
            force_success_on_unreachable=True,
        )
        self.assertEqual(result, "DONE")
        self.assertIn("failed", (adapter.last_error or "").lower())

    def test_pick_failed_status_is_not_overridden(self) -> None:
        adapter = OmxHttpStationAdapter(url="http://omx.local", poll_sec=0.01)

        def fake_post(path, body, timeout=10.0):
            del body, timeout
            if path == "/pick":
                return {"success": True, "status": "RUNNING"}
            raise AssertionError(path)

        def fake_get(path, timeout=5.0):
            del timeout
            self.assertEqual(path, "/pick/state")
            return {"success": True, "status": "FAILED", "message": "grasp failed", "done": 0}

        adapter._post = fake_post  # type: ignore[method-assign]
        adapter._get = fake_get  # type: ignore[method-assign]
        result = adapter.pick(
            device_code="cart-2",
            slug="sandwich",
            quantity=1,
            order_id=2,
            force_success_on_unreachable=True,
        )
        self.assertEqual(result, "FAILED")
        self.assertEqual(adapter.last_error, "grasp failed")


if __name__ == "__main__":
    unittest.main()
