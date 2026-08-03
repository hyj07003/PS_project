from __future__ import annotations

import time
from typing import Literal

ORDER_FLOW = [
    "CREATED",
    "ASSIGNED",
    "PICKING",
    "CHECKOUT",
    "PACKING",
    "COMPLETED",
]


class MockCartAdapter:
    def __init__(self) -> None:
        self._next = 0

    def assign_cart(self, order_id: int = 0) -> dict[str, str] | None:
        time.sleep(0.2)
        self._next = (self._next % 2) + 1
        return {"deviceCode": f"cart-{self._next}"}

    def navigate(
        self, device_code: str = "", waypoint: str = ""
    ) -> Literal["ARRIVED", "FAILED"]:
        time.sleep(0.8)
        return "ARRIVED"


class MockStationAdapter:
    def start_picking(self, order_id: int = 0) -> Literal["DONE", "FAILED"]:
        time.sleep(0.6)
        return "DONE"

    def checkout(self, order_id: int = 0) -> Literal["DONE", "FAILED"]:
        time.sleep(0.4)
        return "DONE"

    def pack(self, order_id: int = 0) -> Literal["DONE", "FAILED"]:
        time.sleep(0.5)
        return "DONE"


class MockAiAdapter:
    def request_pick_plan(self, order_id: int = 0) -> dict[str, list[str]]:
        time.sleep(0.15)
        return {"waypoints": ["aisle-a", "aisle-b", "checkout"]}


class PinkyHttpCartAdapter:
    """PINKY_URL 이 설정된 경우 실제 pinky Flask 서버로 assign/navigate."""

    def __init__(self, base_url: str):
        self.base_url = base_url.rstrip("/")
        self._next = 0

    def _post(self, path: str, body: dict) -> dict:
        import json
        import urllib.request

        req = urllib.request.Request(
            f"{self.base_url}{path}",
            data=json.dumps(body).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(req, timeout=10) as res:
            return json.loads(res.read().decode("utf-8"))

    def assign_cart(self, order_id: int = 0) -> dict[str, str] | None:
        self._next = (self._next % 2) + 1
        code = f"cart-{self._next}"
        try:
            self._post(
                "/cmd/assign",
                {"orderId": order_id, "deviceCode": code},
            )
        except Exception:
            pass
        return {"deviceCode": code}

    def navigate(
        self, device_code: str = "", waypoint: str = ""
    ) -> Literal["ARRIVED", "FAILED"]:
        try:
            result = self._post(
                "/cmd/navigate",
                {"deviceCode": device_code, "waypoint": waypoint},
            )
            status = result.get("status", "ARRIVED")
            return "ARRIVED" if status == "ARRIVED" else "FAILED"
        except Exception:
            return "FAILED"
