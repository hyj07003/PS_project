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
