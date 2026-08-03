from __future__ import annotations

import sqlite3
import threading
from typing import Any

from ..adapters import ORDER_FLOW, MockAiAdapter, MockCartAdapter, MockStationAdapter
from ..db import now_iso
from ..errors import ApiError
from .carts import CartsService


class OrdersService:
    def __init__(self, conn: sqlite3.Connection, carts: CartsService):
        self.conn = conn
        self.carts = carts
        self.cart_port = MockCartAdapter()
        self.station_port = MockStationAdapter()
        self.ai_port = MockAiAdapter()

    def get_by_id(self, order_id: int) -> dict[str, Any]:
        order = self.conn.execute(
            "SELECT * FROM orders WHERE id = ?",
            (order_id,),
        ).fetchone()
        if not order:
            raise ApiError(404, "order not found")

        items = self.conn.execute(
            "SELECT * FROM order_items WHERE order_id = ?",
            (order_id,),
        ).fetchall()

        return {
            "id": order["id"],
            "userId": order["user_id"],
            "status": order["status"],
            "totalPrice": order["total_price"],
            "createdAt": order["created_at"],
            "updatedAt": order["updated_at"],
            "items": [
                {
                    "id": i["id"],
                    "productId": i["product_id"],
                    "productName": i["product_name"],
                    "unitPrice": i["unit_price"],
                    "quantity": i["quantity"],
                }
                for i in items
            ],
        }

    def create_from_cart(self, user_id: int) -> dict[str, Any]:
        cart = self.carts.get_cart(user_id)
        if not cart["items"]:
            raise ApiError(400, "cart is empty")

        ts = now_iso()
        total = sum(
            (item["product"]["price"] if item.get("product") else 0) * item["quantity"]
            for item in cart["items"]
        )

        try:
            with self.conn.transaction() as db:
                cur = db.execute(
                    """
                    INSERT INTO orders (user_id, status, total_price, created_at, updated_at)
                    VALUES (?, 'CREATED', ?, ?, ?)
                    """,
                    (user_id, total, ts, ts),
                )
                order_id = cur.lastrowid

                for item in cart["items"]:
                    db.execute(
                        """
                        INSERT INTO order_items (order_id, product_id, product_name, unit_price, quantity)
                        VALUES (?, ?, ?, ?, ?)
                        """,
                        (
                            order_id,
                            item["productId"],
                            item["product"]["name"],
                            item["product"]["price"],
                            item["quantity"],
                        ),
                    )

                mission_cur = db.execute(
                    """
                    INSERT INTO missions (order_id, device_id, status, created_at)
                    VALUES (?, NULL, 'CREATED', ?)
                    """,
                    (order_id, ts),
                )
                mission_id = mission_cur.lastrowid
                db.execute(
                    """
                    INSERT INTO mission_events (mission_id, from_status, to_status, note, created_at)
                    VALUES (?, NULL, 'CREATED', 'order created', ?)
                    """,
                    (mission_id, ts),
                )
        except Exception:
            raise

        self.carts.clear(user_id)

        thread = threading.Thread(
            target=self._run_mock_pipeline,
            args=(order_id, mission_id),
            daemon=True,
        )
        thread.start()
        return self.get_by_id(order_id)  # type: ignore[arg-type]

    def _set_status(self, order_id: int, mission_id: int, status: str) -> None:
        current = self.conn.execute(
            "SELECT status FROM orders WHERE id = ?",
            (order_id,),
        ).fetchone()
        ts = now_iso()
        self.conn.execute(
            "UPDATE orders SET status = ?, updated_at = ? WHERE id = ?",
            (status, ts, order_id),
        )
        self.conn.execute(
            "UPDATE missions SET status = ? WHERE id = ?",
            (status, mission_id),
        )
        flow_idx = ORDER_FLOW.index(status) if status in ORDER_FLOW else -1
        self.conn.execute(
            """
            INSERT INTO mission_events (mission_id, from_status, to_status, note, created_at)
            VALUES (?, ?, ?, ?, ?)
            """,
            (
                mission_id,
                current["status"] if current else None,
                status,
                f"flow:{flow_idx}",
                ts,
            ),
        )
        self.conn.commit()

    def _run_mock_pipeline(self, order_id: int, mission_id: int) -> None:
        try:
            self.ai_port.request_pick_plan(order_id)
            assigned = self.cart_port.assign_cart(order_id)
            if assigned:
                device = self.conn.execute(
                    "SELECT id FROM devices WHERE code = ?",
                    (assigned["deviceCode"],),
                ).fetchone()
                if device:
                    self.conn.execute(
                        "UPDATE missions SET device_id = ? WHERE id = ?",
                        (device["id"], mission_id),
                    )
                    self.conn.execute(
                        "UPDATE devices SET status = 'busy' WHERE id = ?",
                        (device["id"],),
                    )
                    self.conn.commit()
                self.cart_port.navigate(assigned["deviceCode"], "aisle-a")

            self._set_status(order_id, mission_id, "ASSIGNED")
            self.station_port.start_picking(order_id)
            self._set_status(order_id, mission_id, "PICKING")
            self.station_port.checkout(order_id)
            self._set_status(order_id, mission_id, "CHECKOUT")
            self.station_port.pack(order_id)
            self._set_status(order_id, mission_id, "PACKING")
            self._set_status(order_id, mission_id, "COMPLETED")

            mission = self.conn.execute(
                "SELECT device_id FROM missions WHERE id = ?",
                (mission_id,),
            ).fetchone()
            if mission and mission["device_id"]:
                self.conn.execute(
                    "UPDATE devices SET status = 'idle' WHERE id = ?",
                    (mission["device_id"],),
                )
                self.conn.commit()
        except Exception:
            self._set_status(order_id, mission_id, "FAILED")

    def list_devices(self) -> list[dict[str, Any]]:
        rows = self.conn.execute("SELECT * FROM devices").fetchall()
        return [dict(r) for r in rows]
