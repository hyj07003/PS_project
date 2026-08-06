from __future__ import annotations

import os
import threading
import time
from typing import Any

from ..adapters import (
    ORDER_FLOW,
    MockAiAdapter,
    MockCartAdapter,
    MockStationAdapter,
    PinkyHttpCartAdapter,
    parse_pinky_robot_urls,
)
from ..db import now_iso
from ..errors import ApiError
from ..waypoints import (
    WAYPOINTS,
    get_waypoint,
    home_for_device,
    nearest_neighbor_order,
    waypoint_ids_for_slugs,
)
from .carts import CartsService

_dispatch_lock = threading.Lock()

ACTIVE_STATUSES = ("ASSIGNED", "PICKING", "CHECKOUT", "PACKING", "RETURNING")


class OrdersService:
    def __init__(self, conn, carts: CartsService):
        self.conn = conn
        self.carts = carts
        urls = parse_pinky_robot_urls()
        if urls:
            self.cart_port = PinkyHttpCartAdapter(urls)
        else:
            self.cart_port = MockCartAdapter()
        self.station_port = MockStationAdapter()
        self.ai_port = MockAiAdapter()
        self._dwell_sec = float(os.environ.get("PICK_DWELL_SEC", "3"))
        self._nav_timeout = float(os.environ.get("PICK_NAV_TIMEOUT_SEC", "180"))

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
                VALUES (?, NULL, 'CREATED', 'order queued', ?)
                """,
                (mission_id, ts),
            )

        self.carts.clear(user_id)
        # Dispatch ASAP (reclaim stuck carts first)
        self.reclaim_stale_carts()
        threading.Thread(target=self.try_dispatch, daemon=True).start()
        # Also try inline so assignment is visible before response when possible
        try:
            self.try_dispatch()
        except Exception:
            pass
        return self.get_by_id(order_id)  # type: ignore[arg-type]

    def reclaim_stale_carts(self) -> int:
        """
        idle로 되돌림: active 미션이 없는데 busy/error 인 cart.
        (이전 실패·재시작으로 할당이 멈춘 경우 복구)
        """
        rows = self.conn.execute(
            """
            SELECT d.id, d.code, d.status
            FROM devices d
            WHERE d.type = 'cart' AND d.status IN ('busy', 'error')
            """
        ).fetchall()
        fixed = 0
        for d in rows:
            active = self.conn.execute(
                """
                SELECT m.id FROM missions m
                WHERE m.device_id = ?
                  AND m.status IN ('ASSIGNED', 'PICKING', 'CHECKOUT', 'PACKING', 'RETURNING')
                LIMIT 1
                """,
                (d["id"],),
            ).fetchone()
            if active:
                continue
            self.conn.execute(
                "UPDATE devices SET status = 'idle' WHERE id = ?",
                (d["id"],),
            )
            fixed += 1
        if fixed:
            self.conn.commit()
        return fixed

    def try_dispatch(self) -> None:
        """Assign FIFO CREATED missions to idle cart devices."""
        with _dispatch_lock:
            self.reclaim_stale_carts()
            while True:
                mission = self.conn.execute(
                    """
                    SELECT m.id, m.order_id
                    FROM missions m
                    WHERE m.status = 'CREATED' AND m.device_id IS NULL
                    ORDER BY m.id ASC
                    LIMIT 1
                    """
                ).fetchone()
                if not mission:
                    return

                device = None
                candidates = self.conn.execute(
                    """
                    SELECT id, code FROM devices
                    WHERE type = 'cart' AND status = 'idle'
                    ORDER BY code ASC
                    """
                ).fetchall()
                # Prefer reachable Pinky; skip offline cart-1 when only cart-2 is up
                for cand in candidates:
                    code = str(cand["code"])
                    if self.cart_port.is_reachable(code):
                        device = cand
                        break
                if not device:
                    # Do not assign to unreachable carts (goals would fail instantly)
                    return

                order_id = int(mission["order_id"])
                mission_id = int(mission["id"])
                device_id = int(device["id"])
                device_code = str(device["code"])

                self.conn.execute(
                    "UPDATE missions SET device_id = ? WHERE id = ?",
                    (device_id, mission_id),
                )
                self.conn.execute(
                    "UPDATE devices SET status = 'busy' WHERE id = ?",
                    (device_id,),
                )
                self.conn.commit()
                self._set_status(order_id, mission_id, "ASSIGNED", note="dispatched")

                threading.Thread(
                    target=self._run_pick_tour,
                    args=(order_id, mission_id, device_code),
                    daemon=True,
                ).start()

    def _set_status(
        self,
        order_id: int,
        mission_id: int,
        status: str,
        note: str | None = None,
    ) -> None:
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
                note or f"flow:{flow_idx}",
                ts,
            ),
        )
        self.conn.commit()

    def _mission_note(
        self,
        mission_id: int,
        note: str,
        *,
        status: str | None = None,
    ) -> None:
        """Append mission_events without changing order flow (for dwell markers)."""
        row = self.conn.execute(
            "SELECT status FROM missions WHERE id = ?",
            (mission_id,),
        ).fetchone()
        cur = status or (row["status"] if row else "PICKING")
        self.conn.execute(
            """
            INSERT INTO mission_events (mission_id, from_status, to_status, note, created_at)
            VALUES (?, ?, ?, ?, ?)
            """,
            (mission_id, cur, cur, note, now_iso()),
        )
        self.conn.commit()

    def _set_waypoint(
        self,
        mission_id: int,
        waypoint_id: str | None,
        *,
        label_suffix: str | None = None,
    ) -> None:
        if waypoint_id and waypoint_id in WAYPOINTS:
            wp = WAYPOINTS[waypoint_id]
            label = wp.label
            if label_suffix:
                label = f"{label} · {label_suffix}"
            self.conn.execute(
                """
                UPDATE missions
                SET current_waypoint = ?, current_waypoint_label = ?
                WHERE id = ?
                """,
                (wp.id, label, mission_id),
            )
        else:
            self.conn.execute(
                """
                UPDATE missions
                SET current_waypoint = NULL, current_waypoint_label = NULL
                WHERE id = ?
                """,
                (mission_id,),
            )
        self.conn.commit()

    def _nav_or_fail(
        self,
        device_code: str,
        x: float,
        y: float,
        yaw: float,
        waypoint_id: str,
    ) -> None:
        # Nav2 aborts/replans mid-leg often; retry before failing the whole tour.
        attempts = max(1, int(os.environ.get("PICK_NAV_RETRIES", "3")))
        last_detail = "unknown"
        for attempt in range(1, attempts + 1):
            result = self.cart_port.navigate_pose(
                device_code,
                x,
                y,
                yaw,
                timeout_sec=self._nav_timeout,
            )
            if result == "ARRIVED":
                return
            last_detail = getattr(self.cart_port, "last_nav_error", None) or "unknown"
            if attempt < attempts:
                time.sleep(1.0)
                continue
            raise RuntimeError(
                f"nav failed at {waypoint_id} after {attempts} tries: {last_detail}"
            )

    def _dwell_at(
        self,
        device_code: str,
        mission_id: int,
        waypoint_id: str,
    ) -> None:
        """Stop at goal, then wait PICK_DWELL_SEC (default 3s) before next goal."""
        try:
            self.cart_port.stop_nav(device_code)
        except Exception:
            pass
        dwell = max(0.0, float(self._dwell_sec))
        self._set_waypoint(
            mission_id,
            waypoint_id,
            label_suffix=f"대기 {dwell:g}초",
        )
        self._mission_note(mission_id, f"dwell start {waypoint_id} {dwell}s")
        if dwell > 0:
            time.sleep(dwell)
        self._mission_note(mission_id, f"dwell end {waypoint_id}")
        self._set_waypoint(mission_id, waypoint_id)

    def _return_home(
        self,
        device_code: str,
        mission_id: int,
        *,
        stop_first: bool = False,
        best_effort: bool = False,
    ) -> bool:
        """Navigate to device wait spot (S1/S2). best_effort: never raise."""
        home = home_for_device(device_code)
        if stop_first:
            try:
                self.cart_port.stop_nav(device_code)
                time.sleep(0.3)
            except Exception:
                pass
        self._set_waypoint(mission_id, home.id)
        try:
            self._nav_or_fail(device_code, home.x, home.y, home.yaw, home.id)
            self._dwell_at(device_code, mission_id, home.id)
            return True
        except Exception:
            if best_effort:
                return False
            raise

    def _release_device(self, mission_id: int) -> None:
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

    def _run_pick_tour(
        self, order_id: int, mission_id: int, device_code: str
    ) -> None:
        home = home_for_device(device_code)
        try:
            self.ai_port.request_pick_plan(order_id)
            self.cart_port.notify_assign(device_code, order_id)

            # 최초 이니셜 포즈: 홈(S1/S2), 오른쪽(+x, yaw=0)
            self.cart_port.set_initial_pose(
                device_code, home.x, home.y, home.yaw
            )
            # AMCL settle after initialpose
            time.sleep(float(os.environ.get("PICK_INITIALPOSE_SETTLE_SEC", "1.5")))

            pose = self.cart_port.get_pose(device_code)
            if pose:
                from ..waypoints import Waypoint

                start = Waypoint(
                    id="START",
                    label="current",
                    x=pose["x"],
                    y=pose["y"],
                    yaw=float(pose.get("yaw") or 0.0),
                )
            else:
                start = home

            rows = self.conn.execute(
                """
                SELECT oi.product_id, p.slug
                FROM order_items oi
                JOIN products p ON p.id = oi.product_id
                WHERE oi.order_id = ?
                """,
                (order_id,),
            ).fetchall()
            slugs = [r["slug"] for r in rows if r["slug"]]
            shelf_ids = waypoint_ids_for_slugs(slugs)
            tour = nearest_neighbor_order(start, shelf_ids)

            self._set_status(order_id, mission_id, "PICKING", note="pick tour start")

            for wp in tour:
                self._set_waypoint(mission_id, wp.id)
                self._nav_or_fail(device_code, wp.x, wp.y, wp.yaw, wp.id)
                self._dwell_at(device_code, mission_id, wp.id)

            self._set_status(order_id, mission_id, "CHECKOUT", note="checkout")
            c = get_waypoint("C")
            self._set_waypoint(mission_id, "C")
            self._nav_or_fail(device_code, c.x, c.y, c.yaw, "C")
            self._dwell_at(device_code, mission_id, "C")

            self._set_status(order_id, mission_id, "PACKING", note="transport wait")
            p = get_waypoint("P")
            self._set_waypoint(mission_id, "P")
            self._nav_or_fail(device_code, p.x, p.y, p.yaw, "P")
            self._dwell_at(device_code, mission_id, "P")

            # 대기장소 복귀 중 — 모니터링 할당은 이때까지 유지, 도착 후 COMPLETED
            self._set_status(
                order_id, mission_id, "RETURNING", note="returning to wait spot"
            )
            self._return_home(device_code, mission_id, best_effort=False)

            self._set_waypoint(mission_id, None)
            self._set_status(
                order_id,
                mission_id,
                "COMPLETED",
                note="arrived wait spot — job done",
            )
            self._release_device(mission_id)
        except Exception as exc:
            # 실패해도 대기장소 복귀 완료 전까지는 RETURNING 으로 할당 표시 유지
            try:
                self._set_status(
                    order_id,
                    mission_id,
                    "RETURNING",
                    note=f"abort return home:{exc}",
                )
            except Exception:
                pass
            home_ok = self._return_home(
                device_code,
                mission_id,
                stop_first=True,
                best_effort=True,
            )
            note = f"failed:{exc}"
            if home_ok:
                note = f"{note}; arrived wait spot"
            else:
                note = f"{note}; home return failed"
            self._set_waypoint(mission_id, None)
            self._set_status(order_id, mission_id, "FAILED", note=note)
            self._release_device(mission_id)
        finally:
            threading.Thread(target=self.try_dispatch, daemon=True).start()

    def queue_length(self) -> int:
        row = self.conn.execute(
            """
            SELECT COUNT(*) AS c FROM missions
            WHERE status = 'CREATED' AND device_id IS NULL
            """
        ).fetchone()
        return int(row["c"] if row else 0)

    def list_devices(self) -> list[dict[str, Any]]:
        rows = self.conn.execute("SELECT * FROM devices").fetchall()
        return [dict(r) for r in rows]
