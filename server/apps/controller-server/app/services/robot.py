from __future__ import annotations

from typing import Any

from ..db import now_iso
from ..errors import ApiError

ACTIVE_STATUSES = ("ASSIGNED", "PICKING", "CHECKOUT", "PACKING", "RETURNING")


class RobotService:
    """Missions / devices / telemetry for Pinky robot bridge."""

    def __init__(self, conn, orders_service=None):
        self.conn = conn
        self._telemetry: dict[str, Any] = {}
        self._orders = orders_service

    def set_orders_service(self, orders_service) -> None:
        self._orders = orders_service

    def list_missions(
        self,
        status: str | None = None,
        device_code: str | None = None,
        active: bool = False,
        include_order: bool = False,
    ) -> list[dict[str, Any]]:
        sql = """
          SELECT m.id, m.order_id, m.device_id, m.status, m.created_at,
                 m.current_waypoint, m.current_waypoint_label,
                 d.code AS device_code
          FROM missions m
          LEFT JOIN devices d ON d.id = m.device_id
        """
        where: list[str] = []
        args: list[Any] = []
        if active:
            placeholders = ",".join("?" for _ in ACTIVE_STATUSES)
            where.append(f"m.status IN ({placeholders})")
            args.extend(ACTIVE_STATUSES)
        elif status:
            where.append("m.status = ?")
            args.append(status)
        if device_code:
            where.append("d.code = ?")
            args.append(device_code)
        if where:
            sql += " WHERE " + " AND ".join(where)
        sql += " ORDER BY m.id DESC"
        rows = self.conn.execute(sql, args).fetchall()
        out = [self._map_mission(dict(r)) for r in rows]
        if include_order:
            for m in out:
                m["order"] = self._order_summary(m["orderId"])
        return out

    def get_active_for_device(self, device_code: str) -> dict[str, Any] | None:
        rows = self.list_missions(
            device_code=device_code, active=True, include_order=True
        )
        return rows[0] if rows else None

    def queue_length(self) -> int:
        row = self.conn.execute(
            """
            SELECT COUNT(*) AS c FROM missions
            WHERE status = 'CREATED' AND device_id IS NULL
            """
        ).fetchone()
        return int(row["c"] if row else 0)

    def _order_summary(self, order_id: int) -> dict[str, Any] | None:
        order = self.conn.execute(
            "SELECT * FROM orders WHERE id = ?",
            (order_id,),
        ).fetchone()
        if not order:
            return None
        items = self.conn.execute(
            "SELECT * FROM order_items WHERE order_id = ?",
            (order_id,),
        ).fetchall()
        return {
            "id": order["id"],
            "status": order["status"],
            "totalPrice": order["total_price"],
            "createdAt": order["created_at"],
            "items": [
                {
                    "productId": i["product_id"],
                    "productName": i["product_name"],
                    "unitPrice": i["unit_price"],
                    "quantity": i["quantity"],
                }
                for i in items
            ],
        }

    def get_mission(self, mission_id: int) -> dict[str, Any]:
        row = self.conn.execute(
            """
            SELECT m.id, m.order_id, m.device_id, m.status, m.created_at,
                   m.current_waypoint, m.current_waypoint_label,
                   d.code AS device_code
            FROM missions m
            LEFT JOIN devices d ON d.id = m.device_id
            WHERE m.id = ?
            """,
            (mission_id,),
        ).fetchone()
        if not row:
            raise ApiError(404, "mission not found")
        mission = self._map_mission(dict(row))
        mission["order"] = self._order_summary(mission["orderId"])
        events = self.conn.execute(
            """
            SELECT id, from_status, to_status, note, created_at
            FROM mission_events WHERE mission_id = ? ORDER BY id ASC
            """,
            (mission_id,),
        ).fetchall()
        mission["events"] = [
            {
                "id": e["id"],
                "fromStatus": e["from_status"],
                "toStatus": e["to_status"],
                "note": e["note"],
                "createdAt": e["created_at"],
            }
            for e in events
        ]
        return mission

    def patch_mission(
        self,
        mission_id: int,
        status: str,
        note: str | None = None,
    ) -> dict[str, Any]:
        current = self.conn.execute(
            "SELECT id, order_id, status FROM missions WHERE id = ?",
            (mission_id,),
        ).fetchone()
        if not current:
            raise ApiError(404, "mission not found")

        ts = now_iso()
        self.conn.execute(
            "UPDATE missions SET status = ? WHERE id = ?",
            (status, mission_id),
        )
        self.conn.execute(
            "UPDATE orders SET status = ?, updated_at = ? WHERE id = ?",
            (status, ts, current["order_id"]),
        )
        self.conn.execute(
            """
            INSERT INTO mission_events (mission_id, from_status, to_status, note, created_at)
            VALUES (?, ?, ?, ?, ?)
            """,
            (
                mission_id,
                current["status"],
                status,
                note or f"robot:{status}",
                ts,
            ),
        )

        if status == "COMPLETED":
            mission = self.conn.execute(
                "SELECT device_id FROM missions WHERE id = ?",
                (mission_id,),
            ).fetchone()
            if mission and mission["device_id"]:
                self.conn.execute(
                    "UPDATE devices SET status = 'idle' WHERE id = ?",
                    (mission["device_id"],),
                )
                self.conn.execute(
                    """
                    UPDATE missions
                    SET current_waypoint = NULL, current_waypoint_label = NULL
                    WHERE id = ?
                    """,
                    (mission_id,),
                )
        elif status in ("ASSIGNED", "PICKING", "CHECKOUT", "PACKING", "RETURNING"):
            mission = self.conn.execute(
                "SELECT device_id FROM missions WHERE id = ?",
                (mission_id,),
            ).fetchone()
            if mission and mission["device_id"]:
                self.conn.execute(
                    "UPDATE devices SET status = 'busy' WHERE id = ?",
                    (mission["device_id"],),
                )
        elif status == "FAILED":
            mission = self.conn.execute(
                "SELECT device_id FROM missions WHERE id = ?",
                (mission_id,),
            ).fetchone()
            if mission and mission["device_id"]:
                self.conn.execute(
                    "UPDATE devices SET status = 'idle' WHERE id = ?",
                    (mission["device_id"],),
                )
                self.conn.execute(
                    """
                    UPDATE missions
                    SET current_waypoint = NULL, current_waypoint_label = NULL
                    WHERE id = ?
                    """,
                    (mission_id,),
                )

        self.conn.commit()
        result = self.get_mission(mission_id)
        if status in ("COMPLETED", "FAILED") and self._orders is not None:
            import threading

            threading.Thread(target=self._orders.try_dispatch, daemon=True).start()
        return result

    def patch_device(self, code: str, status: str) -> dict[str, Any]:
        allowed = {"idle", "busy", "error", "offline"}
        if status not in allowed:
            raise ApiError(400, f"status must be one of {sorted(allowed)}")
        cur = self.conn.execute(
            "UPDATE devices SET status = ? WHERE code = ?",
            (status, code),
        )
        if cur.rowcount == 0:
            raise ApiError(404, "device not found")
        self.conn.commit()
        row = self.conn.execute(
            "SELECT * FROM devices WHERE code = ?",
            (code,),
        ).fetchone()
        if status == "idle" and self._orders is not None:
            import threading

            threading.Thread(target=self._orders.try_dispatch, daemon=True).start()
        return dict(row)

    def save_telemetry(self, payload: dict[str, Any]) -> dict[str, Any]:
        device_code = payload.get("deviceCode") or payload.get("device_code")
        if not device_code:
            raise ApiError(400, "deviceCode required")
        entry = {
            **payload,
            "deviceCode": device_code,
            "receivedAt": now_iso(),
        }
        self._telemetry[device_code] = entry
        return {"ok": True, "deviceCode": device_code, "receivedAt": entry["receivedAt"]}

    def get_telemetry(self, device_code: str) -> dict[str, Any] | None:
        return self._telemetry.get(device_code)

    def list_telemetry(self) -> dict[str, Any]:
        return self._telemetry

    @staticmethod
    def _map_mission(row: dict[str, Any]) -> dict[str, Any]:
        return {
            "id": row["id"],
            "orderId": row["order_id"],
            "deviceId": row["device_id"],
            "deviceCode": row.get("device_code"),
            "status": row["status"],
            "createdAt": row["created_at"],
            "currentWaypoint": row.get("current_waypoint"),
            "currentWaypointLabel": row.get("current_waypoint_label"),
        }
