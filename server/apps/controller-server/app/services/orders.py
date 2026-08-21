from __future__ import annotations

import math
import os
import threading
import time
from typing import Any

from ..adapters import (
    OmxHttpStationAdapter,
    ORDER_FLOW,
    MockAiAdapter,
    MockCartAdapter,
    MockStationAdapter,
    PinkyHttpCartAdapter,
    parse_omx_url,
    parse_pinky_robot_urls,
)
from ..constants import PRODUCT_MAX_STOCK
from ..db import now_iso
from ..errors import ApiError
from ..waypoints import (
    SLUG_TO_WAYPOINT,
    WAYPOINTS,
    aruco_marker_id_for_waypoint,
    aruco_standoff_for_waypoint,
    conflict_aware_tour_order,
    get_waypoint,
    home_for_device,
    shelf_undock_after_aruco,
    shelf_undock_distance_m,
    shelf_undock_odom_travel_m,
    staging_waypoint_id,
    waypoint_ids_for_slugs,
)
from .carts import CartsService
from .traffic import (
    NavGoal,
    TrafficCoordinator,
    TrafficEmergencyWaitError,
    TrafficTimeoutError,
    TrafficYieldError,
)

_dispatch_lock = threading.Lock()


class MissionYielded(Exception):
    """Mission re-queued so another order can use this cart; not a hard failure."""

    pass

ACTIVE_STATUSES = ("ASSIGNED", "PICKING", "CHECKOUT", "PACKING", "RETURNING")
# 대기장소(S1/S2) 복귀 시 강제 헤딩 — map +x (오른쪽)
HOME_YAW = 0.0


def _wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _short_error(exc: BaseException, limit: int = 180) -> str:
    text = " ".join(str(exc).split())
    low = text.lower()
    if "<html" in low or "<!doctype" in low:
        return "pinky HTTP 500 (check pinky_bridge logs)"
    return text[:limit]


def _nav_error_needs_retreat(detail: str) -> bool:
    """Nav2가 시작점 점유/진행불가일 때만 후진 회복.

    로컬라이즈/TF 실패는 후진하면 벽을 향해 밀려 가므로 제외한다.
    """
    text = (detail or "").lower()
    if any(s in text for s in ("interrupted", "canceled", "cancelled")):
        return False
    # 대기장소/AMCL 시드 실패 — cmd_vel 후진 금지
    if any(
        s in text
        for s in (
            "no_tf",
            "no tf",
            "home seed",
            "localization",
            "amcl seed",
            "map→base",
            "map->base",
            "after amcl",
        )
    ):
        return False
    if "error_code=102" in text or "error_code=105" in text or "error_code=108" in text:
        return True
    if "aborted" in text or "no_valid_path" in text:
        return True
    return False


class OrdersService:
    def __init__(self, conn, carts: CartsService):
        self.conn = conn
        self.carts = carts
        urls = parse_pinky_robot_urls()
        if urls:
            self.cart_port = PinkyHttpCartAdapter(urls)
        else:
            self.cart_port = MockCartAdapter()
        self.traffic = TrafficCoordinator(
            self.cart_port, robot_codes=list(urls.keys()) if urls else []
        )
        omx_url = parse_omx_url()
        if omx_url and os.environ.get("ADAPTER_MODE", "").strip().lower() != "mock":
            self.station_port = OmxHttpStationAdapter(omx_url)
        else:
            self.station_port = MockStationAdapter()
        self.ai_port = MockAiAdapter()
        self._dwell_sec = float(os.environ.get("PICK_DWELL_SEC", "0"))
        self._nav_timeout = float(os.environ.get("PICK_NAV_TIMEOUT_SEC", "180"))
        self._mission_timeout = float(os.environ.get("TRAFFIC_MISSION_TIMEOUT", "300"))
        self._aborted_missions: set[int] = set()
        self._abort_lock = threading.Lock()
        self._returning_home: set[str] = set()
        self._tour_locks: dict[str, threading.Lock] = {}
        # Single physical OMX arm — serialize picks across carts.
        self._omx_arm_lock = threading.Lock()
        self._omx_busy_retries = max(
            1, int(os.environ.get("OMX_BUSY_RETRIES", "8"))
        )
        self._omx_busy_retry_sec = float(os.environ.get("OMX_BUSY_RETRY_SEC", "1.0"))

    def _device_tour_lock(self, device_code: str) -> threading.Lock:
        code = (device_code or "").strip()
        with self._abort_lock:
            lock = self._tour_locks.get(code)
            if lock is None:
                lock = threading.Lock()
                self._tour_locks[code] = lock
            return lock

    def _mark_aborted(self, mission_id: int) -> None:
        with self._abort_lock:
            self._aborted_missions.add(mission_id)

    def _clear_aborted(self, mission_id: int) -> None:
        with self._abort_lock:
            self._aborted_missions.discard(mission_id)

    def _is_aborted(self, mission_id: int) -> bool:
        with self._abort_lock:
            return mission_id in self._aborted_missions

    def _ensure_not_aborted(self, mission_id: int) -> None:
        if self._is_aborted(mission_id):
            raise RuntimeError("aborted by operator (stop)")

    def _active_mission_for_device(self, device_code: str):
        return self.conn.execute(
            """
            SELECT m.id, m.order_id, m.status, d.id AS device_id, d.code
            FROM missions m
            JOIN devices d ON d.id = m.device_id
            WHERE d.code = ?
              AND m.status IN (
                'ASSIGNED', 'PICKING', 'CHECKOUT', 'PACKING', 'RETURNING'
              )
            ORDER BY m.id DESC
            LIMIT 1
            """,
            (device_code,),
        ).fetchone()

    def abort_device(self, device_code: str, *, dispatch: bool = True) -> dict[str, Any]:
        """
        주행 정지: Nav2 stop + 활성 미션 FAILED.
        그 자리에 멈춤 (홈 복귀 없음).
        """
        code = (device_code or "").strip()
        if not code:
            raise ApiError(400, "deviceCode required")

        mission = self._active_mission_for_device(code)
        try:
            self.cart_port.stop_nav(code)
        except Exception:
            pass
        try:
            self.traffic.interrupt_robot(code)
        except Exception:
            pass

        aborted_mission = None
        if mission:
            mid = int(mission["id"])
            oid = int(mission["order_id"])
            self._mark_aborted(mid)
            # 투어 스레드가 홈 복귀하지 않도록 즉시 FAILED
            if mission["status"] != "FAILED":
                self._set_waypoint(mid, None)
                self._set_status(
                    oid,
                    mid,
                    "FAILED",
                    note="failed:operator stop — stayed in place",
                )
            self._release_device(mid)
            aborted_mission = {
                "id": mid,
                "orderId": oid,
                "status": "FAILED",
            }

        if dispatch:
            threading.Thread(target=self.try_dispatch, daemon=True).start()
        return {
            "ok": True,
            "deviceCode": code,
            "stopped": True,
            "mission": aborted_mission,
        }

    def return_home_device(self, device_code: str) -> dict[str, Any]:
        """
        활성 작업이 있으면 FAILED 처리 후, 해당 카트 대기장소(S1/S2)로 복귀.
        """
        code = (device_code or "").strip()
        if not code:
            raise ApiError(400, "deviceCode required")

        abort_info = self.abort_device(code, dispatch=False)
        home = home_for_device(code)

        with self._abort_lock:
            already = code in self._returning_home
            self._returning_home.add(code)

        # 복귀 중 busy 유지 (다른 주문 할당 방지)
        self.conn.execute(
            "UPDATE devices SET status = 'busy' WHERE code = ?",
            (code,),
        )
        self.conn.commit()

        def _go() -> None:
            try:
                try:
                    self.cart_port.stop_nav(code)
                    time.sleep(0.4)
                except Exception:
                    pass
                try:
                    self.traffic.acquire_return_home(code, timeout_sec=5.0)
                except TypeError:
                    self.traffic.acquire_return_home(code)
                except Exception:
                    pass
                try:
                    attempts = max(1, int(os.environ.get("PICK_NAV_RETRIES", "3")))
                    arrived = False
                    errors: list[str] = []
                    for attempt in range(1, attempts + 1):
                        result = self.cart_port.navigate_pose(
                            code,
                            home.x,
                            home.y,
                            HOME_YAW,
                            timeout_sec=self._nav_timeout,
                            require_yaw=False,
                        )
                        if result == "ARRIVED":
                            arrived = True
                            break
                        detail = (
                            getattr(self.cart_port, "last_nav_error", None)
                            or "nav failed"
                        )
                        errors.append(f"try{attempt}:{detail}")
                        if attempt < attempts:
                            time.sleep(1.0)
                    if not arrived:
                        print(
                            f"[return-home] {code} failed: "
                            + " | ".join(errors),
                            flush=True,
                        )
                finally:
                    self.traffic.release_return_home(code)
            finally:
                with self._abort_lock:
                    self._returning_home.discard(code)
                self.conn.execute(
                    "UPDATE devices SET status = 'idle' WHERE code = ?",
                    (code,),
                )
                self.conn.commit()
                threading.Thread(target=self.try_dispatch, daemon=True).start()

        threading.Thread(
            target=_go, name=f"return-home-{code}", daemon=True
        ).start()
        return {
            "ok": True,
            "deviceCode": code,
            "home": {"id": home.id, "x": home.x, "y": home.y, "yaw": home.yaw},
            "returning": True,
            "alreadyReturning": already,
            "mission": abort_info.get("mission"),
        }

    def sync_device_home_poses(self, *, only_idle: bool = True) -> None:
        """홈 initialpose 자동 동기화는 하지 않는다.

        과거: idle/pose 공백 때 S1/S2 를 넣어 작업 중 멈칫·dwell 레이스로
        대기장소 점프가 반복됨. 홈 시드는 로봇 pinky 부트 루프만 담당.
        """
        return

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
            for item in cart["items"]:
                qty = int(item["quantity"])
                if qty < 1:
                    raise ApiError(400, "quantity must be >= 1")
                if qty > PRODUCT_MAX_STOCK:
                    raise ApiError(
                        400,
                        f"quantity exceeds max stock ({PRODUCT_MAX_STOCK})",
                    )
                row = db.execute(
                    "SELECT id, name, stock FROM products WHERE id = ? AND is_active = 1",
                    (item["productId"],),
                ).fetchone()
                if not row:
                    raise ApiError(400, f"product {item['productId']} not available")
                if int(row["stock"]) < qty:
                    raise ApiError(
                        400,
                        f"insufficient stock for {row['name']} "
                        f"(have {row['stock']}, need {qty})",
                    )

            cur = db.execute(
                """
                INSERT INTO orders (user_id, status, total_price, created_at, updated_at)
                VALUES (?, 'CREATED', ?, ?, ?)
                """,
                (user_id, total, ts, ts),
            )
            order_id = cur.lastrowid

            for item in cart["items"]:
                qty = int(item["quantity"])
                updated = db.execute(
                    """
                    UPDATE products
                    SET stock = stock - ?, updated_at = ?
                    WHERE id = ? AND stock >= ?
                    """,
                    (qty, ts, item["productId"], qty),
                )
                if updated.rowcount == 0:
                    raise ApiError(
                        400,
                        f"insufficient stock for product {item['productId']}",
                    )
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
                        qty,
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
        또한 ASSIGNED 상태로 너무 오래 멈춘 미션은 FAILED 처리
        (피킹 스레드가 죽거나 DB 락에 걸린 경우).
        """
        fixed = 0
        # Stuck ASSIGNED with no progress (no PICKING+)
        stuck_sec = float(os.environ.get("PICK_ASSIGNED_STUCK_SEC", "90"))
        stuck = self.conn.execute(
            """
            SELECT m.id, m.order_id, m.device_id, m.created_at
            FROM missions m
            WHERE m.status = 'ASSIGNED'
            """
        ).fetchall()
        from datetime import datetime, timezone

        now = datetime.now(timezone.utc)
        for m in stuck:
            # Prefer mission event time for ASSIGNED if available
            ev = self.conn.execute(
                """
                SELECT created_at FROM mission_events
                WHERE mission_id = ? AND to_status = 'ASSIGNED'
                ORDER BY id DESC LIMIT 1
                """,
                (m["id"],),
            ).fetchone()
            ts_raw = (ev["created_at"] if ev else m["created_at"]) or ""
            try:
                assigned_at = datetime.fromisoformat(ts_raw.replace("Z", "+00:00"))
            except Exception:
                continue
            age = (now - assigned_at).total_seconds()
            if age < stuck_sec:
                continue
            oid = int(m["order_id"])
            mid = int(m["id"])
            self._set_status(
                oid,
                mid,
                "FAILED",
                note=f"failed:stuck in ASSIGNED >{stuck_sec:.0f}s (robot offline or tour thread dead)",
            )
            self._set_waypoint(mid, None)
            if m["device_id"]:
                self.conn.execute(
                    "UPDATE devices SET status = 'idle' WHERE id = ?",
                    (m["device_id"],),
                )
                self.conn.commit()
            fixed += 1

        rows = self.conn.execute(
            """
            SELECT d.id, d.code, d.status
            FROM devices d
            WHERE d.type = 'cart' AND d.status IN ('busy', 'error')
            """
        ).fetchall()
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
        """Assign FIFO QUEUED/CREATED missions to idle cart devices."""
        with _dispatch_lock:
            self.reclaim_stale_carts()
            while True:
                mission = self.conn.execute(
                    """
                    SELECT m.id, m.order_id
                    FROM missions m
                    WHERE m.status IN ('QUEUED', 'CREATED') AND m.device_id IS NULL
                    ORDER BY COALESCE(m.yield_count, 0) ASC, m.id ASC
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
                    with self._abort_lock:
                        if code in self._returning_home:
                            continue
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
                # FIFO home-depart uses this timestamp — set before tour thread races.
                try:
                    self.traffic.register_mission(device_code, mission_id)
                except Exception:
                    pass

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
        *,
        require_yaw: bool = False,
        mission_id: int | None = None,
        allow_retreat: bool = True,
        skip_traffic_wait: bool = False,
    ) -> None:
        # Nav2 aborts/replans mid-leg often; retry before failing the whole tour.
        attempts = max(1, int(os.environ.get("PICK_NAV_RETRIES", "3")))
        yaw_tol = float(os.environ.get("PICK_HOME_YAW_TOL_RAD", "0.12"))
        errors: list[str] = []
        leg_goal = NavGoal(float(x), float(y), float(yaw))
        if mission_id is not None:
            self._ensure_not_aborted(mission_id)
            # Emergency (비상대기→W7) may interrupt acquire; retry after clear.
            while True:
                try:
                    self.traffic.acquire_nav_leg(
                        device_code,
                        leg_goal,
                        mission_id,
                        waypoint_id,
                        skip_traffic_wait=skip_traffic_wait,
                    )
                    break
                except TrafficEmergencyWaitError:
                    self._emergency_wait_on_peer_path(
                        device_code,
                        mission_id,
                        waypoint_id=waypoint_id,
                    )
                    self.traffic.clear_stale_emergency_wait(device_code)
                    self._ensure_not_aborted(mission_id)
                except TrafficTimeoutError:
                    self._ensure_not_aborted(mission_id)
                    raise
        try:
            for attempt in range(1, attempts + 1):
                if mission_id is not None:
                    self._ensure_not_aborted(mission_id)
                # Stale emergency_wait flag must not block after paths diverge.
                self.traffic.clear_stale_emergency_wait(device_code)
                # Only hold when *currently* on a peer's remaining NAV path.
                if self.traffic.is_on_peer_active_path(device_code):
                    self._emergency_wait_on_peer_path(
                        device_code,
                        mission_id,
                        waypoint_id=waypoint_id,
                    )
                    self.traffic.clear_stale_emergency_wait(device_code)
                    if self.traffic.is_on_peer_active_path(device_code):
                        errors.append(f"try{attempt}:emergency_hold")
                        if attempt < attempts:
                            continue
                        # Last attempt: do not hard-fail — release traffic and
                        # let outer tour handle timeout; try navigate anyway.
                        if mission_id is not None:
                            self._mission_note(
                                mission_id,
                                f"emergency hold exhausted at {waypoint_id}; "
                                "clearing flag and retrying nav",
                            )
                        self.traffic.clear_stale_emergency_wait(device_code)
                result = self.cart_port.navigate_pose(
                    device_code,
                    x,
                    y,
                    yaw,
                    timeout_sec=self._nav_timeout,
                    require_yaw=require_yaw,
                    yaw_tol_rad=yaw_tol,
                )
                if result == "ARRIVED":
                    if mission_id is not None:
                        self._ensure_not_aborted(mission_id)
                    return
                detail = getattr(self.cart_port, "last_nav_error", None) or "unknown"
                errors.append(f"try{attempt}:{detail}")
                if mission_id is not None:
                    self._mission_note(
                        mission_id,
                        f"nav retry {waypoint_id} {attempt}/{attempts}: {detail}",
                    )
                if attempt < attempts:
                    self.traffic.clear_stale_emergency_wait(device_code)
                    # Peer path only: hold still. Never use stale emergency flag.
                    if self.traffic.is_on_peer_active_path(device_code):
                        self._emergency_wait_on_peer_path(
                            device_code,
                            mission_id,
                            waypoint_id=waypoint_id,
                        )
                        self.traffic.clear_stale_emergency_wait(device_code)
                    elif allow_retreat and _nav_error_needs_retreat(detail):
                        self._retreat_from_obstacle(
                            device_code,
                            mission_id,
                            waypoint_id=waypoint_id,
                        )
                    time.sleep(0.4)
                    continue
                raise RuntimeError(
                    f"nav failed at {waypoint_id} after {attempts} tries: "
                    + " | ".join(errors)
                )
        finally:
            if mission_id is not None:
                self.traffic.release_nav_leg(device_code)

    def _near_device_home(self, device_code: str) -> bool:
        """True when pose is still at this cart's S1/S2 wait spot."""
        home = home_for_device(device_code)
        near_m = float(os.environ.get("PICK_HOME_POSE_NEAR_M", "0.4"))
        pose = self.cart_port.get_pose(device_code)
        if not pose or pose.get("x") is None or pose.get("y") is None:
            return False
        dx = float(pose["x"]) - home.x
        dy = float(pose["y"]) - home.y
        return (dx * dx + dy * dy) ** 0.5 < near_m

    def _near_waypoint(
        self, device_code: str, waypoint_id: str, near_m: float | None = None
    ) -> bool:
        wp = get_waypoint(waypoint_id)
        radius = near_m if near_m is not None else float(
            os.environ.get("TRAFFIC_STAGING_NEAR_M", "0.40")
        )
        pose = self.cart_port.get_pose(device_code)
        if not pose or pose.get("x") is None or pose.get("y") is None:
            return False
        dx = float(pose["x"]) - wp.x
        dy = float(pose["y"]) - wp.y
        return (dx * dx + dy * dy) ** 0.5 < radius

    def _ensure_waypoint_access(
        self,
        device_code: str,
        waypoint_id: str,
        mission_id: int,
    ) -> None:
        """Wait until target shelf zone is free.

        Still at S1/S2: stay put (no W7 hop — avoids spin/backup at home).
        Emergency (on peer path): 비상대기 at W7 only — never stack with 충돌/매대 대기.
        Already out on the floor (zone occupied): stage at W7 as 매대 대기.
        Soft timeout → TrafficYieldError so peer orders / other shelves proceed.
        """
        if self.traffic.waypoint_access_granted(device_code, waypoint_id):
            return
        staging_id = staging_waypoint_id()
        staging = get_waypoint(staging_id)
        home = home_for_device(device_code)
        poll_sec = float(os.environ.get("TRAFFIC_STAGING_POLL_SEC", "2.0"))
        note_interval = max(5.0, poll_sec * 3)
        last_note = 0.0
        zone_yield = float(
            os.environ.get(
                "TRAFFIC_ZONE_YIELD_SEC",
                os.environ.get("TRAFFIC_YIELD_SEC", "60"),
            )
        )
        deadline = time.monotonic() + max(5.0, zone_yield)
        while not self.traffic.waypoint_access_granted(device_code, waypoint_id):
            self._ensure_not_aborted(mission_id)
            if time.monotonic() >= deadline:
                raise TrafficYieldError(
                    f"zone yield for {device_code} at {waypoint_id} "
                    f"after {zone_yield:.0f}s"
                )
            # 비상대기 우선: 피어 경로 위면 W7 비상대기만 (충돌/매대 대기와 동시 X).
            if self.traffic.is_on_peer_active_path(device_code):
                self._emergency_wait_on_peer_path(
                    device_code,
                    mission_id,
                    waypoint_id=waypoint_id,
                )
                self.traffic.clear_stale_emergency_wait(device_code)
                continue
            now = time.monotonic()
            if self._near_device_home(device_code):
                self._set_waypoint(
                    mission_id,
                    home.id,
                    label_suffix="매대 대기",
                )
                if now - last_note >= note_interval:
                    self._mission_note(
                        mission_id,
                        f"zone wait at home {home.id} for {waypoint_id} "
                        f"(peer occupies or P return)",
                    )
                    last_note = now
            else:
                self._set_waypoint(
                    mission_id,
                    staging_id,
                    label_suffix="매대 대기",
                )
                if not self._near_waypoint(device_code, staging_id):
                    if now - last_note >= note_interval:
                        self._mission_note(
                            mission_id,
                            f"nav staging {staging_id} for zone {waypoint_id}",
                        )
                        last_note = now
                    self._nav_or_fail(
                        device_code,
                        staging.x,
                        staging.y,
                        staging.yaw,
                        staging_id,
                        require_yaw=False,
                        mission_id=mission_id,
                        allow_retreat=not self._recovery_suppressed(device_code),
                        skip_traffic_wait=True,
                    )
                elif now - last_note >= note_interval:
                    self._mission_note(
                        mission_id,
                        f"staging at {staging_id} for zone {waypoint_id}",
                    )
                    last_note = now
            try:
                self.traffic.acquire_waypoint_access(
                    device_code,
                    waypoint_id,
                    timeout_sec=min(poll_sec, max(0.5, deadline - time.monotonic())),
                )
            except TrafficTimeoutError:
                pass
            time.sleep(max(0.2, poll_sec))

    def _acquire_waypoint_zone(
        self,
        device_code: str,
        waypoint_id: str,
        mission_id: int,
    ) -> None:
        """Wait for access, atomically claim zone, then nav/dock may proceed."""
        while True:
            self._ensure_not_aborted(mission_id)
            if not self.traffic.waypoint_access_granted(device_code, waypoint_id):
                self._ensure_waypoint_access(device_code, waypoint_id, mission_id)
                continue
            if self.traffic.try_claim_waypoint_zone(device_code, waypoint_id):
                return
            time.sleep(max(0.2, float(os.environ.get("TRAFFIC_STAGING_POLL_SEC", "2.0"))))

    def _aruco_dock_or_fail(
        self,
        device_code: str,
        waypoint_id: str,
        mission_id: int | None = None,
    ) -> tuple[float, float | None]:
        marker_id = aruco_marker_id_for_waypoint(waypoint_id)
        if marker_id is None:
            return 0.0, None
        standoff = aruco_standoff_for_waypoint(waypoint_id)
        timeout = float(os.environ.get("ARUCO_DOCK_TIMEOUT_SEC", "60"))
        attempts = max(1, int(os.environ.get("PICK_ARUCO_RETRIES", "2")))
        last_detail = "unknown"
        last_travel = 0.0

        phase_labels = {
            "SEARCH": "마커 탐색 중",
            "LOST": "마커 재탐색 중",
            "FACE": "정면·자세 정렬 중",
            "SHIFT": "횡방향 위치 조정 중",
            "APPROACH": "접근·파킹 중",
            "US_APPROACH": "초음파 접근 중",
            "ARRIVED": "도킹 완료",
            "TIMEOUT": "도킹 타임아웃",
            "NO_MARKER": "마커 미검출",
        }

        for attempt in range(1, attempts + 1):
            if mission_id is not None:
                self._ensure_not_aborted(mission_id)
            dock = getattr(self.cart_port, "aruco_dock", None)
            if not callable(dock):
                return 0.0, None

            if mission_id is not None:
                self._set_waypoint(
                    mission_id,
                    waypoint_id,
                    label_suffix="마커 탐색 중",
                )
                self._mission_note(
                    mission_id,
                    f"aruco dock start {waypoint_id} marker={marker_id} try={attempt}",
                )

            stop_poll = threading.Event()
            last_label = ["마커 탐색 중"]

            def _poll_aruco_status() -> None:
                getter = getattr(self.cart_port, "get_nav_state", None)
                while not stop_poll.wait(0.45):
                    if mission_id is None or not callable(getter):
                        continue
                    try:
                        state = getter(device_code) or {}
                        dock_st = state.get("arucoDock") or {}
                        phase = dock_st.get("phase")
                        label = (
                            dock_st.get("phaseLabel")
                            or phase_labels.get(str(phase or ""), None)
                        )
                        if not label:
                            continue
                        if label == last_label[0]:
                            continue
                        last_label[0] = label
                        self._set_waypoint(
                            mission_id,
                            waypoint_id,
                            label_suffix=label,
                        )
                        self._mission_note(
                            mission_id,
                            f"aruco {waypoint_id}: {label}",
                        )
                    except Exception:
                        continue

            poller = threading.Thread(target=_poll_aruco_status, daemon=True)
            poller.start()
            try:
                result = dock(
                    device_code,
                    marker_id,
                    standoff_m=standoff,
                    timeout_sec=timeout,
                )
            finally:
                stop_poll.set()
                poller.join(timeout=1.5)

            if result.get("success") or result.get("status") == "ARRIVED":
                dist = result.get("distanceM")
                approach_travel = result.get("approachTravelM")
                note = f"aruco dock {waypoint_id} ok"
                final_m: float | None = None
                if dist is not None:
                    try:
                        final_m = float(dist)
                        note += f" distance={final_m:.3f}"
                    except (TypeError, ValueError):
                        pass
                if approach_travel is not None:
                    try:
                        note += f" approach={float(approach_travel):.3f}m"
                    except (TypeError, ValueError):
                        pass
                if mission_id is not None:
                    self._set_waypoint(
                        mission_id,
                        waypoint_id,
                        label_suffix="도킹 완료",
                    )
                    self._mission_note(mission_id, note)
                try:
                    travel = (
                        float(approach_travel)
                        if approach_travel is not None
                        else 0.0
                    )
                except (TypeError, ValueError):
                    travel = 0.0
                return max(0.0, travel), final_m
            last_detail = (
                getattr(self.cart_port, "last_aruco_error", None)
                or result.get("message")
                or result.get("status")
                or "unknown"
            )
            for key in ("approachTravelM", "distanceM"):
                raw = result.get(key)
                if raw is None:
                    continue
                try:
                    last_travel = max(last_travel, float(raw))
                except (TypeError, ValueError):
                    pass
            if mission_id is not None:
                self._set_waypoint(
                    mission_id,
                    waypoint_id,
                    label_suffix=f"도킹 재시도 {attempt}/{attempts}",
                )
            if attempt < attempts:
                time.sleep(0.5)
                continue
            self._retreat_from_obstacle(
                device_code,
                mission_id,
                waypoint_id=waypoint_id,
                travel_m=last_travel,
            )
            raise RuntimeError(
                f"aruco dock failed at {waypoint_id} after {attempts} tries: {last_detail}"
            )
        return 0.0, None

    def _undock_after_shelf_aruco(
        self,
        device_code: str,
        waypoint_id: str,
        approach_travel_m: float,
        mission_id: int | None = None,
        *,
        final_range_m: float | None = None,
    ) -> None:
        """도킹 대기 후 후진. 측정 range가 접근 직전 값에 도달하거나 odom 예산 소진 시 정지."""
        if not shelf_undock_after_aruco(waypoint_id):
            return
        target = shelf_undock_distance_m(waypoint_id, approach_travel_m)
        min_m = float(os.environ.get("PICK_SHELF_UNDOCK_MIN_M", "0.005"))
        if target < min_m:
            return
        speed = float(os.environ.get("PICK_SHELF_UNDOCK_SPEED_MPS", "0.02"))
        odom_budget = shelf_undock_odom_travel_m(
            waypoint_id,
            approach_travel_m,
            final_range_m=final_range_m,
        )
        max_travel = float(os.environ.get("PICK_SHELF_UNDOCK_MAX_M", "0.80"))
        odom_budget = max(min_m, min(odom_budget, max_travel))
        marker_id = aruco_marker_id_for_waypoint(waypoint_id)
        timeout = float(os.environ.get("PINKY_ARUCO_UNDOCK_TIMEOUT_SEC", "30"))

        if mission_id is not None:
            self._set_waypoint(
                mission_id,
                waypoint_id,
                label_suffix="후진 중",
            )
            self._mission_note(
                mission_id,
                f"undock {waypoint_id} until range={target:.3f}m "
                f"odomCap={odom_budget:.3f}m",
            )

        undock_fn = getattr(self.cart_port, "aruco_undock", None)
        if callable(undock_fn) and marker_id is not None:
            if mission_id is not None:
                self._ensure_not_aborted(mission_id)
            result = undock_fn(
                device_code,
                marker_id,
                target_range_m=target,
                timeout_sec=timeout,
                speed_mps=speed,
                max_travel_m=odom_budget,
            )
            if mission_id is not None:
                self._mission_note(
                    mission_id,
                    f"undock {waypoint_id} "
                    f"ok={bool(result.get('success'))} "
                    f"range={result.get('distanceM')} "
                    f"moved={result.get('movedM')} "
                    f"cap={result.get('maxTravelM')} "
                    f"{result.get('message') or ''}".strip(),
                )
                suffix = "후진 완료" if result.get("success") else "후진 부분"
                self._set_waypoint(mission_id, waypoint_id, label_suffix=suffix)
            if result.get("success"):
                return

        rel = getattr(self.cart_port, "relative_move", None)
        if not callable(rel):
            return
        if mission_id is not None:
            self._mission_note(
                mission_id,
                f"undock {waypoint_id} fallback odom back {odom_budget:.3f}m",
            )
        step_max = float(os.environ.get("PICK_SHELF_UNDOCK_STEP_M", "0.20"))
        remaining = odom_budget
        moved_sum = 0.0
        max_steps = max(4, int(math.ceil(odom_budget / max(step_max, 0.05))) + 3)
        steps = 0
        while remaining > min_m and steps < max_steps:
            steps += 1
            if mission_id is not None:
                self._ensure_not_aborted(mission_id)
            if self._recovery_suppressed(device_code):
                if mission_id is not None:
                    self._mission_note(
                        mission_id,
                        f"undock {waypoint_id} aborted: emergency wait (no reverse)",
                    )
                break
            step = min(remaining, step_max)
            step_timeout = max(2.5, step / max(speed, 0.01) + 2.0)
            result = rel(
                device_code,
                -step,
                speed_mps=speed,
                timeout_sec=step_timeout,
                bypass_collision=True,
                ignore_scan=True,
            )
            try:
                moved = abs(
                    float(
                        result.get(
                            "movedM",
                            step if result.get("success") else 0.0,
                        )
                    )
                )
            except (TypeError, ValueError):
                moved = step if result.get("success") else 0.0
            moved_sum += moved
            remaining = max(0.0, odom_budget - moved_sum)
            if not result.get("success") and moved < min_m:
                break
            if moved < min_m:
                break
        if mission_id is not None:
            self._mission_note(
                mission_id,
                f"undock {waypoint_id} fallback done moved={moved_sum:.3f}m",
            )
            self._set_waypoint(mission_id, waypoint_id, label_suffix="후진 완료")

    def _recovery_suppressed(self, device_code: str) -> bool:
        """True only while pose is on a peer's remaining NAVIGATING path.

        Do not use last_wait_reason — emergency_wait flags were sticky and froze
        carts after paths no longer overlapped, ending in mission failure.
        """
        return self.traffic.is_on_peer_active_path(device_code)

    def _emergency_wait_on_peer_path(
        self,
        device_code: str,
        mission_id: int | None,
        *,
        waypoint_id: str | None = None,
    ) -> None:
        """비상대기: 피어 주행 경로 위면 W7로 회피 후 대기. 충돌/매대 대기와 병행하지 않음."""
        poll_sec = float(os.environ.get("TRAFFIC_EMERGENCY_WAIT_SEC", "2.0"))
        max_sec = float(os.environ.get("TRAFFIC_EMERGENCY_WAIT_MAX_SEC", "45"))
        deadline = time.monotonic() + max(poll_sec, max_sec)
        wid = (waypoint_id or "").strip().upper() or "?"
        staging_id = staging_waypoint_id()
        staging = get_waypoint(staging_id)

        if mission_id is not None:
            self._mission_note(
                mission_id,
                f"emergency wait → {staging_id} (비상대기) for {wid}",
            )
            self._set_waypoint(mission_id, staging_id, label_suffix="비상대기")

        # Home pad: stay put (W7 hop from S1/S2 causes REJECT storms).
        if self._near_device_home(device_code):
            while time.monotonic() < deadline:
                if mission_id is not None:
                    try:
                        self._ensure_not_aborted(mission_id)
                    except Exception:
                        break
                try:
                    self.cart_port.stop_nav(device_code, freeze=False)
                except TypeError:
                    try:
                        self.cart_port.stop_nav(device_code)
                    except Exception:
                        pass
                except Exception:
                    pass
                if not self.traffic.is_on_peer_active_path(device_code):
                    break
                time.sleep(max(0.5, poll_sec))
            self.traffic.clear_stale_emergency_wait(device_code)
            return

        # Floor: move to W7 once, then hold until peer path clears.
        # Use navigate_pose directly — _nav_or_fail would re-enter emergency wait.
        if not self._near_waypoint(device_code, staging_id):
            try:
                self.cart_port.stop_nav(device_code, freeze=False)
            except TypeError:
                try:
                    self.cart_port.stop_nav(device_code)
                except Exception:
                    pass
            except Exception:
                pass
            try:
                result = self.cart_port.navigate_pose(
                    device_code,
                    staging.x,
                    staging.y,
                    staging.yaw,
                    timeout_sec=min(
                        60.0,
                        float(os.environ.get("TRAFFIC_EMERGENCY_W7_NAV_SEC", "45")),
                    ),
                    require_yaw=False,
                )
                if mission_id is not None and result != "ARRIVED":
                    detail = (
                        getattr(self.cart_port, "last_nav_error", None) or result
                    )
                    self._mission_note(
                        mission_id,
                        f"emergency W7 nav incomplete: {detail}; hold at best effort",
                    )
            except Exception as exc:
                if mission_id is not None:
                    self._mission_note(
                        mission_id,
                        f"emergency W7 nav failed: {_short_error(exc)}; hold still",
                    )

        while time.monotonic() < deadline:
            if mission_id is not None:
                try:
                    self._ensure_not_aborted(mission_id)
                except Exception:
                    break
                self._set_waypoint(mission_id, staging_id, label_suffix="비상대기")
            try:
                self.cart_port.stop_nav(device_code, freeze=False)
            except TypeError:
                try:
                    self.cart_port.stop_nav(device_code)
                except Exception:
                    pass
            except Exception:
                pass
            if not self.traffic.is_on_peer_active_path(device_code):
                break
            time.sleep(max(0.5, poll_sec))
        self.traffic.clear_stale_emergency_wait(device_code)

    def _retreat_from_obstacle(
        self,
        device_code: str,
        mission_id: int | None,
        *,
        waypoint_id: str | None = None,
        travel_m: float | None = None,
    ) -> None:
        """선반/벽에 박힌 채 Nav2 하면 error 102로 즉시 abort — cmd_vel 후진으로 빠져나온다."""
        if self._recovery_suppressed(device_code):
            self._emergency_wait_on_peer_path(
                device_code, mission_id, waypoint_id=waypoint_id
            )
            return
        wid = (waypoint_id or "").strip().upper()
        if wid == "C":
            return
        fallback = float(os.environ.get("PICK_ABORT_BACKUP_M", "0.30"))
        total = 0.0
        if wid and shelf_undock_after_aruco(wid):
            total = shelf_undock_odom_travel_m(wid, travel_m)
        if total < 0.05:
            total = fallback
        max_m = float(os.environ.get("PICK_SHELF_UNDOCK_MAX_M", "0.80"))
        total = min(max(total, 0.05), max_m)
        speed = float(os.environ.get("PICK_SHELF_UNDOCK_SPEED_MPS", "0.02"))
        step_max = float(os.environ.get("PICK_SHELF_UNDOCK_STEP_M", "0.20"))
        rel = getattr(self.cart_port, "relative_move", None)
        if not callable(rel):
            return
        if mission_id is not None:
            self._mission_note(
                mission_id,
                f"retreat {wid or 'abort'} back {total:.3f}m before nav",
            )
        remaining = total
        moved_sum = 0.0
        min_m = 0.005
        max_steps = max(4, int(math.ceil(total / max(step_max, 0.05))) + 3)
        steps = 0
        while remaining > min_m and steps < max_steps:
            steps += 1
            if mission_id is not None:
                self._ensure_not_aborted(mission_id)
            if self._recovery_suppressed(device_code):
                if mission_id is not None:
                    self._mission_note(
                        mission_id,
                        "retreat aborted: emergency wait (no reverse)",
                    )
                break
            step = min(remaining, step_max)
            timeout = max(2.5, step / max(speed, 0.01) + 2.0)
            result = rel(
                device_code,
                -step,
                speed_mps=speed,
                timeout_sec=timeout,
                bypass_collision=True,
                ignore_scan=True,
            )
            try:
                moved = abs(
                    float(
                        result.get(
                            "movedM",
                            step if result.get("success") else 0.0,
                        )
                    )
                )
            except (TypeError, ValueError):
                moved = step if result.get("success") else 0.0
            moved_sum += moved
            remaining = max(0.0, total - moved_sum)
            if not result.get("success") and moved < min_m:
                if mission_id is not None:
                    self._mission_note(
                        mission_id,
                        f"retreat step fail: {result.get('message') or result}",
                    )
                break
            if moved < min_m:
                if mission_id is not None:
                    self._mission_note(
                        mission_id,
                        f"retreat step zero: {result.get('message') or result}",
                    )
                break
        if mission_id is not None:
            self._mission_note(
                mission_id,
                f"retreat done moved={moved_sum:.3f}m",
            )

    def _dock_dwell_undock(
        self,
        device_code: str,
        waypoint_id: str,
        mission_id: int,
    ) -> None:
        """ArUco dock → (optional dwell) → undock. No OMX at C/P."""
        travel, final_range = self._aruco_dock_or_fail(
            device_code, waypoint_id, mission_id
        )
        # Dwell default 0 — do not insert a fixed 3s pause.
        self._dwell_at(device_code, mission_id, waypoint_id)
        self._undock_after_shelf_aruco(
            device_code,
            waypoint_id,
            travel,
            mission_id,
            final_range_m=final_range,
        )

    def _acquire_omx_arm(self, mission_id: int) -> None:
        """Wait for the single OMX arm; abortable while waiting."""
        waited = False
        while True:
            self._ensure_not_aborted(mission_id)
            if self._omx_arm_lock.acquire(timeout=0.5):
                if waited:
                    self._mission_note(mission_id, "omx arm acquired")
                return
            if not waited:
                self._mission_note(mission_id, "omx arm wait")
                waited = True

    def _omx_pick_at_shelf(
        self,
        device_code: str,
        order_id: int,
        mission_id: int,
        waypoint_id: str,
        picks: list[tuple[str, int]],
    ) -> None:
        """OMX pick then continue tour. No dwell pause.

        Mock / no OMX URL / server unreachable → treat as success and proceed.
        """
        if not picks:
            return
        if not isinstance(self.station_port, OmxHttpStationAdapter):
            self._mission_note(
                mission_id,
                f"omx skip at {waypoint_id} (mock/no OMX_URL) — success",
            )
            return
        # Quick probe: if OMX HTTP is down, skip pick as success (no 3s wait).
        if not self.station_port.is_server_reachable():
            self._mission_note(
                mission_id,
                f"omx unreachable at {waypoint_id} — success override, continue",
            )
            return

        timeout = float(os.environ.get("OMX_PICK_TIMEOUT_SEC", "90"))
        self._acquire_omx_arm(mission_id)
        try:
            for slug, qty in picks:
                self._ensure_not_aborted(mission_id)
                qty = max(1, min(int(qty), PRODUCT_MAX_STOCK))
                self._set_waypoint(
                    mission_id,
                    waypoint_id,
                    label_suffix=f"{slug} 0/{qty}",
                )
                self._mission_note(mission_id, f"omx pick start {slug} x{qty}")

                def _on_progress(done: int, total: int) -> None:
                    self._set_waypoint(
                        mission_id,
                        waypoint_id,
                        label_suffix=f"{slug} {done}/{total}",
                    )
                    self._mission_note(mission_id, f"omx pick {slug} {done}/{total}")

                result = "FAILED"
                detail = "unknown"
                done = 0
                for attempt in range(self._omx_busy_retries):
                    self._ensure_not_aborted(mission_id)
                    result = self.station_port.pick(
                        device_code=device_code,
                        slug=slug,
                        quantity=qty,
                        order_id=order_id,
                        timeout_sec=timeout,
                        should_abort=lambda: self._is_aborted(mission_id),
                        on_progress=_on_progress,
                        force_success_on_unreachable=True,
                    )
                    done = int(self.station_port.last_state.get("done") or 0)
                    detail = self.station_port.last_error or "unknown"
                    busy = (
                        result == "FAILED"
                        and (
                            "409" in detail
                            or "busy" in detail.lower()
                            or "rejected" in detail.lower()
                        )
                    )
                    if not busy or attempt + 1 >= self._omx_busy_retries:
                        break
                    self._mission_note(
                        mission_id,
                        f"omx busy retry {attempt + 1}/{self._omx_busy_retries} {slug}",
                    )
                    time.sleep(self._omx_busy_retry_sec)

                if result == "DONE":
                    if self.station_port.last_error:
                        self._mission_note(
                            mission_id,
                            f"omx-unreachable-override {slug} x{qty}: "
                            f"{self.station_port.last_error}",
                        )
                    else:
                        self._mission_note(
                            mission_id, f"omx pick done {slug} {done}/{qty}"
                        )
                    continue
                self._mission_note(
                    mission_id,
                    f"omx pick {result} {slug} {done}/{qty}: {detail}",
                )
                raise RuntimeError(f"OMX pick {result} at {waypoint_id}: {detail}")
        finally:
            self._omx_arm_lock.release()

    def _dock_pick_or_dwell_undock(
        self,
        device_code: str,
        waypoint_id: str,
        mission_id: int,
        order_id: int,
        shelf_picks: list[tuple[str, int]],
    ) -> None:
        travel, final_range = self._aruco_dock_or_fail(
            device_code, waypoint_id, mission_id
        )
        self._omx_pick_at_shelf(
            device_code,
            order_id,
            mission_id,
            waypoint_id,
            shelf_picks,
        )
        self._undock_after_shelf_aruco(
            device_code,
            waypoint_id,
            travel,
            mission_id,
            final_range_m=final_range,
        )

    def _dwell_at(
        self,
        device_code: str,
        mission_id: int,
        waypoint_id: str,
    ) -> None:
        """Optional pause after arrive. Default PICK_DWELL_SEC=0 (disabled).

        stop_nav 를 호출하지 않는다. goal_wait 로 이미 도착한 뒤 stop 하면
        AMCL idle-freeze 가 걸리고, 그 사이 pose 조회 실패 시 홈 initialpose
        가 들어와 계산대 이동 중 대기장소로 점프한다.
        """
        del device_code
        self._ensure_not_aborted(mission_id)
        dwell = max(0.0, float(self._dwell_sec))
        if dwell <= 0:
            return
        self._set_waypoint(
            mission_id,
            waypoint_id,
            label_suffix=f"대기 {dwell:g}초",
        )
        self._mission_note(mission_id, f"dwell start {waypoint_id} {dwell}s")
        end = time.time() + dwell
        while time.time() < end:
            self._ensure_not_aborted(mission_id)
            time.sleep(min(0.25, end - time.time()))
        self._mission_note(mission_id, f"dwell end {waypoint_id}")
        self._set_waypoint(mission_id, waypoint_id)

    def _leave_checkout_then_pack(self, device_code: str, mission_id: int) -> None:
        """계산대 스탠드오프에서 현재 자세로 짧게 전진한 뒤 운송대기(P)로 Nav2."""
        p = get_waypoint("P")
        fwd = float(os.environ.get("PICK_C_EXIT_FORWARD_M", "0.25"))
        rel = getattr(self.cart_port, "relative_move", None)
        if fwd >= 0.05 and callable(rel):
            if mission_id is not None:
                self._ensure_not_aborted(mission_id)
            self._set_waypoint(mission_id, "C", label_suffix="출차 전진")
            self._mission_note(mission_id, f"checkout exit forward {fwd:.3f}m")
            speed = float(os.environ.get("PICK_SHELF_UNDOCK_SPEED_MPS", "0.02"))
            timeout = max(2.5, fwd / max(speed, 0.01) + 2.0)
            result = rel(
                device_code,
                fwd,
                speed_mps=speed,
                timeout_sec=timeout,
                bypass_collision=True,
                ignore_scan=True,
            )
            self._mission_note(
                mission_id,
                f"checkout exit forward done success={bool(result.get('success'))} "
                f"moved={result.get('movedM')}",
            )
        self._set_waypoint(mission_id, "P")
        self._acquire_waypoint_zone(device_code, "P", mission_id)
        try:
            self._nav_or_fail(
                device_code,
                p.x,
                p.y,
                p.yaw,
                "P",
                require_yaw=True,
                mission_id=mission_id,
                allow_retreat=False,
            )
            self._dock_dwell_undock(device_code, "P", mission_id)
        finally:
            self.traffic.release_waypoint_zone(device_code)

    def _return_home(
        self,
        device_code: str,
        mission_id: int,
        *,
        stop_first: bool = False,
        best_effort: bool = False,
    ) -> bool:
        """Navigate to wait spot (S1/S2) using waypoint pose."""
        home = home_for_device(device_code)
        home_yaw = float(home.yaw)
        yaw_tol = float(os.environ.get("PICK_HOME_YAW_TOL_RAD", "0.12"))
        if stop_first:
            try:
                stop = self.cart_port.stop_nav
                try:
                    stop(device_code, freeze=False)
                except TypeError:
                    stop(device_code)
                time.sleep(0.3)
            except Exception:
                pass
        self._set_waypoint(mission_id, home.id)
        try:
            self.traffic.acquire_return_home(device_code)
            try:
                self._nav_or_fail(
                    device_code,
                    home.x,
                    home.y,
                    home_yaw,
                    home.id,
                    require_yaw=False,
                    mission_id=mission_id,
                    allow_retreat=False,
                )
                for align in range(1, 4):
                    self._ensure_not_aborted(mission_id)
                    pose = self.cart_port.get_pose(device_code)
                    if pose:
                        err = abs(
                            _wrap_angle(float(pose.get("yaw") or 0.0) - home_yaw)
                        )
                        if err <= yaw_tol:
                            break
                        self._mission_note(
                            mission_id,
                            f"home yaw align {home.id} err={err:.2f} try={align}",
                        )
                    self._nav_or_fail(
                        device_code,
                        home.x,
                        home.y,
                        home_yaw,
                        f"{home.id}-yaw",
                        require_yaw=True,
                        mission_id=mission_id,
                        allow_retreat=False,
                    )
            finally:
                self.traffic.release_return_home(device_code)
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

    def _requeue_mission_for_peer_priority(
        self,
        order_id: int,
        mission_id: int,
        device_code: str,
        reason: str,
    ) -> None:
        """Release cart and put this mission behind less-yielded orders."""
        try:
            self.cart_port.stop_nav(device_code, freeze=False)
        except TypeError:
            try:
                self.cart_port.stop_nav(device_code)
            except Exception:
                pass
        except Exception:
            pass
        try:
            self.traffic.release_waypoint_zone(device_code)
        except Exception:
            pass
        try:
            self.traffic.release_nav_leg(device_code)
        except Exception:
            pass
        self.traffic.interrupt_robot(device_code)

        row = self.conn.execute(
            "SELECT device_id, COALESCE(yield_count, 0) AS yc FROM missions WHERE id = ?",
            (mission_id,),
        ).fetchone()
        device_id = int(row["device_id"]) if row and row["device_id"] else None
        yc = int(row["yc"] if row else 0) + 1
        ts = now_iso()
        self.conn.execute(
            """
            UPDATE missions
            SET status = 'QUEUED',
                device_id = NULL,
                current_waypoint = NULL,
                current_waypoint_label = NULL,
                yield_count = ?
            WHERE id = ?
            """,
            (yc, mission_id),
        )
        self.conn.execute(
            "UPDATE orders SET status = 'QUEUED', updated_at = ? WHERE id = ?",
            (ts, order_id),
        )
        self.conn.execute(
            """
            INSERT INTO mission_events (mission_id, from_status, to_status, note, created_at)
            VALUES (?, 'PICKING', 'QUEUED', ?, ?)
            """,
            (
                mission_id,
                f"yield#{yc} for peer priority: {reason}"[:240],
                ts,
            ),
        )
        if device_id is not None:
            self.conn.execute(
                "UPDATE devices SET status = 'idle' WHERE id = ?",
                (device_id,),
            )
        self.conn.commit()
        self._mission_note(
            mission_id,
            f"requeued yield#{yc} ({device_code}): {reason}"[:240],
            status="QUEUED",
        )

    def _run_pick_tour(
        self, order_id: int, mission_id: int, device_code: str
    ) -> None:
        lock = self._device_tour_lock(device_code)
        lock.acquire()
        try:
            self._run_pick_tour_locked(order_id, mission_id, device_code)
        finally:
            lock.release()

    def _run_pick_tour_locked(
        self, order_id: int, mission_id: int, device_code: str
    ) -> None:
        home = home_for_device(device_code)
        try:
            self._ensure_not_aborted(mission_id)
            if not self.cart_port.is_reachable(device_code):
                raise RuntimeError(
                    f"pinky unreachable for {device_code} "
                    f"(PINKY_ROBOTS URL / robot run.py 확인)"
                )

            self.ai_port.request_pick_plan(order_id)
            self.cart_port.notify_assign(device_code, order_id)
            # register_mission may already have run at dispatch (FIFO timestamp).
            self.traffic.register_mission(device_code, mission_id)

            # 현재 pose 기준 투어 시작. 홈 initialpose 는 넣지 않음
            # (작업 중/재할당 시 대기장소 점프 방지 — pinky ensure 가 현재 pose 사용)
            near_m = float(os.environ.get("PICK_HOME_POSE_NEAR_M", "0.4"))
            pose = self.cart_port.get_pose(device_code)
            start = home
            if pose and pose.get("x") is not None and pose.get("y") is not None:
                dx = float(pose["x"]) - home.x
                dy = float(pose["y"]) - home.y
                dist = (dx * dx + dy * dy) ** 0.5
                if dist < near_m:
                    start = home
                else:
                    from ..waypoints import Waypoint

                    start = Waypoint(
                        id="START",
                        label="current",
                        x=float(pose["x"]),
                        y=float(pose["y"]),
                        yaw=float(pose.get("yaw") or HOME_YAW),
                    )

            rows = self.conn.execute(
                """
                SELECT oi.product_id, p.slug, oi.quantity
                FROM order_items oi
                JOIN products p ON p.id = oi.product_id
                WHERE oi.order_id = ?
                """,
                (order_id,),
            ).fetchall()
            slugs = [r["slug"] for r in rows if r["slug"]]
            shelf_ids = waypoint_ids_for_slugs(slugs)
            shelf_picks_by_waypoint: dict[str, list[tuple[str, int]]] = {}
            qty_by_slug: dict[str, int] = {}
            for row in rows:
                slug = row["slug"]
                if not slug:
                    continue
                qty_by_slug[slug] = qty_by_slug.get(slug, 0) + int(row["quantity"] or 0)
            for slug, qty in qty_by_slug.items():
                wid = SLUG_TO_WAYPOINT.get(slug)
                if not wid:
                    continue
                if wid not in shelf_picks_by_waypoint:
                    shelf_picks_by_waypoint[wid] = []
                shelf_picks_by_waypoint[wid].append((slug, max(1, int(qty))))
            defer_ids = self.traffic.conflicting_waypoints(device_code, shelf_ids)
            tour = conflict_aware_tour_order(start, shelf_ids, defer_ids)
            remaining = [wp.id for wp in tour] + ["C", "P"]
            self.traffic.update_remaining(device_code, remaining)

            self._set_status(order_id, mission_id, "PICKING", note="pick tour start")

            pending = list(tour)
            deferred_once: set[str] = set()
            while pending:
                self._ensure_not_aborted(mission_id)
                wp = pending.pop(0)
                self._set_waypoint(mission_id, wp.id)
                retain_zone_for_checkout = False
                try:
                    self._acquire_waypoint_zone(device_code, wp.id, mission_id)
                    self._nav_or_fail(
                        device_code,
                        wp.x,
                        wp.y,
                        wp.yaw,
                        wp.id,
                        require_yaw=True,
                        mission_id=mission_id,
                    )
                    self._dock_pick_or_dwell_undock(
                        device_code,
                        wp.id,
                        mission_id,
                        order_id,
                        shelf_picks_by_waypoint.get(wp.id, []),
                    )
                    # W6와 C는 동일 좌표 — 다음이 계산대면 구역 유지 후 즉시 계산대.
                    if wp.id == "W6" and not pending:
                        rem_after = (
                            remaining[1:]
                            if remaining and remaining[0] == "W6"
                            else [w for w in remaining if w != "W6"]
                        )
                        if rem_after and rem_after[0] == "C":
                            retain_zone_for_checkout = True
                            self._mission_note(
                                mission_id,
                                "W6 zone retained for immediate checkout (shared C)",
                            )
                except TrafficYieldError as yield_exc:
                    try:
                        self.traffic.release_nav_leg(device_code)
                    except Exception:
                        pass
                    reason = _short_error(yield_exc)
                    # First conflict: defer this shelf so peer path/order can proceed.
                    if wp.id not in deferred_once and pending:
                        deferred_once.add(wp.id)
                        pending.append(wp)
                        if remaining and remaining[0] == wp.id:
                            remaining = remaining[1:] + [wp.id]
                        elif wp.id in remaining:
                            remaining = [w for w in remaining if w != wp.id] + [
                                wp.id
                            ]
                        self.traffic.update_remaining(device_code, remaining)
                        self._mission_note(
                            mission_id,
                            f"path/zone yield at {wp.id}; defer shelf for peer: "
                            f"{reason}",
                        )
                        continue
                    self._requeue_mission_for_peer_priority(
                        order_id,
                        mission_id,
                        device_code,
                        reason,
                    )
                    raise MissionYielded(reason) from yield_exc
                finally:
                    if not retain_zone_for_checkout:
                        self.traffic.release_waypoint_zone(device_code)
                if remaining and remaining[0] == wp.id:
                    remaining = remaining[1:]
                else:
                    remaining = [w for w in remaining if w != wp.id]
                self.traffic.update_remaining(device_code, remaining)

            self._ensure_not_aborted(mission_id)
            self._set_status(order_id, mission_id, "CHECKOUT", note="checkout")
            c = get_waypoint("C")
            self._set_waypoint(mission_id, "C")
            try:
                # W6 점유 유지 중이면 try_claim이 C로 업그레이드(동일 구역).
                self._acquire_waypoint_zone(device_code, "C", mission_id)
                self._nav_or_fail(
                    device_code,
                    c.x,
                    c.y,
                    c.yaw,
                    "C",
                    require_yaw=True,
                    mission_id=mission_id,
                    allow_retreat=False,
                )
                self._dock_dwell_undock(device_code, "C", mission_id)
            except TrafficYieldError as yield_exc:
                try:
                    self.traffic.release_waypoint_zone(device_code)
                except Exception:
                    pass
                reason = _short_error(yield_exc)
                self._requeue_mission_for_peer_priority(
                    order_id,
                    mission_id,
                    device_code,
                    f"checkout:{reason}",
                )
                raise MissionYielded(reason) from yield_exc
            finally:
                self.traffic.release_waypoint_zone(device_code)
            remaining = [w for w in remaining if w != "C"]
            self.traffic.update_remaining(device_code, remaining)

            self._ensure_not_aborted(mission_id)
            self._set_status(order_id, mission_id, "PACKING", note="transport wait")
            self._leave_checkout_then_pack(device_code, mission_id)
            self.traffic.mark_returning_home(device_code)
            remaining = [w for w in remaining if w != "P"]
            self.traffic.update_remaining(device_code, remaining)

            # 대기장소 복귀 중 — P 접근 대기가 홈 도착 전에 풀리지 않도록
            # returning_home 을 acquire 전에 켠다 (acquire_return_home 이 재설정).
            self._ensure_not_aborted(mission_id)
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
        except MissionYielded:
            # Already requeued + device idle; do not mark FAILED or force home.
            pass
        except Exception as exc:
            if self._is_aborted(mission_id):
                # 운영자 정지: 이미 FAILED 처리됨 — 그 자리 유지, 홈 복귀 안 함
                pass
            else:
                # Don't block forever on home return when pinky is already down
                note = f"failed:{_short_error(exc)}"
                try:
                    if self.cart_port.is_reachable(device_code):
                        self._set_status(
                            order_id,
                            mission_id,
                            "RETURNING",
                            note=f"abort return home:{_short_error(exc)}",
                        )
                        home_ok = self._return_home(
                            device_code,
                            mission_id,
                            stop_first=True,
                            best_effort=True,
                        )
                        if home_ok:
                            note = f"{note}; arrived wait spot"
                        else:
                            note = f"{note}; home return failed"
                    else:
                        note = f"{note}; skip home (pinky unreachable)"
                except Exception as home_exc:
                    note = f"{note}; home exc:{_short_error(home_exc)}"
                self._set_waypoint(mission_id, None)
                self._set_status(order_id, mission_id, "FAILED", note=note)
                self._release_device(mission_id)
        finally:
            self.traffic.unregister_mission(device_code)
            self._clear_aborted(mission_id)
            threading.Thread(target=self.try_dispatch, daemon=True).start()

    def queue_length(self) -> int:
        row = self.conn.execute(
            """
            SELECT COUNT(*) AS c FROM missions
            WHERE status IN ('QUEUED', 'CREATED') AND device_id IS NULL
            """
        ).fetchone()
        return int(row["c"] if row else 0)

    def list_devices(self) -> list[dict[str, Any]]:
        rows = self.conn.execute("SELECT * FROM devices").fetchall()
        return [dict(r) for r in rows]
