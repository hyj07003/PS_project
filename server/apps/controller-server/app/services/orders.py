from __future__ import annotations

import math
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
    aruco_marker_id_for_waypoint,
    get_waypoint,
    home_for_device,
    nearest_neighbor_order,
    waypoint_ids_for_slugs,
)
from .carts import CartsService

_dispatch_lock = threading.Lock()

ACTIVE_STATUSES = ("ASSIGNED", "PICKING", "CHECKOUT", "PACKING", "RETURNING")
# 대기장소(S1/S2) 복귀 시 강제 헤딩 — map +x (오른쪽)
HOME_YAW = 0.0


def _wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


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
        self._aborted_missions: set[int] = set()
        self._abort_lock = threading.Lock()
        self._returning_home: set[str] = set()

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

    def abort_device(self, device_code: str) -> dict[str, Any]:
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

        abort_info = self.abort_device(code)
        home = home_for_device(code)

        with self._abort_lock:
            if code in self._returning_home:
                return {
                    "ok": True,
                    "deviceCode": code,
                    "home": home.id,
                    "alreadyReturning": True,
                    "mission": abort_info.get("mission"),
                }
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
                    time.sleep(0.3)
                except Exception:
                    pass
                # 미션 없이 홈으로 이동
                attempts = max(1, int(os.environ.get("PICK_NAV_RETRIES", "3")))
                for attempt in range(1, attempts + 1):
                    result = self.cart_port.navigate_pose(
                        code,
                        home.x,
                        home.y,
                        home.yaw,
                        timeout_sec=self._nav_timeout,
                        require_yaw=False,
                    )
                    if result == "ARRIVED":
                        break
                    if attempt < attempts:
                        time.sleep(1.0)
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
    ) -> None:
        # Nav2 aborts/replans mid-leg often; retry before failing the whole tour.
        attempts = max(1, int(os.environ.get("PICK_NAV_RETRIES", "3")))
        yaw_tol = float(os.environ.get("PICK_HOME_YAW_TOL_RAD", "0.12"))
        errors: list[str] = []
        for attempt in range(1, attempts + 1):
            if mission_id is not None:
                self._ensure_not_aborted(mission_id)
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
                    mission_id, f"nav retry {waypoint_id} {attempt}/{attempts}: {detail}"
                )
            if attempt < attempts:
                time.sleep(1.0)
                continue
            raise RuntimeError(
                f"nav failed at {waypoint_id} after {attempts} tries: "
                + " | ".join(errors)
            )

    def _aruco_dock_or_fail(
        self,
        device_code: str,
        waypoint_id: str,
        mission_id: int | None = None,
    ) -> None:
        marker_id = aruco_marker_id_for_waypoint(waypoint_id)
        if marker_id is None:
            return
        standoff = float(os.environ.get("ARUCO_DOCK_STANDOFF_M", "0.12"))
        timeout = float(os.environ.get("ARUCO_DOCK_TIMEOUT_SEC", "60"))
        attempts = max(1, int(os.environ.get("PICK_ARUCO_RETRIES", "2")))
        last_detail = "unknown"

        phase_labels = {
            "SEARCH": "마커 탐색 중",
            "LOST": "마커 재탐색 중",
            "FACE": "정면·자세 정렬 중",
            "SHIFT": "횡방향 위치 조정 중",
            "APPROACH": "접근·파킹 중",
            "ARRIVED": "도킹 완료",
            "TIMEOUT": "도킹 타임아웃",
            "NO_MARKER": "마커 미검출",
        }

        for attempt in range(1, attempts + 1):
            if mission_id is not None:
                self._ensure_not_aborted(mission_id)
            dock = getattr(self.cart_port, "aruco_dock", None)
            if not callable(dock):
                return

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
                note = f"aruco dock {waypoint_id} ok"
                if dist is not None:
                    try:
                        note += f" distance={float(dist):.3f}"
                    except (TypeError, ValueError):
                        pass
                if mission_id is not None:
                    self._set_waypoint(
                        mission_id,
                        waypoint_id,
                        label_suffix="도킹 완료",
                    )
                    self._mission_note(mission_id, note)
                return
            last_detail = (
                getattr(self.cart_port, "last_aruco_error", None)
                or result.get("message")
                or result.get("status")
                or "unknown"
            )
            if mission_id is not None:
                self._set_waypoint(
                    mission_id,
                    waypoint_id,
                    label_suffix=f"도킹 재시도 {attempt}/{attempts}",
                )
            if attempt < attempts:
                time.sleep(0.5)
                continue
            raise RuntimeError(
                f"aruco dock failed at {waypoint_id} after {attempts} tries: {last_detail}"
            )

    def _dwell_at(
        self,
        device_code: str,
        mission_id: int,
        waypoint_id: str,
    ) -> None:
        """웨이포인트 도착 후 PICK_DWELL_SEC(기본 3s) 대기.

        stop_nav 를 호출하지 않는다. goal_wait 로 이미 도착한 뒤 stop 하면
        AMCL idle-freeze 가 걸리고, 그 사이 pose 조회 실패 시 홈 initialpose
        가 들어와 계산대 이동 중 대기장소로 점프한다.
        """
        self._ensure_not_aborted(mission_id)
        dwell = max(0.0, float(self._dwell_sec))
        self._set_waypoint(
            mission_id,
            waypoint_id,
            label_suffix=f"대기 {dwell:g}초",
        )
        self._mission_note(mission_id, f"dwell start {waypoint_id} {dwell}s")
        if dwell > 0:
            end = time.time() + dwell
            while time.time() < end:
                self._ensure_not_aborted(mission_id)
                time.sleep(min(0.25, end - time.time()))
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
        """Navigate to wait spot (S1/S2) using waypoint pose."""
        home = home_for_device(device_code)
        home_yaw = float(home.yaw)
        yaw_tol = float(os.environ.get("PICK_HOME_YAW_TOL_RAD", "0.12"))
        if stop_first:
            try:
                self.cart_port.stop_nav(device_code)
                time.sleep(0.3)
            except Exception:
                pass
        self._set_waypoint(mission_id, home.id)
        try:
            self._nav_or_fail(
                device_code,
                home.x,
                home.y,
                home_yaw,
                home.id,
                require_yaw=False,
                mission_id=mission_id,
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
                )
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
            if not self.cart_port.is_reachable(device_code):
                raise RuntimeError(
                    f"pinky unreachable for {device_code} "
                    f"(PINKY_ROBOTS URL / robot run.py 확인)"
                )

            self.ai_port.request_pick_plan(order_id)
            self.cart_port.notify_assign(device_code, order_id)

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
                self._ensure_not_aborted(mission_id)
                self._set_waypoint(mission_id, wp.id)
                self._nav_or_fail(
                    device_code,
                    wp.x,
                    wp.y,
                    wp.yaw,
                    wp.id,
                    mission_id=mission_id,
                )
                self._aruco_dock_or_fail(device_code, wp.id, mission_id)
                self._dwell_at(device_code, mission_id, wp.id)

            self._ensure_not_aborted(mission_id)
            self._set_status(order_id, mission_id, "CHECKOUT", note="checkout")
            c = get_waypoint("C")
            self._set_waypoint(mission_id, "C")
            self._nav_or_fail(
                device_code, c.x, c.y, c.yaw, "C", mission_id=mission_id
            )
            self._aruco_dock_or_fail(device_code, "C", mission_id)
            self._dwell_at(device_code, mission_id, "C")

            self._ensure_not_aborted(mission_id)
            self._set_status(order_id, mission_id, "PACKING", note="transport wait")
            p = get_waypoint("P")
            self._set_waypoint(mission_id, "P")
            self._nav_or_fail(
                device_code, p.x, p.y, p.yaw, "P", mission_id=mission_id
            )
            self._aruco_dock_or_fail(device_code, "P", mission_id)
            self._dwell_at(device_code, mission_id, "P")

            # 대기장소 복귀 중 — 모니터링 할당은 이때까지 유지, 도착 후 COMPLETED
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
        except Exception as exc:
            if self._is_aborted(mission_id):
                # 운영자 정지: 이미 FAILED 처리됨 — 그 자리 유지, 홈 복귀 안 함
                pass
            else:
                # Don't block forever on home return when pinky is already down
                note = f"failed:{exc}"
                try:
                    if self.cart_port.is_reachable(device_code):
                        self._set_status(
                            order_id,
                            mission_id,
                            "RETURNING",
                            note=f"abort return home:{exc}",
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
                    note = f"{note}; home exc:{home_exc}"
                self._set_waypoint(mission_id, None)
                self._set_status(order_id, mission_id, "FAILED", note=note)
                self._release_device(mission_id)
        finally:
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
