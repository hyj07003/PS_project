from __future__ import annotations

import json
import math
import os
import time
import urllib.error
import urllib.request
from typing import Any, Literal

ORDER_FLOW = [
    "CREATED",
    "ASSIGNED",
    "PICKING",
    "CHECKOUT",
    "PACKING",
    "RETURNING",
    "COMPLETED",
]


def _wrap_angle(a: float) -> float:
    """Normalize radians to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def parse_pinky_robot_urls() -> dict[str, str]:
    """
    cart-1 / cart-2 → base URL.
    Prefers PINKY_ROBOTS=cart-1=url,cart-2=url then PINKY_URL / PINKY_URL_2.
    """
    raw = (os.environ.get("PINKY_ROBOTS") or "").strip()
    out: dict[str, str] = {}
    if raw:
        for part in raw.split(","):
            part = part.strip()
            if not part:
                continue
            eq = part.index("=") if "=" in part else -1
            if eq > 0:
                code = part[:eq].strip()
                url = part[eq + 1 :].strip().rstrip("/")
                if code and url:
                    out[code] = url
        if out:
            return out

    single = (os.environ.get("PINKY_URL") or "").strip().rstrip("/")
    second = (os.environ.get("PINKY_URL_2") or "").strip().rstrip("/")
    if single:
        out["cart-1"] = single
    if second:
        out["cart-2"] = second
    elif single and "cart-2" not in out:
        # single-robot demo: both codes hit same pinky
        out.setdefault("cart-1", single)
    return out


class MockCartAdapter:
    """Local demo without Pinky HTTP — simulate travel delay."""

    def notify_assign(self, device_code: str, order_id: int = 0) -> dict[str, Any]:
        time.sleep(0.1)
        return {"ok": True, "deviceCode": device_code, "orderId": order_id}

    def set_initial_pose(
        self, device_code: str, x: float, y: float, yaw: float = 0.0
    ) -> dict[str, Any]:
        del device_code
        return {
            "success": True,
            "message": "mock initialpose",
            "pose": {"x": x, "y": y, "yaw": yaw},
        }

    def navigate_pose(
        self,
        device_code: str,
        x: float,
        y: float,
        yaw: float = 0.0,
        timeout_sec: float = 120.0,
        **_kwargs: Any,
    ) -> Literal["ARRIVED", "FAILED"]:
        del device_code, x, y, yaw, timeout_sec, _kwargs
        time.sleep(0.5)
        return "ARRIVED"

    def get_pose(self, device_code: str) -> dict[str, float] | None:
        del device_code
        return None

    def is_reachable(self, device_code: str) -> bool:
        del device_code
        return True

    def stop_nav(self, device_code: str) -> dict[str, Any]:
        del device_code
        return {"success": True, "message": "mock stop"}


class MockStationAdapter:
    def start_picking(self, order_id: int = 0) -> Literal["DONE", "FAILED"]:
        del order_id
        return "DONE"

    def checkout(self, order_id: int = 0) -> Literal["DONE", "FAILED"]:
        del order_id
        return "DONE"

    def pack(self, order_id: int = 0) -> Literal["DONE", "FAILED"]:
        del order_id
        return "DONE"


class MockAiAdapter:
    def request_pick_plan(self, order_id: int = 0) -> dict[str, list[str]]:
        del order_id
        return {"waypoints": []}


class PinkyHttpCartAdapter:
    """Per-robot Pinky Flask: assign + Nav2 goal_wait (or goal+poll fallback)."""

    def __init__(self, urls: dict[str, str] | None = None):
        self.urls = urls if urls is not None else parse_pinky_robot_urls()
        self.last_nav_error: str | None = None
        # Older pinky builds lack /nav/goal_wait — fall back once discovered.
        self._goal_wait_supported: dict[str, bool] = {}

    def _base(self, device_code: str) -> str | None:
        # Do not silently send cart-2 goals to cart-1's URL.
        if device_code in self.urls:
            return self.urls[device_code]
        if len(self.urls) == 1:
            return next(iter(self.urls.values()))
        return None

    def _read_json_response(self, res: Any) -> dict:
        return json.loads(res.read().decode("utf-8"))

    def _post(
        self,
        device_code: str,
        path: str,
        body: dict,
        timeout: float = 10.0,
        *,
        accept_http_error: bool = False,
    ) -> dict:
        base = self._base(device_code)
        if not base:
            raise RuntimeError(f"no pinky URL for {device_code}")
        req = urllib.request.Request(
            f"{base}{path}",
            data=json.dumps(body).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=timeout) as res:
                return self._read_json_response(res)
        except urllib.error.HTTPError as exc:
            raw = exc.read().decode("utf-8", errors="replace")
            if accept_http_error:
                try:
                    payload = json.loads(raw) if raw else {}
                except json.JSONDecodeError:
                    payload = {"success": False, "message": raw[:200] or str(exc)}
                if isinstance(payload, dict):
                    payload.setdefault("httpStatus", exc.code)
                    return payload
            raise

    def _get(self, device_code: str, path: str, timeout: float = 5.0) -> dict:
        base = self._base(device_code)
        if not base:
            raise RuntimeError(f"no pinky URL for {device_code}")
        req = urllib.request.Request(f"{base}{path}", method="GET")
        with urllib.request.urlopen(req, timeout=timeout) as res:
            return self._read_json_response(res)

    def notify_assign(self, device_code: str, order_id: int = 0) -> dict[str, Any]:
        try:
            return self._post(
                device_code,
                "/cmd/assign",
                {"orderId": order_id, "deviceCode": device_code},
            )
        except Exception as exc:
            return {"ok": False, "message": str(exc)}

    def set_initial_pose(
        self, device_code: str, x: float, y: float, yaw: float = 0.0
    ) -> dict[str, Any]:
        try:
            return self._post(
                device_code,
                "/nav/initialpose",
                {"x": x, "y": y, "yaw": yaw},
                timeout=5.0,
            )
        except Exception as exc:
            return {"success": False, "message": str(exc)}

    def navigate_pose(
        self,
        device_code: str,
        x: float,
        y: float,
        yaw: float = 0.0,
        timeout_sec: float = 180.0,
        *,
        require_yaw: bool = False,
        yaw_tol_rad: float = 0.12,
    ) -> Literal["ARRIVED", "FAILED"]:
        self.last_nav_error = None
        use_wait = self._goal_wait_supported.get(device_code, True)
        poll_kw = dict(require_yaw=require_yaw, yaw_tol_rad=yaw_tol_rad)
        if use_wait:
            try:
                result = self._post(
                    device_code,
                    "/nav/goal_wait",
                    {"x": x, "y": y, "yaw": yaw, "timeoutSec": timeout_sec},
                    timeout=timeout_sec + 15.0,
                    accept_http_error=True,
                )
                http_status = int(result.get("httpStatus") or 200)
                if http_status == 404:
                    self._goal_wait_supported[device_code] = False
                    return self._navigate_goal_poll(
                        device_code, x, y, yaw, timeout_sec, **poll_kw
                    )
                if result.get("success") or result.get("status") == "SUCCEEDED":
                    if require_yaw and not self._yaw_ok(device_code, yaw, yaw_tol_rad):
                        self.last_nav_error = "yaw not aligned after goal_wait"
                        return "FAILED"
                    return "ARRIVED"
                self.last_nav_error = str(
                    result.get("message") or result.get("status") or "goal_wait failed"
                )
                return "FAILED"
            except urllib.error.HTTPError as exc:
                if exc.code == 404:
                    self._goal_wait_supported[device_code] = False
                    return self._navigate_goal_poll(
                        device_code, x, y, yaw, timeout_sec, **poll_kw
                    )
                self.last_nav_error = f"HTTP {exc.code}"
                return "FAILED"
            except (urllib.error.URLError, TimeoutError, OSError, ValueError) as exc:
                self.last_nav_error = str(exc)
                return "FAILED"
        return self._navigate_goal_poll(
            device_code, x, y, yaw, timeout_sec, **poll_kw
        )

    def _yaw_ok(
        self, device_code: str, yaw: float, yaw_tol_rad: float
    ) -> bool:
        pose = self.get_pose(device_code)
        if not pose:
            return False
        return abs(_wrap_angle(float(pose.get("yaw") or 0.0) - float(yaw))) <= yaw_tol_rad

    def _navigate_goal_poll(
        self,
        device_code: str,
        x: float,
        y: float,
        yaw: float,
        timeout_sec: float,
        arrive_radius_m: float = 0.05,  # 0.55
        *,
        require_yaw: bool = False,
        yaw_tol_rad: float = 0.12,
    ) -> Literal["ARRIVED", "FAILED"]:
        """Compat path for pinky without /nav/goal_wait: POST /nav/goal + poll state."""
        try:
            sent = self._post(
                device_code,
                "/nav/goal",
                {"x": x, "y": y, "yaw": yaw},
                # ensure localization(AMCL activate+settle) 포함
                timeout=45.0,
                accept_http_error=True,
            )
        except (urllib.error.URLError, TimeoutError, OSError, ValueError) as exc:
            self.last_nav_error = f"goal send: {exc}"
            return "FAILED"
        if not sent.get("success"):
            self.last_nav_error = str(sent.get("message") or "goal not accepted")
            return "FAILED"

        deadline = time.time() + max(1.0, float(timeout_sec))
        saw_navigating = False
        idle_since: float | None = None
        idle_fail_sec = float(os.environ.get("PICK_NAV_IDLE_FAIL_SEC", "2.5"))
        near_ok_m = arrive_radius_m * 1.35

        def arrived(pose: dict, dist: float | None, radius: float) -> bool:
            if dist is None or dist > radius:
                return False
            if not require_yaw:
                return True
            py = float(pose.get("yaw") or 0.0)
            return abs(_wrap_angle(py - float(yaw))) <= yaw_tol_rad

        while time.time() < deadline:
            try:
                state = self._get(device_code, "/nav/state", timeout=3.0)
            except Exception:
                time.sleep(0.4)
                continue
            navigating = bool(state.get("navigating"))
            pose = state.get("pose") if isinstance(state.get("pose"), dict) else None
            dist = None
            if pose and pose.get("x") is not None and pose.get("y") is not None:
                dx = float(pose["x"]) - float(x)
                dy = float(pose["y"]) - float(y)
                dist = (dx * dx + dy * dy) ** 0.5
                if not navigating and arrived(pose, dist, arrive_radius_m):
                    return "ARRIVED"

            if navigating:
                saw_navigating = True
                idle_since = None
            elif saw_navigating:
                if idle_since is None:
                    idle_since = time.time()
                idle_for = time.time() - idle_since
                if pose and arrived(pose, dist, near_ok_m):
                    return "ARRIVED"
                if idle_for >= idle_fail_sec:
                    if (
                        require_yaw
                        and pose
                        and dist is not None
                        and dist <= near_ok_m
                    ):
                        err = abs(
                            _wrap_angle(float(pose.get("yaw") or 0.0) - float(yaw))
                        )
                        self.last_nav_error = f"yaw not aligned (err={err:.2f}rad)"
                    else:
                        self.last_nav_error = (
                            f"nav ended far from goal (dist={dist:.2f}m)"
                            if dist is not None
                            else "nav ended without pose"
                        )
                    return "FAILED"
            time.sleep(0.35)

        try:
            state = self._get(device_code, "/nav/state", timeout=3.0)
            pose = state.get("pose") if isinstance(state.get("pose"), dict) else None
            if pose and pose.get("x") is not None and pose.get("y") is not None:
                dx = float(pose["x"]) - float(x)
                dy = float(pose["y"]) - float(y)
                dist = (dx * dx + dy * dy) ** 0.5
                if arrived(pose, dist, near_ok_m):
                    return "ARRIVED"
                self.last_nav_error = f"nav poll timeout (dist={dist:.2f}m)"
            else:
                self.last_nav_error = "nav poll timeout"
        except Exception:
            self.last_nav_error = "nav poll timeout"
        return "FAILED"

    def get_pose(self, device_code: str) -> dict[str, float] | None:
        try:
            state = self._get(device_code, "/nav/state")
            pose = state.get("pose")
            if (
                isinstance(pose, dict)
                and pose.get("x") is not None
                and pose.get("y") is not None
            ):
                return {
                    "x": float(pose["x"]),
                    "y": float(pose["y"]),
                    "yaw": float(pose.get("yaw") or 0.0),
                }
        except Exception:
            pass
        return None

    def is_reachable(self, device_code: str) -> bool:
        if self._base(device_code) is None:
            return False
        try:
            self._get(device_code, "/health", timeout=2.0)
            return True
        except Exception:
            return False

    def stop_nav(self, device_code: str) -> dict[str, Any]:
        try:
            return self._post(
                device_code,
                "/nav/stop",
                {},
                timeout=5.0,
                accept_http_error=True,
            )
        except Exception as exc:
            return {"success": False, "message": str(exc)}
