from __future__ import annotations

import json
import logging
import math
import os
import time
import urllib.error
import urllib.request
from typing import Any, Callable, Literal

from .config import get_omx_connect_timeout, get_omx_url

logger = logging.getLogger(__name__)

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


def parse_omx_url() -> str | None:
    """OMX 로봇팔 PC URL (관제 PC와 다른 머신일 수 있음)."""
    return get_omx_url()


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

    def get_nav_state(self, device_code: str) -> dict[str, Any]:
        del device_code
        return {
            "arucoDock": {
                "active": False,
                "phase": None,
                "phaseLabel": None,
            },
            "nav2Readiness": {
                "ready": True,
                "tfValid": True,
                "scanFresh": True,
                "failures": [],
            },
            "navigationAction": {"state": "UNKNOWN", "goalId": None},
        }

    def get_nav_state_full(self, device_code: str) -> dict[str, Any]:
        return self.get_nav_state(device_code)

    def plan_pose(
        self,
        device_code: str,
        x: float,
        y: float,
        yaw: float = 0.0,
        timeout_sec: float = 10.0,
    ) -> dict[str, Any]:
        del device_code, timeout_sec
        return {
            "success": True,
            "path": {
                "poses": [
                    {"x": 0.0, "y": 0.0},
                    {"x": float(x), "y": float(y)},
                ],
            },
        }

    def get_active_path(self, device_code: str) -> dict[str, Any]:
        del device_code
        return {
            "success": True,
            "path": {
                "poses": [{"x": 0.0, "y": 0.0}, {"x": 0.5, "y": 0.0}],
            },
        }

    def is_reachable(self, device_code: str) -> bool:
        del device_code
        return True

    def stop_nav(self, device_code: str, *, freeze: bool = True) -> dict[str, Any]:
        del device_code, freeze
        return {"success": True, "message": "mock stop"}

    def aruco_dock(
        self,
        device_code: str,
        marker_id: int,
        standoff_m: float = 0.07,
        timeout_sec: float = 45.0,
    ) -> dict[str, Any]:
        del device_code, timeout_sec
        self.last_aruco_error = None
        return {
            "success": True,
            "status": "ARRIVED",
            "message": "mock aruco dock",
            "markerId": int(marker_id),
            "distanceM": float(standoff_m),
            "approachTravelM": float(standoff_m),
        }

    def aruco_undock(
        self,
        device_code: str,
        marker_id: int,
        target_range_m: float,
        *,
        timeout_sec: float = 30.0,
        speed_mps: float = 0.02,
        max_travel_m: float | None = None,
    ) -> dict[str, Any]:
        del device_code
        return {
            "success": True,
            "status": "UNDOCKED",
            "message": "mock aruco undock",
            "markerId": int(marker_id),
            "targetRangeM": float(target_range_m),
            "distanceM": float(target_range_m),
            "movedM": 0.08,
        }

    def relative_move(
        self,
        device_code: str,
        distance_m: float,
        *,
        speed_mps: float = 0.02,
        timeout_sec: float | None = None,
        bypass_collision: bool = False,
        ignore_scan: bool = False,
    ) -> dict[str, Any]:
        del device_code, speed_mps, timeout_sec, bypass_collision, ignore_scan
        return {
            "success": True,
            "message": "mock relative move",
            "movedM": abs(float(distance_m)),
            "requestedM": float(distance_m),
        }


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


class OmxHttpStationAdapter:
    """HTTP adapter for OMX robot arm pick API."""

    def __init__(self, url: str | None = None, poll_sec: float | None = None) -> None:
        self.url = url if url is not None else parse_omx_url()
        self.poll_sec = float(
            poll_sec
            if poll_sec is not None
            else os.environ.get("OMX_POLL_SEC", "0.5")
        )
        self.pick_timeout_sec = float(os.environ.get("OMX_PICK_TIMEOUT_SEC", "90"))
        self.connect_timeout = get_omx_connect_timeout()
        self.last_error: str | None = None
        self.last_state: dict[str, Any] = {}
        if self.url:
            logger.info(
                "OMX adapter: %s (connect_timeout=%.1fs, poll=%.2fs)",
                self.url,
                self.connect_timeout,
                self.poll_sec,
            )

    def _request_timeout(self, timeout: float) -> float:
        return max(float(timeout), self.connect_timeout)

    def _post(self, path: str, body: dict[str, Any], timeout: float = 10.0) -> dict[str, Any]:
        if not self.url:
            raise RuntimeError("OMX_URL is not configured")
        req = urllib.request.Request(
            f"{self.url}{path}",
            data=json.dumps(body).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(
                req, timeout=self._request_timeout(timeout)
            ) as res:
                return json.loads(res.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            raw = exc.read().decode("utf-8", errors="replace")
            try:
                payload = json.loads(raw) if raw else {}
            except json.JSONDecodeError:
                payload = {"success": False, "message": raw[:200] or str(exc)}
            if isinstance(payload, dict):
                payload.setdefault("httpStatus", exc.code)
                return payload
            return {"success": False, "message": str(exc), "httpStatus": exc.code}

    def _get(self, path: str, timeout: float = 5.0) -> dict[str, Any]:
        if not self.url:
            raise RuntimeError("OMX_URL is not configured")
        req = urllib.request.Request(f"{self.url}{path}", method="GET")
        with urllib.request.urlopen(
            req, timeout=self._request_timeout(timeout)
        ) as res:
            return json.loads(res.read().decode("utf-8"))

    def is_reachable(self) -> bool:
        if not self.url:
            return False
        try:
            health = self._get("/health", timeout=min(3.0, self.connect_timeout))
            return bool(health.get("robotConnected", False))
        except Exception as exc:
            logger.debug("OMX unreachable at %s: %s", self.url, exc)
            return False

    def supported_slugs(self) -> list[str]:
        try:
            return list(self._get("/products", timeout=3.0).get("slugs") or [])
        except Exception:
            return []

    def pick(
        self,
        *,
        device_code: str,
        slug: str,
        quantity: int,
        order_id: int,
        timeout_sec: float | None = None,
        should_abort: Callable[[], bool] | None = None,
        on_progress: Callable[[int, int], None] | None = None,
        force_success_on_unreachable: bool = False,
    ) -> Literal["DONE", "FAILED", "ABORTED"]:
        self.last_error = None
        self.last_state = {}
        try:
            started = self._post(
                "/pick",
                {
                    "orderId": int(order_id),
                    "deviceCode": device_code,
                    "slug": slug,
                    "quantity": int(quantity),
                    "timeoutSec": float(timeout_sec or self.pick_timeout_sec),
                },
                timeout=15.0,
            )
        except (urllib.error.URLError, TimeoutError, OSError, ValueError) as exc:
            self.last_error = f"omx request failed: {exc}"
            return "DONE" if force_success_on_unreachable else "FAILED"
        except Exception as exc:
            self.last_error = f"omx request failed: {exc}"
            return "DONE" if force_success_on_unreachable else "FAILED"

        if not started.get("success"):
            self.last_error = str(started.get("message") or "omx pick rejected")
            if int(started.get("httpStatus") or 0) <= 0 and force_success_on_unreachable:
                return "DONE"
            return "FAILED"

        effective_timeout = float(timeout_sec or self.pick_timeout_sec)
        deadline = time.time() + max(5.0, effective_timeout * max(1, int(quantity)))
        stop_sent = False
        last_done = -1
        while time.time() < deadline:
            time.sleep(max(0.2, self.poll_sec))
            try:
                state = self._get("/pick/state", timeout=3.0)
            except Exception as exc:
                self.last_error = f"omx state polling failed: {exc}"
                return "DONE" if force_success_on_unreachable else "FAILED"
            self.last_state = state
            done = int(state.get("done") or 0)
            total = int(state.get("total") or quantity)
            if on_progress and done != last_done:
                on_progress(done, total)
                last_done = done
            if should_abort and should_abort() and not stop_sent:
                stop_sent = True
                try:
                    self._post("/pick/stop", {"mode": "afterCurrent"}, timeout=5.0)
                except Exception:
                    pass
            status = str(state.get("status") or "")
            if status in ("DONE", "FAILED", "ABORTED"):
                if status in ("FAILED", "ABORTED"):
                    self.last_error = str(state.get("message") or status)
                return status  # type: ignore[return-value]
        self.last_error = "omx pick timeout"
        return "DONE" if force_success_on_unreachable else "FAILED"

    def stop(self, mode: str = "afterCurrent") -> dict[str, Any]:
        try:
            return self._post("/pick/stop", {"mode": mode}, timeout=5.0)
        except Exception as exc:
            return {"success": False, "message": str(exc)}


class MockAiAdapter:
    def request_pick_plan(self, order_id: int = 0) -> dict[str, list[str]]:
        del order_id
        return {"waypoints": []}


class PinkyHttpCartAdapter:
    """Per-robot Pinky Flask: assign + Nav2 goal_wait (or goal+poll fallback)."""

    def __init__(self, urls: dict[str, str] | None = None):
        self.urls = urls if urls is not None else parse_pinky_robot_urls()
        self.last_nav_error: str | None = None
        self.last_aruco_error: str | None = None
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
                    low = (raw or "").lower()
                    if "<html" in low or "<!doctype" in low:
                        msg = f"pinky HTTP {exc.code} Internal Server Error"
                    else:
                        msg = (raw[:200] if raw else "") or str(exc)
                    payload = {"success": False, "message": msg}
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
                msg = str(
                    result.get("message") or result.get("status") or "goal_wait failed"
                )
                st = result.get("status")
                http_status = int(result.get("httpStatus") or 200)
                if st and st not in msg:
                    msg = f"{msg} (status={st})"
                if http_status >= 400:
                    msg = f"HTTP {http_status}: {msg}"
                self.last_nav_error = msg
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

    def get_nav_state(self, device_code: str) -> dict[str, Any]:
        try:
            return self._get(device_code, "/nav/state", timeout=2.5)
        except Exception:
            return {}

    def get_nav_state_full(self, device_code: str) -> dict[str, Any]:
        return self.get_nav_state(device_code)

    def plan_pose(
        self,
        device_code: str,
        x: float,
        y: float,
        yaw: float = 0.0,
        timeout_sec: float = 10.0,
    ) -> dict[str, Any]:
        try:
            return self._post(
                device_code,
                "/nav/plan",
                {
                    "x": float(x),
                    "y": float(y),
                    "yaw": float(yaw),
                    "timeoutSec": float(timeout_sec),
                },
                timeout=float(timeout_sec) + 5.0,
                accept_http_error=True,
            )
        except (urllib.error.URLError, TimeoutError, OSError, ValueError) as exc:
            return {"success": False, "message": str(exc)}

    def get_active_path(self, device_code: str) -> dict[str, Any]:
        try:
            return self._get(device_code, "/nav/path", timeout=2.5)
        except Exception:
            return {"success": False, "path": None}

    def is_reachable(self, device_code: str) -> bool:
        if self._base(device_code) is None:
            return False
        try:
            self._get(device_code, "/health", timeout=2.0)
            return True
        except Exception:
            return False

    def stop_nav(self, device_code: str, *, freeze: bool = True) -> dict[str, Any]:
        try:
            return self._post(
                device_code,
                "/nav/stop",
                {"freeze": bool(freeze)},
                timeout=5.0,
                accept_http_error=True,
            )
        except Exception as exc:
            return {"success": False, "message": str(exc)}

    def aruco_dock(
        self,
        device_code: str,
        marker_id: int,
        standoff_m: float = 0.07,
        timeout_sec: float = 45.0,
    ) -> dict[str, Any]:
        """POST /nav/aruco_dock — blocking visual approach to marker."""
        self.last_aruco_error = None
        timeout = max(1.0, float(timeout_sec))
        try:
            result = self._post(
                device_code,
                "/nav/aruco_dock",
                {
                    "markerId": int(marker_id),
                    "standoffM": float(standoff_m),
                    "timeoutSec": timeout,
                },
                timeout=timeout + 20.0,
                accept_http_error=True,
            )
        except (urllib.error.URLError, TimeoutError, OSError, ValueError) as exc:
            self.last_aruco_error = str(exc)
            return {
                "success": False,
                "status": "FAILED",
                "message": str(exc),
                "markerId": int(marker_id),
            }
        if result.get("success") or result.get("status") == "ARRIVED":
            return result
        self.last_aruco_error = str(
            result.get("message") or result.get("status") or "aruco_dock failed"
        )
        return result

    def aruco_undock(
        self,
        device_code: str,
        marker_id: int,
        target_range_m: float,
        *,
        timeout_sec: float = 30.0,
        speed_mps: float = 0.02,
        max_travel_m: float | None = None,
    ) -> dict[str, Any]:
        """POST /nav/aruco_undock — reverse until range reaches target."""
        timeout = max(1.0, float(timeout_sec))
        payload: dict[str, Any] = {
            "markerId": int(marker_id),
            "targetRangeM": float(target_range_m),
            "timeoutSec": timeout,
            "speedMps": float(speed_mps),
        }
        if max_travel_m is not None:
            payload["maxTravelM"] = float(max_travel_m)
        try:
            return self._post(
                device_code,
                "/nav/aruco_undock",
                payload,
                timeout=timeout + 15.0,
                accept_http_error=True,
            )
        except (urllib.error.URLError, TimeoutError, OSError, ValueError) as exc:
            return {"success": False, "message": str(exc)}

    def relative_move(
        self,
        device_code: str,
        distance_m: float,
        *,
        speed_mps: float = 0.02,
        timeout_sec: float | None = None,
        bypass_collision: bool = False,
        ignore_scan: bool = False,
    ) -> dict[str, Any]:
        """POST /nav/relative_move — odom closed-loop micro motion (undock)."""
        payload: dict[str, Any] = {
            "distanceM": float(distance_m),
            "speedMps": float(speed_mps),
            "bypassCollision": bool(bypass_collision),
            "ignoreScan": bool(ignore_scan),
        }
        if timeout_sec is not None:
            payload["timeoutSec"] = float(timeout_sec)
        timeout = float(timeout_sec) if timeout_sec is not None else max(
            3.0, abs(float(distance_m)) / max(float(speed_mps), 0.01) + 2.0
        )
        try:
            return self._post(
                device_code,
                "/nav/relative_move",
                payload,
                timeout=timeout + 5.0,
                accept_http_error=True,
            )
        except (urllib.error.URLError, TimeoutError, OSError, ValueError) as exc:
            return {"success": False, "message": str(exc)}
