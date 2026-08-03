from __future__ import annotations

from typing import Any

import urllib.error
import urllib.request
import json


class ControllerClient:
    """SmartShop controller-server (`CONTROLLER_URL`) HTTP 클라이언트."""

    def __init__(self, base_url: str):
        self.base_url = base_url.rstrip("/")

    def _request(
        self,
        method: str,
        path: str,
        body: dict[str, Any] | None = None,
        timeout: float = 5.0,
    ) -> Any:
        url = f"{self.base_url}{path}"
        data = None
        headers = {"Accept": "application/json"}
        if body is not None:
            data = json.dumps(body).encode("utf-8")
            headers["Content-Type"] = "application/json"
        req = urllib.request.Request(url, data=data, headers=headers, method=method)
        try:
            with urllib.request.urlopen(req, timeout=timeout) as res:
                text = res.read().decode("utf-8")
                return json.loads(text) if text else None
        except urllib.error.HTTPError as exc:
            err_body = exc.read().decode("utf-8", errors="replace")
            try:
                payload = json.loads(err_body) if err_body else {}
            except json.JSONDecodeError:
                payload = {"message": err_body}
            message = payload.get("message") if isinstance(payload, dict) else err_body
            raise RuntimeError(f"controller {method} {path} → {exc.code}: {message}") from exc
        except urllib.error.URLError as exc:
            raise RuntimeError(f"controller unreachable: {exc.reason}") from exc

    def health(self) -> Any:
        return self._request("GET", "/health")

    def get_order(self, order_id: int) -> Any:
        return self._request("GET", f"/orders/{order_id}")

    def list_devices(self) -> Any:
        return self._request("GET", "/devices")

    def patch_device(self, code: str, status: str) -> Any:
        return self._request("PATCH", f"/devices/{code}", {"status": status})

    def list_missions(self, status: str | None = None, device_code: str | None = None) -> Any:
        qs = []
        if status:
            qs.append(f"status={status}")
        if device_code:
            qs.append(f"deviceCode={device_code}")
        path = "/missions" + (("?" + "&".join(qs)) if qs else "")
        return self._request("GET", path)

    def get_mission(self, mission_id: int) -> Any:
        return self._request("GET", f"/missions/{mission_id}")

    def patch_mission(
        self,
        mission_id: int,
        status: str,
        note: str | None = None,
    ) -> Any:
        body: dict[str, Any] = {"status": status}
        if note:
            body["note"] = note
        return self._request("PATCH", f"/missions/{mission_id}", body)

    def post_telemetry(self, payload: dict[str, Any]) -> Any:
        return self._request("POST", "/robot/telemetry", payload)

    def heartbeat(self, device_code: str, status: str = "idle") -> Any:
        return self.patch_device(device_code, status)
