"""OMX 픽업 스테이션 어댑터 — controller-server 쪽에 넣을 코드.

넣을 위치: server/apps/controller-server/app/adapters.py 맨 아래에 추가.
MockStationAdapter 와 같은 인터페이스를 유지하되, 수량과 인터럽트를 다룬다.

환경변수
    OMX_URL=http://127.0.0.1:8080        없으면 Mock 으로 동작
    OMX_POLL_SEC=0.5                     진행 상태 폴링 주기
    ADAPTER_MODE=mock                    기존 규약 그대로

pinky 어댑터와 같은 규약을 따른다:
  · POST JSON, camelCase 필드
  · 응답 {"success", "status", "message"}
  · GET /health 로 is_reachable 판단
  · 상태는 폴링으로 읽는다 (pinky 는 /nav/state 를 0.35초 주기로 폴링한다)
"""

from __future__ import annotations

import json
import os
import time
import urllib.error
import urllib.request
from typing import Any, Callable, Literal


def parse_omx_url() -> str | None:
    raw = (os.environ.get("OMX_URL") or "").strip().rstrip("/")
    return raw or None


class OmxHttpStationAdapter:
    """OMX 로봇팔 픽업 스테이션.

    MockStationAdapter 와 달리 수량을 받는다. 팔이 하나뿐이라 한 번에
    한 작업만 처리하며, 두 번째 요청은 409 로 거절된다.
    """

    def __init__(self, url: str | None = None, poll_sec: float | None = None):
        self.url = url if url is not None else parse_omx_url()
        self.poll_sec = float(
            poll_sec if poll_sec is not None
            else os.environ.get("OMX_POLL_SEC", "0.5"))
        self.last_error: str | None = None
        self.last_state: dict[str, Any] = {}

    # ── HTTP 기본 ────────────────────────────────────────────────
    def _post(self, path: str, body: dict, timeout: float = 10.0) -> dict:
        if not self.url:
            raise RuntimeError("OMX_URL 이 설정되지 않았습니다")
        req = urllib.request.Request(
            f"{self.url}{path}",
            data=json.dumps(body).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=timeout) as res:
                return json.loads(res.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            raw = exc.read().decode("utf-8", errors="replace")
            try:
                payload = json.loads(raw) if raw else {}
            except json.JSONDecodeError:
                payload = {"success": False, "message": raw[:200] or str(exc)}
            payload.setdefault("httpStatus", exc.code)
            return payload

    def _get(self, path: str, timeout: float = 5.0) -> dict:
        if not self.url:
            raise RuntimeError("OMX_URL 이 설정되지 않았습니다")
        req = urllib.request.Request(f"{self.url}{path}", method="GET")
        with urllib.request.urlopen(req, timeout=timeout) as res:
            return json.loads(res.read().decode("utf-8"))

    # ── 공개 API ─────────────────────────────────────────────────
    def is_reachable(self) -> bool:
        if not self.url:
            return False
        try:
            r = self._get("/health", timeout=2.0)
            return bool(r.get("robotConnected"))
        except Exception:
            return False

    def supported_slugs(self) -> list[str]:
        """OMX 가 집을 수 있는 상품 목록. cola 는 없다."""
        try:
            return list(self._get("/products", timeout=3.0).get("slugs") or [])
        except Exception:
            return []

    def pick(
        self,
        device_code: str,
        slug: str,
        quantity: int,
        order_id: int = 0,
        *,
        timeout_sec: float = 90.0,
        should_abort: Callable[[], bool] | None = None,
        on_progress: Callable[[int, int], None] | None = None,
    ) -> Literal["DONE", "FAILED", "ABORTED"]:
        """수량만큼 집어 카트에 담는다. 완료될 때까지 폴링하며 기다린다.

        should_abort  매 폴링마다 호출. True 를 돌려주면 OMX 에 정지를 요청한다.
                      orders.py 의 _ensure_not_aborted 와 같은 판단을 넣으면 된다.
        on_progress   (done, total) 로 진행 개수를 알려준다. 미션 노트에 쓰면 좋다.
        """
        self.last_error = None
        try:
            started = self._post("/pick", {
                "orderId": int(order_id),
                "deviceCode": device_code,
                "slug": slug,
                "quantity": int(quantity),
                "timeoutSec": float(timeout_sec),
            }, timeout=15.0)
        except (urllib.error.URLError, TimeoutError, OSError, ValueError) as exc:
            self.last_error = f"pick 요청 실패: {exc}"
            return "FAILED"

        if not started.get("success"):
            self.last_error = str(started.get("message") or "pick 거절됨")
            return "FAILED"

        # 작업 전체 여유: 픽업 1회 최대 timeout_sec + 리셋 여유
        deadline = time.time() + quantity * (timeout_sec + 15.0)
        last_done = -1
        stop_sent = False

        while time.time() < deadline:
            time.sleep(self.poll_sec)
            try:
                st = self._get("/pick/state", timeout=3.0)
            except Exception:
                continue
            self.last_state = st

            done = int(st.get("done") or 0)
            if on_progress and done != last_done:
                on_progress(done, int(st.get("total") or quantity))
                last_done = done

            if should_abort and should_abort() and not stop_sent:
                # 운영자 정지: 지금 집는 것만 마치고 세운다.
                # 팔이 물체를 든 채로 멈추면 회수가 번거로우므로 afterCurrent 가 기본이다.
                stop_sent = True
                try:
                    self._post("/pick/stop", {"mode": "afterCurrent"}, timeout=5.0)
                except Exception:
                    pass

            status = str(st.get("status") or "")
            if status in ("DONE", "FAILED", "ABORTED"):
                if status == "FAILED":
                    self.last_error = str(st.get("message") or "픽업 실패")
                elif status == "ABORTED":
                    self.last_error = str(st.get("message") or "정지됨")
                return status  # type: ignore[return-value]

        self.last_error = "OMX 응답 시간 초과"
        try:
            self._post("/pick/stop", {"mode": "immediate"}, timeout=5.0)
        except Exception:
            pass
        return "FAILED"

    def stop(self, mode: str = "afterCurrent") -> dict[str, Any]:
        try:
            return self._post("/pick/stop", {"mode": mode}, timeout=5.0)
        except Exception as exc:
            return {"success": False, "message": str(exc)}

    def home(self) -> dict[str, Any]:
        try:
            return self._post("/home", {}, timeout=20.0)
        except Exception as exc:
            return {"success": False, "message": str(exc)}

    # MockStationAdapter 호환 (기존 호출부가 있다면)
    def start_picking(self, order_id: int = 0) -> Literal["DONE", "FAILED"]:
        del order_id
        return "DONE"

    def checkout(self, order_id: int = 0) -> Literal["DONE", "FAILED"]:
        del order_id
        return "DONE"

    def pack(self, order_id: int = 0) -> Literal["DONE", "FAILED"]:
        del order_id
        return "DONE"
