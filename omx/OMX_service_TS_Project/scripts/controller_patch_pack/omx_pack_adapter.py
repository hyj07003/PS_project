# ═══════════════════════════════════════════════════════════════════════
#  관제 서버(controller-server) 에 추가할 포장 어댑터
#
#  설치: 이 파일 내용을 app/adapters.py 맨 아래에 붙인다.
#        (또는 app/omx_pack_adapter.py 로 두고 import 해도 된다)
#
#  기존 OmxHttpStationAdapter(픽업, :8080) 와 같은 규약이다. 다른 점만
#  적으면 아래 세 가지다.
#
#   ① slug 가 없다. 포장 정책(ACT)에는 언어 조건이 없어서 무엇을 담을지
#      지정할 방법이 없다. 적재함에 있는 것을 담을 뿐이다.
#
#   ② quantity 가 없다. 작업의 단위는 "이 적재함을 비워라" 이지 "몇 개를
#      담아라" 가 아니다. 완료는 OMX 가 탑뷰로 적재함을 보고 판정한다.
#      maxAttempts 는 재시도 횟수다 — 한 번에 다 옮기지 못하는 일이 흔하다
#      (2026-08-21 실측: 물건 3개씩 3회에서 3/1/1 개만 옮겼다).
#
#   ③ 팔이 다르다. 픽업 팔(:8080)과 포장 팔(:8081)은 별개 하드웨어이므로
#      _acquire_omx_arm 락을 공유하면 안 된다. 두 작업은 동시에 돌 수 있다.
# ═══════════════════════════════════════════════════════════════════════


def parse_omx_pack_url() -> str | None:
    """포장 팔 URL. 픽업의 get_omx_url() 과 같은 방식으로 정규화한다 —
    스킴이 없으면 http:// 를 붙인다(.env 에 IP만 적는 일이 있다).

    config.py 에 get_pack_url() 을 두는 편이 픽업과 대칭이지만, 그러면
    파일을 하나 더 건드려야 한다. 여기 두어도 동작은 같다.
    """
    raw = (os.environ.get("PACK_URL") or "").strip().rstrip("/")
    if not raw:
        return None
    if not raw.startswith(("http://", "https://")):
        raw = f"http://{raw}"
    return raw


def get_omx_pack_connect_timeout() -> float:
    return float(os.environ.get("PACK_CONNECT_TIMEOUT_SEC", "5"))


class OmxPackStationAdapter:
    """HTTP adapter for the OMX packing arm (:8081).

    사용 예:

        adapter = OmxPackStationAdapter()
        result = adapter.pack(
            device_code="cart-1",
            order_id=order_id,
            should_abort=lambda: self._is_aborted(mission_id),
            on_progress=lambda attempt, total, box: ...,
        )
        if result != "DONE":
            raise RuntimeError(adapter.last_error)
    """

    def __init__(self, url: str | None = None, poll_sec: float | None = None) -> None:
        self.url = url if url is not None else parse_omx_pack_url()
        self.poll_sec = float(
            poll_sec if poll_sec is not None
            else os.environ.get("PACK_POLL_SEC", "0.5")
        )
        # 한 시도(에피소드)의 상한. OMX 쪽 기본 timeoutSec 와 맞춘다.
        self.attempt_timeout_sec = float(
            os.environ.get("PACK_ATTEMPT_TIMEOUT_SEC", "90"))
        self.max_attempts = int(os.environ.get("PACK_MAX_ATTEMPTS", "3"))
        self.connect_timeout = get_omx_pack_connect_timeout()
        self.last_error: str | None = None
        self.last_state: dict[str, Any] = {}
        if self.url:
            logger.info(
                "OMX pack adapter: %s (connect_timeout=%.1fs, poll=%.2fs)",
                self.url, self.connect_timeout, self.poll_sec,
            )

    # ── HTTP 도우미 — 픽업 어댑터와 같은 모양 ──────────────────────
    def _request_timeout(self, timeout: float) -> float:
        return max(float(timeout), self.connect_timeout)

    def _post(self, path: str, body: dict[str, Any],
              timeout: float = 10.0) -> dict[str, Any]:
        if not self.url:
            raise RuntimeError("PACK_URL is not configured")
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
            raise RuntimeError("PACK_URL is not configured")
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
            logger.debug("OMX pack unreachable at %s: %s", self.url, exc)
            return False

    def supported_devices(self) -> list[str]:
        """포장이 아는 deviceCode 목록 (cart-1, cart-2)."""
        try:
            return sorted((self._get("/baskets", timeout=3.0)
                           .get("devices") or {}).keys())
        except Exception:
            return []

    # ── 본체 ───────────────────────────────────────────────────────
    def pack(
        self,
        *,
        device_code: str,
        order_id: int = 0,
        max_attempts: int | None = None,
        timeout_sec: float | None = None,
        should_abort: Callable[[], bool] | None = None,
        on_progress: Callable[[int, int, Any], None] | None = None,
        force_success_on_unreachable: bool = False,
    ) -> Literal["DONE", "FAILED", "ABORTED"]:
        """적재함을 비운다. 완료 판정은 OMX 가 탑뷰로 한다.

        on_progress(attempt, maxAttempts, box_empty) 로 진행을 흘린다.
        box_empty 는 True/False/None 이고 **None 은 "확인하지 못했다"** 다 —
        비지 않았다는 뜻이 아니다.
        """
        self.last_error = None
        self.last_state = {}
        attempts = int(max_attempts if max_attempts is not None else self.max_attempts)
        per_attempt = float(timeout_sec or self.attempt_timeout_sec)

        try:
            started = self._post(
                "/pack",
                {
                    "orderId": int(order_id),
                    "deviceCode": device_code,
                    "maxAttempts": attempts,
                    "timeoutSec": per_attempt,
                },
                timeout=15.0,
            )
        except Exception as exc:                      # noqa: BLE001
            self.last_error = f"omx pack request failed: {exc}"
            return "DONE" if force_success_on_unreachable else "FAILED"

        if not started.get("success"):
            self.last_error = str(started.get("message") or "omx pack rejected")
            if int(started.get("httpStatus") or 0) <= 0 and force_success_on_unreachable:
                return "DONE"
            return "FAILED"

        # 시도마다 per_attempt 가 걸릴 수 있고, 시도 사이에 적재함 확인
        # (최대 6초)이 끼므로 여유를 둔다.
        deadline = time.time() + max(10.0, (per_attempt + 10.0) * attempts)
        stop_sent = False
        last_seen = (-1, None)
        while time.time() < deadline:
            time.sleep(max(0.2, self.poll_sec))
            try:
                state = self._get("/pack/state", timeout=3.0)
            except Exception as exc:                  # noqa: BLE001
                self.last_error = f"omx pack state polling failed: {exc}"
                return "DONE" if force_success_on_unreachable else "FAILED"
            self.last_state = state

            attempt = int(state.get("attempt") or 0)
            total = int(state.get("maxAttempts") or attempts)
            box_empty = state.get("boxEmpty")
            if on_progress and (attempt, box_empty) != last_seen:
                on_progress(attempt, total, box_empty)
                last_seen = (attempt, box_empty)

            if should_abort and should_abort() and not stop_sent:
                stop_sent = True
                try:
                    self._post("/pack/stop", {"mode": "afterCurrent"}, timeout=5.0)
                except Exception:                     # noqa: BLE001
                    pass

            status = str(state.get("status") or "")
            if status in ("DONE", "FAILED", "ABORTED"):
                if status in ("FAILED", "ABORTED"):
                    self.last_error = str(state.get("message") or status)
                return status                          # type: ignore[return-value]

        self.last_error = "omx pack timeout"
        return "DONE" if force_success_on_unreachable else "FAILED"

    def stop(self, mode: str = "afterCurrent") -> dict[str, Any]:
        try:
            return self._post("/pack/stop", {"mode": mode}, timeout=5.0)
        except Exception as exc:                      # noqa: BLE001
            return {"success": False, "message": str(exc)}
