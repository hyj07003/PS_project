"""OMX 포장 서버 — 관제서버가 부르는 HTTP 인터페이스.

픽업 서버(omx_yolo.server)와 **같은 규약**이다. 관제 입장에서는 어댑터가
하나 더 느는 것뿐이고, 포트와 경로 이름만 다르다.

    관제 :4100 ──┬─> 픽업 :8080   POST /pick   (SmolVLA + YOLO 주석)
                 └─> 포장 :8081   POST /pack   (ACT, 주석 없음)

규약은 pinky 어댑터(PinkyHttpCartAdapter)에서 그대로 가져왔다:
  · POST 는 JSON 본문, 필드는 camelCase (orderId, deviceCode, timeoutSec)
  · 응답은 {"success": bool, "status": ..., "message": str}
  · GET /health 로 도달 가능 여부를 판단한다 (is_reachable)
  · 진행 상태는 폴링으로 읽는다
  · 인증 없음

엔드포인트
  POST /pack        {"orderId","deviceCode","maxAttempts"}  → 202, 즉시 반환
  GET  /pack/state  진행 상태 (boxEmpty 가 완료 여부)
  POST /pack/stop   {"mode":"afterCurrent"|"immediate"}
  POST /home        홈 복귀 (복구용)
  GET  /health      장치 점검
  GET  /baskets     아는 deviceCode·바구니 목록
  GET  /view /stream /frame.jpg   화면

픽업의 /pick 과 다른 점 두 가지.

**slug 가 없다.** ACT 정책은 언어 조건이 없어서 무엇을 담을지 지정할 수 없다 —
적재함에 있는 것을 담을 뿐이다. 어느 바구니에 담을지는 deviceCode 가
정한다(cart-1→yellow, cart-2→mint).

**quantity 가 없다.** 작업의 단위는 "이 적재함을 비워라" 이지 "몇 개를
담아라" 가 아니다. 완료는 탑뷰로 적재함을 보고 판정한다(boxcheck.py).
maxAttempts 는 재시도 횟수다 — 한 번에 다 옮기지 못하는 일이 흔하다(실측 5/9).
옛 요청의 quantity 는 재시도 횟수로 읽어 준다.

실행:
    PYTHONPATH=/home/newuser/il_ws/src \
    ~/venv/pack/bin/python -m omx_pack.server --mock --port 8081
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .vocab import (BOX_CAPACITY, CONTROLLER_DEVICE_BASKET, DEFAULT_CHECKPOINTS,
                    resolve_basket)

logger = logging.getLogger("omx_pack")

VIEW_HTML = """<!doctype html><html lang="ko"><head><meta charset="utf-8">
<title>OMX 포장 화면</title>
<style>
 :root{color-scheme:dark light}
 body{margin:0;background:#0d1116;color:#e3eaf0;
      font-family:ui-monospace,Menlo,Consolas,monospace}
 header{padding:14px 20px;border-bottom:1px solid #28323c;display:flex;
        gap:16px;align-items:baseline;flex-wrap:wrap}
 h1{font-size:16px;margin:0;letter-spacing:-.02em}
 #st{font-size:12px;color:#7a8895}
 #st b{color:#4fcb84}
 main{display:flex;gap:18px;padding:18px;flex-wrap:wrap}
 figure{margin:0}
 img{width:min(46vw,640px);border-radius:8px;border:1px solid #28323c;
     background:#000;display:block}
 figcaption{font-size:12px;color:#7a8895;margin-top:6px}
 .tag{display:inline-block;padding:1px 6px;border-radius:4px;
      background:#1b2530;color:#9fb3c8;margin-right:6px}
</style></head><body>
<header>
  <h1>OMX 포장</h1>
  <div id="st">상태를 불러오는 중…</div>
</header>
<main>
  <figure><img src="/stream?cam=front&fps=12" alt="포장 탑뷰">
    <figcaption><span class="tag">front</span> 포장 탑뷰 — 정책이 보는 화면</figcaption></figure>
  <figure><img src="/stream?cam=wrist&fps=12" alt="손목">
    <figcaption><span class="tag">wrist</span> 포장 팔 손목</figcaption></figure>
</main>
<script>
async function tick(){
  try{
    const r = await fetch('/pack/state'); const s = await r.json();
    const el = document.getElementById('st');
    if(s.status === 'IDLE'){ el.textContent = '대기 중'; return; }
    const box = s.boxEmpty === true ? '적재함 비움'
              : s.boxEmpty === false ? '적재함에 물건 남음'
              : '적재함 확인 전';
    el.innerHTML = `<b>${s.status}</b> · ${s.basket||'?'} 바구니 · `
      + `시도 ${s.attempt}/${s.maxAttempts} · ${box} · ${s.elapsedSec}s`
      + (s.stopRequested ? ` · 정지요청(${s.stopRequested})` : '')
      + (s.message ? ` · ${s.message}` : '');
  }catch(e){ document.getElementById('st').textContent = '연결 끊김'; }
}
tick(); setInterval(tick, 500);
</script></body></html>"""


def make_handler(arm):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt, *args):            # 기본 stderr 로그 억제
            logger.info("%s - %s", self.address_string(), fmt % args)

        # ── 응답 도우미 ────────────────────────────────────────────
        def _send(self, code: int, payload: dict) -> None:
            body = json.dumps(payload, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _fail(self, code: int, message: str, **extra) -> None:
            self._send(code, {"success": False, "status": "FAILED",
                              "message": message, **extra})

        def _q(self, key: str, default: str) -> str:
            from urllib.parse import parse_qs, urlparse

            return parse_qs(urlparse(self.path).query).get(key, [default])[0]

        def _send_html(self, html: str) -> None:
            body = html.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _stream(self, cam: str, fps: float) -> None:
            """MJPEG(multipart/x-mixed-replace). 브라우저 <img> 로 바로 보인다.

            길이를 미리 알 수 없으므로 keep-alive 를 끊는다. ThreadingHTTPServer
            라 이 연결이 오래 열려 있어도 다른 요청을 막지 않는다.
            """
            period = 1.0 / max(1.0, min(float(fps), 30.0))
            bound = "omxframe"
            self.close_connection = True
            self.send_response(200)
            self.send_header("Content-Type",
                             f"multipart/x-mixed-replace; boundary={bound}")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            try:
                while True:
                    t0 = time.time()
                    jpg = arm.encode_jpeg(arm.get_frame(cam))
                    if jpg:
                        self.wfile.write(
                            f"--{bound}\r\nContent-Type: image/jpeg\r\n"
                            f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                    time.sleep(max(0.0, period - (time.time() - t0)))
            except (BrokenPipeError, ConnectionResetError):
                pass                                   # 브라우저가 창을 닫음

        # ── GET ────────────────────────────────────────────────────
        def do_GET(self):                             # noqa: N802
            path = self.path.split("?")[0].rstrip("/") or "/"
            if path == "/health":
                self._send(200, arm.health())
            elif path in ("/pack/state", "/status"):
                self._send(200, arm.state())
            elif path == "/view":
                self._send_html(VIEW_HTML)
            elif path == "/baskets":
                self._send(200, {
                    "success": True, "status": "OK",
                    "devices": CONTROLLER_DEVICE_BASKET,
                    "baskets": sorted(DEFAULT_CHECKPOINTS),
                    "boxCapacity": BOX_CAPACITY,
                    "loaded": getattr(arm, "basket", None),
                    "message": ""})
            elif path in ("/frame", "/frame.jpg"):
                cam = self._q("cam", "front")
                jpg = arm.encode_jpeg(arm.get_frame(cam))
                if jpg is None:
                    self._fail(503, f"프레임을 가져오지 못했습니다: {cam}")
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpg)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(jpg)
            elif path == "/stream":
                self._stream(self._q("cam", "front"), float(self._q("fps", "10")))
            else:
                self._fail(404, f"unknown path: {path}")

        # ── POST ───────────────────────────────────────────────────
        def do_POST(self):                            # noqa: N802
            path = self.path.split("?")[0].rstrip("/") or "/"
            n = int(self.headers.get("Content-Length") or 0)
            try:
                req = json.loads(self.rfile.read(n) or b"{}")
            except json.JSONDecodeError as e:
                self._fail(400, f"JSON 파싱 실패: {e}")
                return
            if not isinstance(req, dict):
                self._fail(400, "JSON 본문은 객체여야 합니다")
                return

            if path == "/pack/stop":
                try:
                    self._send(200, arm.request_stop(
                        str(req.get("mode", "afterCurrent"))))
                except ValueError as e:
                    self._fail(400, str(e))
                return

            if path == "/home":
                if arm.busy:
                    self._fail(409, "포장 작업이 진행 중입니다. 먼저 /pack/stop 하십시오.",
                               status="RUNNING")
                    return
                if not arm.lock.acquire(blocking=False):
                    self._fail(409, "다른 요청을 처리 중입니다", status="RUNNING")
                    return
                try:
                    arm.go_home()
                    self._send(200, {"success": True, "status": "DONE",
                                     "message": "홈 자세로 복귀했습니다"})
                except NotImplementedError as e:
                    self._fail(501, str(e))
                except Exception as e:                # noqa: BLE001
                    logger.exception("홈 복귀 실패")
                    self._fail(500, str(e))
                finally:
                    arm.lock.release()
                return

            if path != "/pack":
                self._fail(404, f"unknown path: {path}")
                return

            # ── POST /pack ──────────────────────────────────────────
            if arm.busy:
                self._fail(409, "이미 포장 작업을 처리 중입니다. 팔이 하나뿐입니다.",
                           status="RUNNING", jobId=(arm.job or {}).get("jobId"))
                return
            if not arm.lock.acquire(blocking=False):
                self._fail(409, "다른 요청을 처리 중입니다", status="RUNNING")
                return
            try:
                device_code = str(req.get("deviceCode", ""))
                basket = resolve_basket(device_code)

                # 올려둔 체크포인트와 다른 바구니를 요청하면 거절한다.
                #
                # 조용히 다른 바구니 모델로 돌리면 팔이 엉뚱한 자리로 간다.
                # 픽업에서 지시문 표기가 어긋나 샌드위치를 집었던 것과 같은
                # 종류의 사고다 — 예외도 로그도 없이 잘못 움직인다.
                loaded = getattr(arm, "basket", None)
                if loaded is not None and basket != loaded:
                    self._fail(409,
                               f"이 서버는 {loaded} 바구니 모델을 올려 두었습니다. "
                               f"{device_code}({basket}) 요청은 처리할 수 없습니다.",
                               status="WRONG_BASKET", loaded=loaded, requested=basket)
                    return

                # quantity 는 더 쓰지 않는다. 포장 작업의 단위는 "이 적재함을
                # 비워라" 이지 "몇 개를 담아라" 가 아니다(2026-08-21 정리).
                # 옛 요청이 들어와도 깨지지 않도록 받아는 주되 재시도 횟수로
                # 읽고, 로그로 알린다.
                if "quantity" in req and "maxAttempts" not in req:
                    logger.info("quantity 는 maxAttempts 로 바뀌었습니다 — "
                                "%s 를 재시도 횟수로 읽습니다", req["quantity"])
                attempts = int(req.get("maxAttempts", req.get("quantity", 3)))

                out = arm.start_job(
                    basket=basket,
                    device_code=device_code,
                    max_attempts=attempts,
                    order_id=int(req.get("orderId", 0)),
                    timeout_s=float(req.get("timeoutSec", 90.0)),
                )
                self._send(202, out)
            except ValueError as e:
                self._fail(400, str(e))
            except Exception as e:                    # noqa: BLE001
                logger.exception("작업 시작 실패")
                self._fail(500, str(e))
            finally:
                arm.lock.release()

    return Handler


def main() -> None:
    p = argparse.ArgumentParser(description="OMX 포장 서버")
    p.add_argument("--mock", action="store_true",
                   help="하드웨어 없이 HTTP·작업·인터럽트만 시험한다")
    p.add_argument("--mock-episode-sec", type=float, default=4.0,
                   help="가짜 팔의 에피소드 길이(초)")
    p.add_argument("--mock-behavior", default="return-home",
                   choices=("return-home", "stop", "keep-moving"),
                   help="가짜 팔이 담기를 끝낸 뒤 하는 행동. 실기에서 어느 쪽인지 "
                        "확인되기 전에 궤적 분석을 시험하기 위한 것")
    p.add_argument("--trace-dir", default=None,
                   help="에피소드 관절 궤적을 이 디렉터리에 .npz 로 남긴다. "
                        "종료 판정과 홈 자세를 측정으로 정하려면 켜 둘 것 "
                        "(분석: python -m omx_pack.trace <디렉터리>)")
    p.add_argument("--basket", default=os.environ.get("PACK_BASKET", "yellow"),
                   choices=sorted(DEFAULT_CHECKPOINTS),
                   help="이 서버가 올려 둘 바구니 모델")
    p.add_argument("--checkpoint", default=None,
                   help="체크포인트 경로를 직접 지정 (기본은 --basket 으로 결정)")
    p.add_argument("--port", type=int, default=int(os.environ.get("PACK_PORT", "8081")))
    p.add_argument("--host", default=os.environ.get("PACK_HOST", "0.0.0.0"))
    p.add_argument("--robot-port", default="/dev/omx_follower_2")
    p.add_argument("--robot-id", default="omx_pack_arm")
    p.add_argument("--front", default="/dev/omx_cam_pack_top")
    p.add_argument("--wrist", default="/dev/omx_cam_pack_hand")
    p.add_argument("--home-after", action="store_true",
                   help="작업이 끝나면 팔을 대기 자세로 되돌린다. "
                        "홈 값은 `python -m omx_pack.home --capture` 로 기록")
    p.add_argument("--strict-start", action="store_true",
                   help="시작 자세가 학습 범위 밖이면 /pack 을 400 으로 거절한다. "
                        "기본은 경고만 남기고 진행")
    p.add_argument("--observe-only", action="store_true",
                   help="팔에 명령을 보내지 않고 관측만 한다. 첫 하드웨어 연결에서 "
                        "연결·카메라·제어주기·홈 자세를 안전하게 확인할 때 쓴다")
    p.add_argument("--finish", default="duration",
                   choices=("duration", "stall", "box-empty"),
                   help="에피소드 종료 판정 방식. box-empty 는 적재함이 비면 끊는다 "
                        "(권장). finish.py 참조")
    p.add_argument("--box", default="box1",
                   help="적재함 판정에 쓸 ROI. box1|box2 또는 cart-1|cart-2")
    p.add_argument("--finish-sec", type=float, default=60.0,
                   help="duration 방식에서 한 에피소드를 돌릴 시간(초)")
    a = p.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")

    if a.mock:
        from .arm import MockArm

        arm = MockArm(episode_sec=a.mock_episode_sec, behavior=a.mock_behavior,
                      trace_dir=a.trace_dir, home_after=a.home_after)
        logger.info("가짜 팔로 시작합니다 — 하드웨어에 연결하지 않습니다 "
                    "(에피소드 %.1f초 · 행동 %s)",
                    a.mock_episode_sec, a.mock_behavior)
    else:
        from .arm import PackArm

        arm = PackArm(basket=a.basket, robot_port=a.robot_port,
                      robot_id=a.robot_id, front_device=a.front,
                      wrist_device=a.wrist, checkpoint=a.checkpoint,
                      finish=a.finish, finish_sec=a.finish_sec,
                      trace_dir=a.trace_dir, observe_only=a.observe_only,
                      strict_start=a.strict_start, box_name=a.box,
                      home_after=a.home_after)

    # 기동 직후 자세를 한 번 찍어 준다. 서버를 띄운 사람이 요청을 보내기
    # 전에 보게 하려는 것이다 — 요청 시점의 경고는 로그에 묻히기 쉽다.
    if not a.mock and getattr(arm, "range", None) is not None:
        try:
            from .dist import format_report

            obs = arm.robot.get_observation()
            logger.info("시작 자세 점검\n%s",
                        format_report(arm.range.check(arm._state_vec(obs))))
        except Exception as exc:                      # noqa: BLE001
            logger.warning("시작 자세를 읽지 못했습니다: %s", exc)

    srv = ThreadingHTTPServer((a.host, a.port), make_handler(arm))
    logger.info("서버 시작 http://%s:%d  (바구니 %s · 종료판정 %s%s)",
                a.host, a.port, getattr(arm, "basket", "mock"), a.finish,
                f" · 궤적 {a.trace_dir}" if a.trace_dir else "")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        logger.info("종료 중...")
    finally:
        srv.server_close()
        arm.close()


if __name__ == "__main__":
    main()
