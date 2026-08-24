"""포장 팔 구동부.

두 가지 구현이 있고 HTTP 계층은 둘을 구분하지 않는다.

    MockArm  하드웨어 없이 도는 가짜 팔. HTTP·작업 모델·인터럽트만 시험한다.
    PackArm  실제 팔 + ACT 정책.

로봇 없이 먼저 만드는 이유는 픽업 때와 같다. 픽업 서버도 HTTP·작업·인터럽트
로직을 하드웨어 없이 다 잡아 놓고 팔에 붙였고, 그래서 팔이 놀고 있는 시간을
버리지 않았다.
"""

from __future__ import annotations

import logging
import threading
import time
from pathlib import Path

import numpy as np

from .dist import OUT, StateRange, format_report
from .finish import make_detector
from .trace import TraceWriter
from .vocab import DEFAULT_CHECKPOINTS, resolve_checkpoint

logger = logging.getLogger(__name__)

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")

# lerobot-rollout 으로 평가 데이터를 기록할 때 쓰는 --dataset.single_task
# 와 같은 문구로 맞춰 둔다. ACT 는 언어 조건이 없어 정책 동작은 바뀌지
# 않는다 — 순수하게 궤적·데이터셋에 남는 라벨이다.
TASK_LABEL = "Pick up one grocery and put it in the basket"



def _connect_with_retry(robot, attempts: int = 3, pause_s: float = 1.5) -> None:
    """로봇 연결을 몇 번 다시 시도한다.

    lerobot 의 연결 검사(_assert_motors_exist)는 각 모터에 ping 을 **한 번씩**
    보내고, 하나라도 응답이 없으면 연결 전체를 실패시킨다. 재시도가 없다.
    반면 scan_port 는 broadcast_ping 을 쓴다 — 그래서 조회로는 6개가 다 보이는데
    연결만 실패하는 일이 생긴다(2026-08-21 포장 팔 ID 11 에서 실제로 겪었다).

    패킷 한 개를 흘렸다고 서버가 못 뜨는 것은 곤란하므로 몇 번 다시 시도한다.
    다만 **가리지는 않는다** — 시도할 때마다 로그를 남기고, 끝내 실패하면
    그대로 예외를 올린다. 매번 재시도가 찍힌다면 그것은 배선을 봐야 한다는
    신호이지 넘어갈 일이 아니다.
    """
    last = None
    for i in range(1, attempts + 1):
        try:
            robot.connect()
            if i > 1:
                logger.warning("연결에 %d회 시도가 필요했습니다 — 모터 버스가 "
                               "불안정합니다. 배선을 확인하십시오.", i)
            return
        except Exception as exc:                      # noqa: BLE001
            last = exc
            if i < attempts:
                logger.warning("연결 실패(%d/%d), %.1f초 후 재시도: %s",
                               i, attempts, pause_s, str(exc).splitlines()[0])
                time.sleep(pause_s)
    raise last


class BaseArm:
    """작업(job) 모델과 인터럽트. 구동 방식과 무관한 부분만 여기 둔다."""

    def __init__(self, fps: int = 30, trace_dir: str | None = None,
                 home_after: bool = False):
        self.fps = fps
        # 궤적 기록 디렉터리. None 이면 기록하지 않는다. 종료 판정과 홈
        # 자세를 측정으로 정하기 위한 것이라, 실기 세션에서는 켜 두는 편이
        # 낫다 — 에피소드당 43KB 뿐이고, 다시 돌릴 기회는 비싸다.
        self.trace_dir = Path(trace_dir) if trace_dir else None
        self.traces: list[str] = []
        # 작업이 끝나면 홈으로 데려다 놓을지. 홈 값이 없으면 경고만 남는다.
        self.home_after = home_after
        self.lock = threading.Lock()
        self.busy = False
        self.job: dict | None = None
        self.last: dict | None = None
        self._stop: str | None = None       # None | 'afterCurrent' | 'immediate'
        self._seq = 0
        self._frames: dict[str, object] = {}
        self._frame_t = 0.0
        self._frame_lock = threading.Lock()

    def preflight(self, basket: str) -> None:
        """작업 시작 직전 점검. 문제가 있으면 ValueError 를 올린다.

        바구니를 **인자로 받는다.** self.job 에서 읽으면 안 된다 — 이 시점에는
        아직 job 이 만들어지기 전이라 None 이거나, 더 나쁘게는 직전 작업의
        바구니가 남아 있다. 그러면 엉뚱한 바구니 기준으로 조용히 검사한다
        (2026-08-21 통합 시험에서 KeyError 로 드러났다).

        구현체가 채운다. 기본은 아무것도 하지 않는다 — 가짜 팔에는
        점검할 하드웨어가 없다.
        """

    def _trace_writer(self, basket: str) -> TraceWriter | None:
        """이번 에피소드용 궤적 기록기. --trace-dir 가 없으면 None."""
        if self.trace_dir is None:
            return None
        j = self.job or {}
        return TraceWriter(dir=self.trace_dir, job_id=str(j.get("jobId", "j0")),
                           index=int(j.get("attempt", 1)), basket=basket,
                           fps=self.fps)

    def _trace_close(self, w: TraceWriter | None, out: dict) -> None:
        if w is None:
            return
        path = w.close({k: out[k] for k in
                        ("success", "finished", "aborted", "reason", "seconds",
                         "observeOnly", "mock", "task")
                        if k in out})
        if path is not None:
            self.traces.append(str(path))
            out["trace"] = str(path)
            logger.info("궤적 기록: %s", path)

    # ── 한 개 담기 — 구현체가 채운다 ──────────────────────────────
    def run_episode(self, basket: str, timeout_s: float) -> dict:
        raise NotImplementedError

    def go_home(self, seconds: float = 3.0) -> dict:
        raise NotImplementedError

    def health(self) -> dict:
        raise NotImplementedError

    def close(self) -> None:
        pass

    # ── 작업 ─────────────────────────────────────────────────────
    def start_job(self, basket: str, device_code: str, max_attempts: int = 3,
                  order_id: int = 0, timeout_s: float = 90.0) -> dict:
        """포장 작업을 시작하고 즉시 반환한다. 실제 동작은 워커 스레드가 한다.

        **작업의 단위는 "이 적재함을 비워라" 다.** 몇 개를 담았는지는 묻지
        않는다 — 포장 워크플로우가 요구하는 것은 적재함이 비는 것뿐이다.
        픽업의 quantity(집을 개수) 모델을 그대로 옮겼다가 없는 요구사항을
        만들었고, 2026-08-21 에 바로잡았다.

        max_attempts 는 재시도 횟수다. 한 에피소드가 끝났는데 적재함이 아직
        비지 않았으면 다시 돌린다. 정책이 한 번에 다 옮기지 못하는 일이
        흔하기 때문이다(실측 5/9). 다 쓰고도 안 비면 FAILED 로 알린다.

        비동기인 이유는 픽업과 같다 — 수 분이 걸리고, 그동안 관제가 진행
        상황을 못 보고 인터럽트도 못 건다.
        """
        n = int(max_attempts)
        if n < 1:
            raise ValueError(f"maxAttempts 는 1 이상이어야 합니다: {max_attempts}")

        # 팔이 움직이기 전에 마지막으로 확인한다. 여기서 막으면 HTTP 가
        # 400 으로 거절하므로 관제가 이유를 그대로 받는다. 작업을 띄운 뒤
        # 실패시키면 관제는 "시작은 됐는데 실패했다" 로만 알게 된다.
        self.preflight(basket)

        self._seq += 1
        self._stop = None
        self.job = {
            "jobId": f"p{self._seq}",
            "orderId": int(order_id),
            "deviceCode": device_code,
            "basket": basket,
            "maxAttempts": n,
            "attempt": 1,
            "boxEmpty": None,          # 아직 확인 전
            "state": "RUNNING",
            "startedAt": time.time(),
            "results": [],
            "message": "",
        }
        # busy 는 **스레드를 띄우기 전에** 세운다.
        #
        # 워커 안에서 세우면 202 를 반환한 뒤 스레드가 실제로 도는 사이에
        # 틈이 생긴다. 그 틈에 /health 가 들어오면 모터 버스를 읽어 제어
        # 루프와 충돌한다(2026-08-21 통합 시험에서 이 충돌로 작업 하나가
        # 통째로 실패했다). 틈은 1ms 남짓이지만 관제는 0.5초마다 폴링하므로
        # 언젠가는 맞는다.
        self.busy = True
        threading.Thread(target=self._run_job, args=(basket, timeout_s),
                         daemon=True).start()
        return self.state()

    def job_complete(self) -> tuple[bool | None, str]:
        """작업이 끝났는지 확인한다. (끝났는가, 설명).

        None 은 "확인할 수 없었다" 는 뜻이다 — 모른다는 것과 안 끝났다는
        것은 다르게 다뤄야 한다. 구현체가 채운다.
        """
        return True, ""

    def _run_job(self, basket: str, timeout_s: float) -> None:
        job = self.job
        assert job is not None
        try:
            self.busy = True
            for i in range(1, job["maxAttempts"] + 1):
                if self._stop:
                    job["state"] = "ABORTED"
                    job["message"] = f"운영자 정지({self._stop})"
                    break
                job["attempt"] = i
                r = self.run_episode(basket, timeout_s)
                r["index"] = i
                job["results"].append(r)

                if r.get("aborted"):
                    job["state"] = "ABORTED"
                    job["message"] = "운영자 즉시 정지"
                    break

                # 에피소드가 끝났다고 작업이 끝난 것은 아니다. 적재함을
                # 확인해서 비었을 때만 완료로 본다. 에피소드는 시간 초과로도
                # 끝나므로 "돌다가 멈췄다" 와 "다 비웠다" 는 다른 일이다.
                empty, why = self.job_complete()
                job["boxEmpty"] = empty
                r["boxEmpty"] = empty

                if empty:
                    job["state"] = "DONE"
                    job["message"] = why or "적재함을 비웠습니다"
                    break

                if empty is None:
                    # 확인 자체가 안 됐다. "모른다" 와 "안 끝났다" 는 다르므로
                    # 비었다고 단정하지 않는다 — 남은 시도가 있으면 계속한다.
                    logger.warning("적재함 상태를 확인하지 못했습니다: %s", why)

                if i < job["maxAttempts"]:
                    logger.info("적재함이 아직 비지 않았습니다 (%s) — "
                                "%d/%d 번째 시도", why, i + 1, job["maxAttempts"])
                    continue

                job["state"] = "FAILED"
                job["message"] = (f"{job['maxAttempts']}회 시도했지만 적재함을 "
                                  f"비우지 못했습니다 ({why})")

            # 작업이 끝나면 팔을 대기 자세로 데려다 놓는다.
            #
            # 정책은 멈춘 자리에 그대로 선다. 그 자리가 적재함 위면 탑뷰를
            # 가려 다음 판정을 방해하고, 사람이 물건을 채워 넣기도 불편하다.
            #
            # 홈 복귀는 **있으면 좋은 것이지 작업 결과를 좌우하지 않는다.**
            # 실패해도 그때까지의 결과(DONE/FAILED/ABORTED)를 덮어쓰지 않는다 —
            # 운영자가 멈춘 것과 작업이 실패한 것은 관제에게 다른 의미다.
            if self.home_after or self._stop == "immediate":
                try:
                    r = self.go_home()
                    logger.info("홈 복귀 완료 (오차 최대 %.2f도)",
                                r.get("maxErrorDeg", float("nan")))
                except NotImplementedError as e:
                    job["message"] += f" (홈 복귀 안 함: {e})"
                    logger.warning("홈 복귀를 건너뜁니다: %s", e)
                except Exception as e:                # noqa: BLE001
                    job["message"] += f" (홈 복귀 실패: {e})"
                    logger.warning("홈 복귀 실패: %s", e)
        except Exception as exc:                      # noqa: BLE001
            logger.exception("작업 실패")
            job["state"] = "FAILED"
            job["message"] = str(exc)
        finally:
            job["finishedAt"] = time.time()
            self.busy = False
            self._stop = None

    def request_stop(self, mode: str) -> dict:
        """mode='afterCurrent' 현재 1개 끝내고 정지 · 'immediate' 즉시 정지."""
        if mode not in ("afterCurrent", "immediate"):
            raise ValueError("mode 는 afterCurrent 또는 immediate 여야 합니다")
        if not self.busy:
            return {"success": True, "status": "IDLE",
                    "message": "진행 중인 작업이 없습니다"}
        self._stop = mode
        return {"success": True, "status": "STOPPING", "mode": mode,
                "message": ("현재 담기를 끝내고 정지합니다" if mode == "afterCurrent"
                            else "즉시 정지 후 홈으로 복귀합니다")}

    def state(self) -> dict:
        """관제가 폴링하는 진행 상태. boxEmpty 가 작업 완료 여부다."""
        if self.job is None:
            return {"success": True, "status": "IDLE", "busy": False}
        j = self.job
        out = {
            "success": j["state"] in ("RUNNING", "DONE"),
            "status": j["state"],
            "busy": bool(self.busy),
            "jobId": j["jobId"], "orderId": j["orderId"],
            "deviceCode": j["deviceCode"], "basket": j["basket"],
            # 관제가 봐야 할 것은 "적재함이 비었는가" 하나다. null 이면
            # 아직 확인 전이거나 확인하지 못했다는 뜻이다.
            "boxEmpty": j["boxEmpty"],
            "attempt": j["attempt"], "maxAttempts": j["maxAttempts"],
            "elapsedSec": round(time.time() - j["startedAt"], 1),
            "results": j["results"],
            "message": j["message"],
        }
        if self._stop:
            out["stopRequested"] = self._stop
        if self.traces:
            out["traces"] = list(self.traces)
        return out

    # ── 화면 ─────────────────────────────────────────────────────
    def _cache_frames(self, obs: dict) -> None:
        with self._frame_lock:
            for k in ("front", "wrist"):
                if k in obs:
                    self._frames[k] = obs[k]
            self._frame_t = time.time()

    def get_frame(self, cam: str = "front", max_age_s: float = 0.5):
        raise NotImplementedError

    @staticmethod
    def encode_jpeg(frame, quality: int = 80) -> bytes | None:
        import cv2

        if frame is None:
            return None
        bgr = cv2.cvtColor(np.asarray(frame), cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
        return buf.tobytes() if ok else None


class MockArm(BaseArm):
    """하드웨어 없이 도는 가짜 팔.

    관절 궤적까지 흉내 낸다. 궤적 기록·분석(trace.py)이 실제로 옳은 판단을
    내리는지 로봇 없이 확인하기 위해서다. 포장 정책이 담기를 끝낸 뒤 어떻게
    행동하는지 **모르는 것이 지금의 핵심 미지수**이므로, 가능한 세 가지를
    전부 흉내 낼 수 있게 했다:

        return-home   담고 나서 시작 자세로 돌아와 멈춘다 (픽업과 같은 형태)
        stop          담은 자리에서 그대로 멈춘다
        keep-moving   끝나도 계속 휘젓는다 (정책이 종료를 모르는 경우)

    실기에서 어느 쪽인지 확인되면, 분석기가 그 경우를 제대로 읽어내는지
    여기서 미리 시험해 둔 셈이 된다.
    """

    # 흉내 낼 홈 자세. 픽업 값이 아니라 포장 ACT 학습 통계의 중앙값 근처로
    # 잡았다(YELLOW observation.state.q50). 진짜 값이 아니라 형태만 맞춘
    # 것이므로 여기서 읽어다 쓰면 안 된다.
    FAKE_HOME = np.array([4.13, -15.50, 5.54, 27.77, 13.35, 55.43], np.float32)

    def __init__(self, fps: int = 30, episode_sec: float = 4.0,
                 behavior: str = "return-home", trace_dir: str | None = None,
                 home_after: bool = False):
        super().__init__(fps=fps, trace_dir=trace_dir, home_after=home_after)
        self.episode_sec = episode_sec
        self.behavior = behavior
        self._t = 0.0
        self._rng = np.random.default_rng(0)

    def _fake_state(self, phase: float) -> np.ndarray:
        """phase 0..1 동안의 관절 자세.

        0.0~0.7  집으러 갔다가 바구니로 옮긴다 (크게 움직임)
        0.7~1.0  behavior 에 따라 갈린다
        """
        home = self.FAKE_HOME
        if phase < 0.7:
            # 부드러운 왕복 — 실제 궤적처럼 프레임 간 변화가 연속이도록
            k = np.sin(phase / 0.7 * np.pi)
            swing = np.array([30.0, 25.0, -40.0, 15.0, -10.0, -6.0], np.float32)
            return home + swing * k
        if self.behavior == "keep-moving":
            k = np.sin((phase - 0.7) / 0.3 * 2 * np.pi)
            swing = np.array([12.0, 8.0, -15.0, 6.0, -4.0, -2.0], np.float32)
            return home + swing * k
        if self.behavior == "stop":
            # 담은 자리에 그대로 정지 — 시작 자세와 다른 곳이다
            return home + np.array([18.0, 12.0, -22.0, 9.0, -6.0, -3.0], np.float32)
        # return-home: 0.7~0.85 에 복귀하고 이후 정지
        if phase < 0.85:
            k = 1.0 - (phase - 0.7) / 0.15
            swing = np.array([18.0, 12.0, -22.0, 9.0, -6.0, -3.0], np.float32)
            return home + swing * k
        return home.copy()

    def run_episode(self, basket: str, timeout_s: float) -> dict:
        n = int(self.episode_sec * self.fps)
        tw = self._trace_writer(basket)
        aborted = False
        i = 0
        # 에피소드마다 시작 자세가 아주 조금 달라진다 — 실제 팔도 정확히
        # 같은 값으로 돌아오지 않는다. 분석기의 편차 계산이 0 이 아니어야
        # 의미 있는 시험이 된다.
        jitter = self._rng.normal(0.0, 0.35, 6).astype(np.float32)
        for i in range(n):
            if self._stop == "immediate":
                aborted = True
                break
            self._t += 1.0 / self.fps
            state = self._fake_state(i / max(n - 1, 1)) + jitter
            if tw is not None:
                tw.append(state)
            self._cache_frames({"front": self._fake_frame("front"),
                                "wrist": self._fake_frame("wrist")})
            time.sleep(1.0 / self.fps)
        out = {
            "basket": basket,
            "success": not aborted,
            "finished": not aborted,
            "seconds": round(min(i + 1, n) / self.fps, 2),
            "aborted": aborted,
            "reason": "운영자 즉시 정지" if aborted else
                      f"가짜 팔 — 정상 종료 ({self.behavior})",
            "mock": True,
        }
        self._trace_close(tw, out)

        self.last = out
        return out

    def _fake_frame(self, cam: str):
        """가짜 화면. 카메라가 없다는 것과 지금 상태를 글자로 알린다.

        처음에는 사인파 무늬만 내보냈는데, 화면만 보고는 "카메라가 고장난
        것인지 가짜인지" 구분할 수 없었다. 스트리밍 경로가 살아 있다는
        것만 확인시켜 주면 되므로, 움직이는 요소는 남기되 무엇을 보고
        있는지는 글자로 밝힌다.
        """
        import cv2

        h, w = 480, 640
        img = np.full((h, w, 3), 18, np.uint8)

        # 살아 있음을 보이는 스캔선 하나. 프레임이 갱신되는지 눈으로 확인된다.
        x = int((self._t * 120) % w)
        cv2.line(img, (x, 0), (x, h), (40, 70, 55), 2)
        cv2.rectangle(img, (0, 0), (w - 1, h - 1), (60, 80, 100), 2)

        j = self.job or {}
        # cv2.putText 는 한글을 못 그린다(물음표로 나온다). 화면 글자는
        # 전부 ASCII 로 둔다 — 한글을 쓰려면 PIL 로 폰트를 얹어야 하는데,
        # 가짜 화면 하나 때문에 의존성을 늘릴 이유가 없다.
        lines = [
            ("MOCK - NO CAMERA", 0.95, (90, 170, 255)),
            (f"cam={cam}", 0.70, (200, 220, 235)),
            (f"behavior={self.behavior}", 0.55, (150, 175, 195)),
            (f"{j.get('basket','-')}  시도 {j.get('attempt',0)}"
             f"/{j.get('maxAttempts',0)}" if j else "idle", 0.70,
             (120, 230, 160)),
            (f"t={self._t:5.1f}s", 0.55, (150, 175, 195)),
        ]
        y = 90
        for text, scale, color in lines:
            cv2.putText(img, text, (28, y), cv2.FONT_HERSHEY_SIMPLEX,
                        scale, color, 2, cv2.LINE_AA)
            y += int(46 * scale) + 26

        # 실제 카메라가 붙으면 이 자리에 진짜 화면이 온다는 안내
        cv2.putText(img, "real camera replaces this when connected",
                    (28, h - 32), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (110, 125, 140), 1, cv2.LINE_AA)
        # OpenCV 는 BGR, 나머지 경로는 RGB 다. 여기서 뒤집지 않으면
        # encode_jpeg 가 다시 뒤집어 색이 반대로 나온다.
        return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    def get_frame(self, cam: str = "front", max_age_s: float = 0.5):
        with self._frame_lock:
            f = self._frames.get(cam)
        return f if f is not None else self._fake_frame(cam)

    def go_home(self, seconds: float = 3.0) -> dict:
        time.sleep(min(seconds, 0.5))
        return {"ok": True, "mock": True}

    def health(self) -> dict:
        return {"success": True, "status": "OK", "robotConnected": False,
                "busy": bool(self.busy), "mock": True, "behavior": self.behavior,
                "message": "가짜 팔입니다. 하드웨어에 연결되어 있지 않습니다."}


class PackArm(BaseArm):
    """실제 포장 팔 + ACT 정책.

    ⚠ 아직 하드웨어에서 검증되지 않았다(2026-08-21). 팔·카메라 미연결 상태에서
    작성했다. 검증 전까지는 --mock 으로 돌릴 것.

    미확정 두 가지:
      · 종료 판정 — finish.py 참조. 기본값은 시간 기준이다
      · lerobot 0.6.1 은 추론 경로가 0.4.4 와 다르다. record 에서 정책 실행이
        빠지고 lerobot.rollout 으로 옮겨졌다. 팀원의 rollout 명령줄을 받으면
        이 클래스를 그쪽에 맞추는 편이 안전하다
    """

    def __init__(self, robot_port: str, robot_id: str,
                 front_device: str, wrist_device: str,
                 baskets: list[str] | None = None, fps: int = 30,
                 finish: str = "duration", finish_sec: float = 60.0,
                 trace_dir: str | None = None, observe_only: bool = False,
                 strict_start: bool = False, home_after: bool = False):
        super().__init__(fps=fps, trace_dir=trace_dir, home_after=home_after)
        self.finish_kind = finish
        self.finish_sec = finish_sec
        self.strict_start = strict_start

        # **바구니 모델을 전부 올린다.**
        #
        # ACT 에는 언어 조건이 없어서 "민트 바구니에 담아라" 를 요청으로
        # 전달할 방법이 없다. 어느 모델을 올렸느냐가 곧 어느 바구니냐다.
        # 처음에는 기동할 때 하나만 올렸는데, 그러면 cart-2 요청을 409 로
        # 거절하게 되고 서버를 다시 띄워야 했다. 팔이 하나뿐이라 서버를 두 개
        # 띄우는 우회도 불가능하다 — 같은 시리얼 포트를 두 프로세스가 열 수
        # 없다. 그래서 둘 다 올리고 deviceCode 로 고른다.
        #
        # 비용은 무시할 만하다: 체크포인트가 각각 0.28 GB(16 GB 중)이고
        # 로드가 0.6초다. 전환 비용은 없다 — 올려둔 것 중에 고르기만 한다.
        self.baskets = list(baskets) if baskets else sorted(DEFAULT_CHECKPOINTS)

        # 적재함 판정기도 바구니 수만큼 만든다. YOLO 가중치는 한 번만 읽히고
        # ROI 만 다르므로 부담이 없다.
        self.box_checkers: dict[str, object] = {}
        if finish == "box-empty":
            from .boxcheck import BoxChecker, load_rois

            rois = load_rois()
            for box, roi in rois.items():
                self.box_checkers[box] = BoxChecker(roi=roi)
                logger.info("적재함 판정 ROI (%s): %s", box, roi)
        # 관측 전용 — 정책을 올리지 않고 팔에 명령도 보내지 않는다.
        # 첫 하드웨어 연결에서 쓴다. 연결·카메라 키·제어 주기·홈 자세를
        # 팔이 움직이지 않는 상태에서 전부 확인할 수 있다.
        self.observe_only = observe_only

        from lerobot.cameras.opencv import OpenCVCameraConfig
        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.policies.factory import get_policy_class, make_pre_post_processors
        from lerobot.robots.omx_follower import OmxFollowerConfig
        from lerobot.robots.utils import make_robot_from_config
        from lerobot.utils.device_utils import get_safe_torch_device

        # 카메라 키는 반드시 front / wrist 다 — ACT 체크포인트가 아는 이름이다.
        #
        # 픽업 정책은 학습 때 --rename_map 으로 front→camera1, wrist→camera2 로
        # 바꿔서 camera1/2 를 안다. 포장은 rename 없이 학습했으므로 원래 이름
        # 그대로다. 두 서버가 다른 이름을 쓰는 것은 실수가 아니라 각자 정책이
        # 아는 이름을 쓰는 것이다.
        #
        #   front = 포장 탑뷰   wrist = 포장 팔 손목 카메라
        # 순서를 바꾸면 정책이 두 시점을 뒤바꿔 받는다. 예외는 나지 않는다.
        #
        # 주석(YOLO)은 그리지 않는다. ACT 는 원본 480x640 프레임으로 학습했다.
        cams = {
            "front": OpenCVCameraConfig(index_or_path=front_device, width=640,
                                        height=480, fps=fps, fourcc="MJPG",
                                        warmup_s=2),
            "wrist": OpenCVCameraConfig(index_or_path=wrist_device, width=640,
                                        height=480, fps=fps, fourcc="MJPG"),
        }
        self.robot = make_robot_from_config(
            OmxFollowerConfig(port=robot_port, id=robot_id, cameras=cams))

        # 학습 분포는 정책과 별개로 읽어 둔다. 관측 전용이라 정책을 안
        # 올리는 경우에도 자세 점검은 할 수 있어야 한다. 바구니마다 통계가
        # 다르므로 따로 보관한다.
        self.ranges: dict[str, StateRange] = {}
        for b in self.baskets:
            try:
                self.ranges[b] = StateRange(resolve_checkpoint(b))
            except Exception as exc:                  # noqa: BLE001
                logger.warning("%s 학습 분포를 읽지 못했습니다 — 시작 자세 "
                               "점검이 꺼집니다: %s", b, exc)

        self.policies: dict[str, dict] = {}
        if observe_only:
            _connect_with_retry(self.robot)
            from lerobot.processor.factory import make_default_processors

            (self._teleop_proc, self._act_proc,
             self._obs_proc) = make_default_processors()
            self._features = self._build_features()
            logger.info("관측 전용으로 연결했습니다 — 팔에 명령을 보내지 않습니다.")
            return

        for b in self.baskets:
            path = resolve_checkpoint(b)
            logger.info("정책 로드 중 (%s 바구니): %s", b, path)
            cfg = PreTrainedConfig.from_pretrained(path)
            cfg.pretrained_path = path
            policy = get_policy_class(cfg.type).from_pretrained(
                path, config=cfg).eval()
            pre, post = make_pre_post_processors(
                policy_cfg=cfg, pretrained_path=path,
                preprocessor_overrides={"device_processor": {"device": cfg.device}})
            self.policies[b] = {"policy": policy, "pre": pre, "post": post,
                                "cfg": cfg,
                                "device": get_safe_torch_device(cfg.device)}

        _connect_with_retry(self.robot)

        from lerobot.processor.factory import make_default_processors

        (self._teleop_proc, self._act_proc,
         self._obs_proc) = make_default_processors()
        self._features = self._build_features()
        logger.info("로봇 연결 완료. 준비됨 (바구니 %s).",
                    ", ".join(self.baskets))

    def _build_features(self) -> dict:
        # 0.6.1 에서 build_dataset_frame 이 lerobot.utils.feature_utils 로
        # 옮겨졌다. combine_feature_dicts 도 같이 옮겨졌다. 0.4.4 의
        # lerobot.datasets.utils 를 그대로 쓰면 ImportError 다.
        from lerobot.datasets.pipeline_features import (
            aggregate_pipeline_dataset_features, create_initial_features)
        from lerobot.utils.feature_utils import combine_feature_dicts

        return combine_feature_dicts(
            aggregate_pipeline_dataset_features(
                pipeline=self._teleop_proc,
                initial_features=create_initial_features(
                    action=self.robot.action_features),
                use_videos=True),
            aggregate_pipeline_dataset_features(
                pipeline=self._obs_proc,
                initial_features=create_initial_features(
                    observation=self.robot.observation_features),
                use_videos=True),
        )

    def _state_vec(self, obs: dict) -> np.ndarray:
        return np.array([obs[f"{m}.pos"] for m in JOINTS], dtype=np.float32)

    def run_episode(self, basket: str, timeout_s: float) -> dict:
        from lerobot.utils.feature_utils import build_dataset_frame
        from lerobot.policies.utils import (make_robot_action,
                                            prepare_observation_for_inference)
        import torch

        # 이번 작업의 바구니에 해당하는 정책을 고른다. 잘못 고르면 팔이
        # 엉뚱한 바구니로 간다 — 예외도 경고도 없이.
        pol = None
        if not self.observe_only:
            if basket not in self.policies:
                raise ValueError(
                    f"{basket} 바구니 모델이 올라와 있지 않습니다 "
                    f"(올린 것: {', '.join(sorted(self.policies))})")
            pol = self.policies[basket]
            pol["policy"].reset()
        det = make_detector(self.finish_kind, fps=self.fps, seconds=self.finish_sec,
                            checker=self._checker_for_job(),
                            frame_fn=lambda: self.get_frame("front"))
        det.reset()
        tw = self._trace_writer(basket)

        period = 1.0 / self.fps
        t0 = time.perf_counter()
        finished = aborted = False
        slow = 0
        n = 0

        while time.perf_counter() - t0 < timeout_s:
            if self._stop == "immediate":
                aborted = True
                break
            tick = time.perf_counter()
            obs = self.robot.get_observation()
            s = self._state_vec(obs)
            n += 1

            self._cache_frames(obs)
            if self.observe_only:
                # 팔은 그대로 두고 관측만 읽는다. 명령이 없으므로 궤적에는
                # 상태만 남는다.
                if tw is not None:
                    tw.append(s)
                left = period - (time.perf_counter() - tick)
                if left < 0:
                    slow += 1
                time.sleep(max(0.0, left))
                if det.update(s):
                    finished = True
                    break
                continue

            frame = build_dataset_frame(self._features, self._obs_proc(obs),
                                        prefix="observation")

            # prepare_observation_for_inference 를 빼먹으면 안 된다.
            #
            # build_dataset_frame 이 내놓는 이미지는 uint8 HWC 다. 정규화기는
            # 통계를 **입력 텐서의 dtype 으로** 캐스팅하므로, uint8 을 그대로
            # 넘기면 평균 0.485 같은 값을 uint8 로 바꾸려다 죽는다:
            #     RuntimeError: value cannot be converted to type uint8
            #                   without overflow
            # 이 함수가 CHW · float32 [0,1] 로 바꾸고 배치 차원과 장치 이동까지
            # 한다. 0.4.4 에서는 predict_action 안에 들어 있어서 보이지 않았다.
            #
            # ACT 는 언어 조건이 없어 이 문자열이 팔의 동작을 바꾸지는
            # 않는다 — 무엇을 어디에 담을지는 어느 체크포인트를 올렸느냐로만
            # 정해진다. 그래도 넘기는 이유는 궤적·데이터셋에 남는 라벨이기
            # 때문이다 — 나중에 이 에피소드를 보거나 재사용할 때 무슨
            # 작업이었는지 알 수 있어야 한다. lerobot-rollout 으로 평가
            # 데이터를 남길 때 쓰는 --dataset.single_task 와 같은 문구로
            # 맞춘다.
            with torch.inference_mode():
                batch = prepare_observation_for_inference(
                    frame, pol["device"], TASK_LABEL, self.robot.robot_type)
                batch = pol["pre"](batch)
                action_values = pol["policy"].select_action(batch)
                action_values = pol["post"](action_values)
            act = make_robot_action(action_values.squeeze(0).cpu(),
                                    self._features)
            self.robot.send_action(self._act_proc((act, obs)))
            if tw is not None:
                # 관측과 **그 프레임의 명령**을 함께 남긴다. 리스트 append
                # 뿐이라 33ms 주기에 영향이 없다.
                tw.append(s, np.array([act.get(f"{j}.pos", np.nan)
                                       for j in JOINTS], np.float32))

            if det.update(s):
                finished = True
                break

            left = period - (time.perf_counter() - tick)
            if left < 0:
                slow += 1
            time.sleep(max(0.0, left))

        if slow:
            logger.warning("제어 주기를 %d/%d 프레임에서 놓쳤습니다 (%.0f%%).",
                           slow, n, 100 * slow / max(n, 1))

        out = {
            "basket": basket,
            # ⚠ 성공 판정이 없다. 픽업은 그리퍼 값과 도착 각도로 판정했지만
            # (omx_yolo.kinematic), 그 기준은 픽업 리그 전용이다. 포장은
            # 기준값을 아직 측정하지 못했으므로 "끝까지 돌았는가" 만 본다.
            # 실제로 바구니에 들어갔는지는 사람이 봐야 한다.
            "success": bool(finished and not aborted),
            "finished": bool(finished),
            "judged": False,
            "seconds": round(n / self.fps, 2),
            "aborted": bool(aborted),
            "reason": ("운영자 즉시 정지" if aborted else
                       getattr(det, "reason", "종료") if finished else
                       "시간 초과"),
            "observeOnly": bool(self.observe_only),
            "task": TASK_LABEL,
        }
        self._trace_close(tw, out)
        # 적재함을 보고 끊었다면 그때 본 화면을 남긴다. 판정이 틀렸을 때
        # "무엇을 보고 그렇게 판단했는가" 를 나중에 확인할 수 있어야 한다.
        frame = getattr(det, "last_frame", None)
        if frame is not None and self.trace_dir is not None:
            try:
                import cv2

                self.trace_dir.mkdir(parents=True, exist_ok=True)
                name = Path(out.get("trace", "") or f"{time.strftime('%H%M%S')}.npz")
                dst = self.trace_dir / (name.stem + "_boxview.jpg")
                cv2.imwrite(str(dst), cv2.cvtColor(np.asarray(frame),
                                                   cv2.COLOR_RGB2BGR))
                out["boxView"] = str(dst)
                logger.info("판정 근거 화면: %s", dst)
            except Exception as exc:                  # noqa: BLE001
                logger.warning("판정 화면을 저장하지 못했습니다: %s", exc)
        if hasattr(det, "looks"):
            out["boxLooks"] = int(det.looks)
        self.last = out
        return out

    def _checker_for_job(self):
        """이번 작업의 deviceCode 에 해당하는 적재함 판정기.

        cart-1 → box1, cart-2 → box2. 바구니 모델과 적재함 ROI 는 **함께**
        움직여야 한다 — 노랑 모델을 돌리면서 box2 를 보면 엉뚱한 상자가
        비었는지 묻게 된다.
        """
        if not self.box_checkers:
            return None
        from .boxcheck import resolve_box

        device = (self.job or {}).get("deviceCode", "")
        box = resolve_box(device)
        if box not in self.box_checkers:
            logger.warning("적재함 ROI 를 찾지 못했습니다: %r → %r", device, box)
            return None
        return self.box_checkers[box]

    def job_complete(self) -> tuple[bool | None, str]:
        """적재함이 비었는지 확인한다 — 이것이 포장 작업의 완료 조건이다.

        에피소드가 끝난 직후에는 팔이 적재함 위에 있을 수 있다. 가려진
        화면으로 판정하면 물건이 있는데도 "비었음" 이 나오므로(2026-08-21
        실측: 40프레임 중 16장), 보일 때까지 잠깐 기다렸다가 본다.

        끝내 못 보면 None 을 돌려준다. **"모른다" 를 "안 비었다" 로 바꾸지
        않는다** — 둘은 다른 일이고, 관제도 다르게 다뤄야 한다.
        """
        checker = self._checker_for_job()
        if checker is None:
            return True, ""            # 적재함 판정을 안 쓰는 설정
        from .boxcheck import roi_is_visible

        deadline = time.time() + 6.0
        last_dark = 1.0
        while time.time() < deadline:
            frame = self.get_frame("front", max_age_s=0.3)
            if frame is not None:
                visible, last_dark = roi_is_visible(frame, checker.roi)
                if visible:
                    empty, det = checker.is_empty(frame)
                    if empty:
                        return True, "적재함이 비어 보입니다"
                    return False, f"물건 {len(det)}개가 남아 있습니다"
            time.sleep(0.3)
        return None, (f"적재함을 볼 수 없었습니다 (팔이 {last_dark*100:.0f}% 가림)")

    def start_pose_check(self, basket: str | None = None) -> dict | None:
        """지금 팔 자세가 학습 분포 안인지 본다.

        ⚠ **작업 중에는 절대 부르면 안 된다.** 모터 버스를 읽는데,
        Dynamixel 포트 핸들러는 스레드 안전하지 않다. 제어 루프가 30Hz 로
        Goal_Position 을 쓰는 동안 HTTP 스레드에서 읽으면 이렇게 죽는다:

            Failed to sync write 'Goal_Position' ... [TxRxResult] Port is in use!

        2026-08-21 통합 시험 중 실제로 발생했다. 관제가 is_reachable() 로
        /health 를 폴링하는 순간 진행 중이던 포장 작업이 통째로 실패했다.
        부르는 쪽(health)에서 busy 를 확인한다.
        """
        if not self.ranges:
            return None
        obs = self.robot.get_observation()
        state = self._state_vec(obs)
        if basket is not None:
            rng = self.ranges.get(basket)
            return rng.summary(state) if rng else None
        # 어느 바구니로 갈지 모르는 상황(대기 중 /health)에서는 전부 본다.
        # 바구니마다 학습 분포가 다르므로 하나로 합칠 수 없다.
        return {b: r.summary(state) for b, r in self.ranges.items()}

    def preflight(self, basket: str) -> None:
        # 이번 작업의 바구니 분포로 본다. 바구니마다 학습 범위가 다르므로
        # 아무 것이나 쓰면 통과·거절이 뒤바뀔 수 있다.
        chk = self.start_pose_check(basket)
        if not chk or "grade" not in chk:
            # 해당 바구니의 분포를 못 읽은 경우다. 점검을 못 했다고 해서
            # 작업을 막지는 않는다 — 막을 근거가 없다.
            if chk:
                logger.warning("%s 바구니 분포로 시작 자세를 볼 수 없습니다", basket)
            return
        if chk["grade"] == OUT:
            msg = ("시작 자세가 학습 범위 밖입니다: "
                   + " / ".join(chk["messages"]))
            if self.strict_start:
                # 거절한다. 정책은 학습 밖 상태에서도 자신 있게 움직이므로,
                # 사람이 보기 전에 팔이 먼저 가는 것을 막는다.
                raise ValueError(
                    msg + " — 팔을 학습 범위 안의 자세로 두고 다시 요청하십시오. "
                          "(--strict-start 로 켜진 검사입니다)")
            logger.warning("%s", msg)
            logger.warning("그대로 진행합니다. 막으려면 --strict-start 를 주십시오.")
        elif chk["grade"] != "OK":
            logger.info("시작 자세가 학습 분포의 가장자리에 있습니다: %s",
                        " / ".join(chk["messages"]))

    def get_frame(self, cam: str = "front", max_age_s: float = 0.5):
        with self._frame_lock:
            fresh = (time.time() - self._frame_t) < max_age_s
            f = self._frames.get(cam)
        if fresh and f is not None:
            return f
        c = self.robot.cameras.get(cam)
        if c is None:
            return None
        try:
            f = c.read_latest()
            with self._frame_lock:
                self._frames[cam] = f
                self._frame_t = time.time()
            return f
        except Exception:                             # noqa: BLE001
            return f

    def go_home(self, seconds: float = 3.0) -> dict:
        """저장된 홈 자세로 천천히 복귀한다.

        포장 정책은 스스로 홈으로 가지 않으므로 여기서 데려다 놓는다.
        홈 값이 없으면 NotImplementedError 를 낸다 — **픽업 값으로 대체하지
        않는다.** 다른 팔이고 다른 자리라 엉뚱한 데로 간다.
        """
        from .home import HOME_PATH, interpolate_home, load_home

        home = load_home()
        if home is None:
            raise NotImplementedError(
                f"포장 팔의 홈 자세가 없습니다 ({HOME_PATH}). "
                "팔을 대기 자세에 두고 `python -m omx_pack.home --capture` "
                "로 기록하십시오.")
        return interpolate_home(self.robot, home, seconds=seconds, fps=self.fps)

    def health(self) -> dict:
        out = {"success": True, "status": "OK", "message": "",
               "robotConnected": bool(self.robot.is_connected),
               "busy": bool(self.busy),
               "baskets": list(self.baskets),
               "boxes": sorted(self.box_checkers),
               "finishMode": self.finish_kind,
               "observeOnly": bool(self.observe_only)}
        # 작업 중에는 모터 버스를 건드리지 않는다. /health 는 관제가 수시로
        # 폴링하는 엔드포인트라, 여기서 버스를 읽으면 작업이 깨진다.
        if self.busy:
            chk = {"grade": "SKIPPED",
                   "note": "작업 중에는 자세를 읽지 않습니다 (모터 버스 충돌 방지)"}
            out["startPose"] = chk
        else:
            try:
                chk = self.start_pose_check()
            except Exception as exc:                  # noqa: BLE001
                chk = {"grade": "UNKNOWN", "error": str(exc)}
            if chk is not None:
                out["startPose"] = chk
                # 바구니별 결과다. 하나라도 범위 밖이면 알린다 — 그 바구니로
                # 요청이 오면 정책이 본 적 없는 상태에서 시작한다.
                bad = [b for b, v in chk.items()
                       if isinstance(v, dict) and v.get("grade") == OUT]
                if bad:
                    out["status"] = "DEGRADED"
                    msgs = []
                    for b in bad:
                        msgs += [f"{b}: {m}" for m in chk[b].get("messages", [])]
                    out["message"] = ("시작 자세가 학습 범위 밖입니다 — "
                                      + "; ".join(msgs))
        # 픽업의 /health 는 리그 기준 대조까지 한다(checkrig). 포장은 기준
        # 배치를 아직 촬영하지 못해 장치 연결만 본다.
        out["rig"] = {"error": "포장 리그 기준값이 아직 없습니다"}
        if not out["robotConnected"]:
            out["success"] = False
            out["status"] = "DEGRADED"
            out["message"] = "로봇이 연결되어 있지 않습니다"
        return out

    def close(self) -> None:
        try:
            self.robot.disconnect()
        except Exception:                             # noqa: BLE001
            pass
