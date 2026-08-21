"""에피소드 종료 판정.

**이 파일은 아직 확정되지 않았다.** 팀원의 포장 추론 스크립트를 받으면
그쪽 방식으로 교체해야 한다. 그때까지 서버가 무한정 도는 것을 막기 위한
잠정 구현이다.

왜 필요한가 — ACT 정책은 "끝났다" 를 알려주지 않는다. 액션 청크를 계속
내놓을 뿐이라, 밖에서 끊지 않으면 영원히 움직인다. 픽업에서도 같은 문제를
겪었고 홈 자세 복귀를 감지해서 끊었다(omx_yolo.success.HomeDetector).

포장에서는 그 방식을 그대로 못 쓴다. 픽업의 홈 자세
`[-1.04, -63.03, 54.08, 42.59, -4.14, 59.54]` 는 픽업 팔 전용 값이고,
포장 팔의 자세 분포는 학습 통계상 전혀 다르다(MINT 평균
`[7.49, -4.57, -4.71, 40.59, 17.61, 55.47]`). 포장 팔에서 다시 측정해야
하는데, 애초에 포장 데모가 홈으로 복귀하며 끝나는지도 확인되지 않았다.

그래서 판정기를 갈아 끼울 수 있게 분리해 둔다. 셋 중 하나를 고른다:

  DurationFinish  정해진 시간만 돌고 끊는다 (기본값 · 가장 안전)
  StallFinish     팔이 멈추면 끝난 것으로 본다 (추정 · 검증 필요)
  HomeFinish      홈 자세 복귀를 감지한다 (포장 홈 자세를 알아낸 뒤에만)

기본값이 DurationFinish 인 이유: 나머지 둘은 **틀렸을 때 조용히 틀린다.**
StallFinish 는 팔이 잠깐 뜸들이는 것을 종료로 오인해 물건을 반만 담고
성공이라 보고할 수 있다. 시간 기준은 최소한 틀린 방향이 예측 가능하다.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import logging

import numpy as np

logger = logging.getLogger(__name__)

# 학습 데이터에서 읽은 에피소드 길이(2026-08-21, 정규화 통계 기준).
#   YELLOW  최대 59.1초 / 1,773프레임    MINT  최대 59.8초 / 1,794프레임
#   평균 timestamp 24.0초 / 24.3초  →  에피소드 평균 길이는 그 두 배 근처
# 약 30fps 로 기록됐고, 가장 긴 에피소드가 60초에 붙어 있다. 데모를 60초에서
# 끊었을 가능성이 높다 — 그렇다면 그보다 길게 잡아 봐야 의미가 없다.
TRAIN_EPISODE_MAX_SEC = 60.0


@dataclass
class DurationFinish:
    """정해진 시간이 지나면 종료로 본다.

    판정을 하지 않는 것과 같다. 학습 에피소드가 60초를 넘지 않으므로 그
    이상 돌려 봐야 학습 분포 밖이다.
    """

    seconds: float = TRAIN_EPISODE_MAX_SEC
    fps: int = 30
    _n: int = field(default=0, init=False)

    def reset(self) -> None:
        self._n = 0

    def update(self, state: np.ndarray) -> bool:      # noqa: ARG002
        self._n += 1
        return self._n >= int(self.seconds * self.fps)

    @property
    def reason(self) -> str:
        return f"{self.seconds:.0f}초 경과 (시간 기준 종료)"


@dataclass
class StallFinish:
    """팔이 일정 시간 거의 움직이지 않으면 종료로 본다.

    **검증되지 않았다.** 포장 정책이 물건을 다 담은 뒤 정지하는지, 아니면
    계속 허공을 저으며 도는지 확인되지 않았다. 사용자 말로는 "박스에 물건이
    없으면 멈출 수도 있고 계속 동작할 수도 있다" 이므로, 실기에서 관찰한
    뒤에 쓸 것.

    오판의 방향이 나쁘다 — 정책이 물건을 놓기 전에 잠깐 뜸들이면 그것을
    종료로 읽고 성공이라 보고한다. 그래서 기본값이 아니다.
    """

    move_thresh: float = 0.5      # 관절 변화량 합(도). 이보다 작으면 정지로 본다
    hold_sec: float = 2.0         # 이만큼 계속 정지해 있어야 종료로 인정
    min_sec: float = 5.0          # 이보다 일찍 끝나는 것은 오판으로 본다
    fps: int = 30
    _prev: np.ndarray | None = field(default=None, init=False)
    _hold: int = field(default=0, init=False)
    _n: int = field(default=0, init=False)

    def reset(self) -> None:
        self._prev = None
        self._hold = 0
        self._n = 0

    def update(self, state: np.ndarray) -> bool:
        self._n += 1
        if self._prev is None:
            self._prev = state.copy()
            return False
        moved = float(np.abs(state - self._prev).sum())
        self._prev = state.copy()
        if moved < self.move_thresh:
            self._hold += 1
        else:
            self._hold = 0
        if self._n < int(self.min_sec * self.fps):
            return False
        return self._hold >= int(self.hold_sec * self.fps)

    @property
    def reason(self) -> str:
        return f"팔이 {self.hold_sec:.0f}초간 정지 (정지 기준 종료 · 미검증)"


@dataclass
class HomeFinish:
    """홈 자세 복귀를 감지한다 — 픽업(omx_yolo.success.HomeDetector)과 같은 방식.

    포장 팔의 홈 자세를 알아낸 뒤에만 쓸 수 있다. 기본값을 채워 두지 않은
    것은 의도적이다. 픽업 값을 그대로 넣어 두면 누군가 그게 측정된 값인 줄
    알고 쓰게 된다.
    """

    home: np.ndarray
    tol: np.ndarray
    hold_frames: int = 8          # 약 0.27초 @30fps
    min_frames: int = 90          # 3초 미만 종료는 오판으로 본다
    _hold: int = field(default=0, init=False)
    _n: int = field(default=0, init=False)

    def reset(self) -> None:
        self._hold = 0
        self._n = 0

    def update(self, state: np.ndarray) -> bool:
        self._n += 1
        if np.all(np.abs(state - self.home) < self.tol):
            self._hold += 1
        else:
            self._hold = 0
        return self._n >= self.min_frames and self._hold >= self.hold_frames

    @property
    def reason(self) -> str:
        return "홈 자세 복귀 감지"


@dataclass
class BoxEmptyFinish:
    """적재함이 비면 끝난 것으로 본다 — 포장의 자연스러운 종료 조건.

    정책은 스스로 멈추지 않는다(2026-08-21 실측: 다 비운 뒤에도 빈 공간과
    박스 테두리를 계속 집으려 든다). 시간으로 끊으면 다 비웠는데 계속 돌거나
    못 비웠는데 끊긴다. 적재함을 보고 끊는 것이 맞다.

    **가림을 피한다.** 팔이 적재함 위에 있으면 안이 안 보인다. 그 상태를
    "비었다" 로 읽으면 작업을 중간에 끊으므로, 팔이 비켜 있을 때만 본다.
    그리고 연속으로 confirm 번 같은 답이 나와야 받아들인다 — 한 프레임에서
    물건이 팔 그림자에 묻히는 일이 있다.

    검출을 매 프레임 돌리지는 않는다. 33ms 주기 안에서 여유가 있긴 하지만
    (YOLO n 모델이 5~6ms) 1초에 한 번이면 충분하고, 남는 시간은 제어에 준다.
    """

    checker: object                    # boxcheck.BoxChecker
    frame_fn: object                   # () -> RGB ndarray | None
    fps: int = 30
    confirm: int = 3                   # 연속 몇 번 비었다고 나와야 인정
    check_every_s: float = 1.0
    min_sec: float = 5.0               # 이보다 일찍 끝나는 것은 오판으로 본다
    blind_warn_s: float = 15.0         # 이만큼 못 보고 있으면 이유를 알린다
    _n: int = field(default=0, init=False)
    _next: int = field(default=0, init=False)
    _hits: int = field(default=0, init=False)
    _last: str = field(default="", init=False)
    _looks: int = field(default=0, init=False)
    _last_look: int = field(default=0, init=False)
    _warned: bool = field(default=False, init=False)
    last_frame: object = field(default=None, init=False)

    def reset(self) -> None:
        self._n = self._hits = self._looks = self._last_look = 0
        self._next = int(self.min_sec * self.fps)
        self._last = ""
        self._warned = False
        self.last_frame = None

    @property
    def looks(self) -> int:
        """실제로 적재함을 들여다본 횟수. 0 이면 판정 기회가 없었다는 뜻이다."""
        return self._looks

    def update(self, state: np.ndarray) -> bool:
        from .boxcheck import roi_is_visible

        self._n += 1
        if self._n < self._next:
            return False
        self._next = self._n + int(self.check_every_s * self.fps)

        frame = self.frame_fn()
        if frame is None:
            return False

        # 가림 확인이 먼저다. 가려진 화면으로 판정하면 물건이 있는데도
        # "비었음" 이 나온다(2026-08-21 실측: 40프레임 중 16장이 그랬다).
        # 회색조 평균 한 번이라 YOLO 를 돌리기 전에 값싸게 거른다.
        visible, dark = roi_is_visible(frame, self.checker.roi)
        if not visible:
            blind = (self._n - self._last_look) / self.fps
            if not self._warned and blind >= self.blind_warn_s:
                self._warned = True
                logger.warning(
                    "%.0f초째 적재함을 보지 못했습니다 — 팔이 계속 적재함을 "
                    "가리고 있습니다(가림 %.0f%%). 비었는데도 정책이 자리를 "
                    "비켜 주지 않으면 이런 상태가 이어집니다.", blind, dark * 100)
            return False

        self._last_look = self._n
        self._warned = False
        self._looks += 1
        empty, det = self.checker.is_empty(frame)
        self.last_frame = frame        # 마지막으로 본 화면 — 판정 근거로 남긴다
        if empty:
            self._hits += 1
            self._last = f"{self._hits}/{self.confirm} 회 연속 비어 보임"
        else:
            self._hits = 0
            self._last = f"물건 {len(det)}개 보임"
        logger.info("적재함 확인 %.1f초: %s (가림 %.0f%%)",
                    self._n / self.fps, self._last, dark * 100)
        return self._hits >= self.confirm

    @property
    def reason(self) -> str:
        return f"적재함이 비었습니다 ({self._last})"


def make_detector(kind: str, fps: int = 30, seconds: float = TRAIN_EPISODE_MAX_SEC,
                  checker=None, frame_fn=None):
    """이름으로 판정기를 만든다. 서버의 --finish 옵션이 쓴다."""
    kind = (kind or "duration").strip().lower()
    if kind == "duration":
        return DurationFinish(seconds=seconds, fps=fps)
    if kind == "stall":
        return StallFinish(fps=fps)
    if kind == "box-empty":
        if checker is None or frame_fn is None:
            raise ValueError("box-empty 판정에는 checker 와 frame_fn 이 필요합니다")
        return BoxEmptyFinish(checker=checker, frame_fn=frame_fn, fps=fps)
    if kind == "home":
        raise ValueError(
            "home 판정기는 포장 팔의 홈 자세를 먼저 측정해야 씁니다. "
            "팀원의 추론 스크립트를 받은 뒤 finish.HomeFinish 에 값을 넣으십시오.")
    raise ValueError(f"모르는 종료 판정 방식입니다: {kind!r} (duration | stall | box-empty | home)")
