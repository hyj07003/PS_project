"""픽업 시도의 완료·성공을 판정한다.

정책(ACT·SmolVLA)은 종료 신호를 학습하지 않는다. 액션 시퀀스만 계속 내놓는다.
따라서 "언제 끝났는지"와 "성공했는지"를 밖에서 만들어야 한다. 이 모듈이
그 두 가지를 담당하고, 두 곳에서 같이 쓰인다.

    1) 중앙 관제 서버 인터페이스 — 픽업 완료 시그널과 성공/실패 상태
    2) 기준선·A/B 측정 — 조건별 성공률 집계 (evaluate.py)

2계층 구조
────────────────────────────────────────────────────────────────────────
계층 1  동작 종료 감지 (HomeDetector)
    관절 상태만 본다. 학습이 필요 없다.
    홈에서 출발 → 홈을 떠남 → 홈으로 복귀해 K프레임 유지  →  "끝났다"

    상태 기계로 만든 이유: 에피소드 시작 시점에는 이미 홈에 있으므로,
    단순히 "홈 근처인가"만 보면 첫 프레임에 즉시 종료로 오판한다.

계층 2  성공 판정 (BoxCounter)
    홈 복귀는 "동작이 끝났다"만 알려주고 "물건이 들어갔다"는 보장하지 않는다.
    헛집기(grasp 실패)도 홈으로 돌아온다. 그래서 탑뷰에서 적재함 안의
    해당 상품 개수가 1 늘었는지 확인한다.
────────────────────────────────────────────────────────────────────────

홈 자세 기준값
    smart_market_prototype_2 (300 에피소드) 의 에피소드 시작 프레임 평균이다.
    prototype_1(177 에피소드) 과 관절별 최대 1.1 이내로 일치한다.

    ⚠ prototype_3 은 홈으로 복귀하지 않는다 (종료 시 shoulder_pan −49.7,
      시작↔종료 평균절대차 8.28). 그 데이터셋에는 이 판정기를 쓸 수 없다.
      prototype_1 / _2 / _4 는 0.22 / 0.29 / 0.46 으로 정상이다.

    리그를 조정했으면 다시 계산할 것:
        python -m omx_yolo.success --calibrate kdy93/smart_market_prototype_2
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper"]

# smart_market_v1 에피소드 시작 프레임 평균 (180 에피소드, 현재 리그·화각).
# 이 데이터셋의 시작↔종료 평균절대차는 0.11 로 지금까지 중 가장 깨끗하다.
HOME = np.array([-1.04, -63.03, 54.08, 42.59, -4.14, 59.54], dtype=np.float32)

# 관절별 허용 오차. 3σ 를 쓰고 최소 5.0 을 보장한다.
# smart_market_v1 의 σ = [2.38, 2.01, 0.54, 3.49, 3.47, 0.29]
#
# ⚠ 리그를 옮기면 반드시 재보정할 것. 옛 prototype_2 기준값을 그대로 쓰니
#   wrist_roll 이 2.86, shoulder_pan 이 1.96 어긋나 180개 중 42개가 "홈 복귀
#   없음" 으로 오판정되었다. 시연 자체는 전부 정상이었다.
#       python -m omx_yolo.success --calibrate <repo_id>
HOME_TOL = np.array([7.1, 6.0, 5.0, 10.5, 10.4, 5.0], dtype=np.float32)

# 홈을 "떠났다"고 인정하는 문턱. 허용 오차의 몇 배 이상 벗어나야 한다.
AWAY_FACTOR = 2.0


@dataclass
class HomeDetector:
    """홈 복귀로 동작 종료를 감지한다.

    사용
        det = HomeDetector()
        for state in stream:                     # state: (6,) 관절 위치
            if det.update(state):
                break                            # 동작 종료
        else:
            ...                                  # 타임아웃
    """

    home: np.ndarray = field(default_factory=lambda: HOME.copy())
    tol: np.ndarray = field(default_factory=lambda: HOME_TOL.copy())
    # 홈에 머물러야 하는 프레임 수. 0.5초(15프레임)는 너무 엄격했다 —
    # smart_market_v1 실측에서 '홈 복귀 없음' 21건 중 17건이 홈에 도착은
    # 했으나 곧바로 다음 동작을 준비하느라 15프레임을 못 채운 경우였다.
    # 최종 프레임이 실제로 홈을 벗어난 것은 4건뿐이었다.
    hold_frames: int = 8           # 약 0.27초 @30fps
    min_frames: int = 90           # 3초 미만 종료는 오판으로 본다
    _n: int = 0
    _left_home: bool = False
    _hold: int = 0

    def reset(self) -> None:
        self._n = 0
        self._left_home = False
        self._hold = 0

    def _at_home(self, s: np.ndarray) -> bool:
        return bool(np.all(np.abs(np.asarray(s, np.float32) - self.home) <= self.tol))

    def _far_from_home(self, s: np.ndarray) -> bool:
        d = np.abs(np.asarray(s, np.float32) - self.home)
        return bool(np.any(d > self.tol * AWAY_FACTOR))

    def update(self, state: np.ndarray) -> bool:
        """한 프레임 진행. 동작이 종료되면 True."""
        self._n += 1
        if not self._left_home:
            if self._far_from_home(state):
                self._left_home = True
            return False
        if self._at_home(state):
            self._hold += 1
        else:
            self._hold = 0
        return self._hold >= self.hold_frames and self._n >= self.min_frames

    @property
    def frames(self) -> int:
        return self._n

    @property
    def left_home(self) -> bool:
        return self._left_home


class BoxCounter:
    """탑뷰 프레임에서 적재함 안의 특정 상품 개수를 센다.

    Annotator 와 같은 검출 설정을 쓴다 — 주석과 판정이 어긋나면 안 된다.
    """

    def __init__(self, weights: str, conf: float | None = None,
                 iou: float | None = None, imgsz: int | None = None):
        import cv2  # noqa: F401  (지연 import)
        from ultralytics import YOLO

        from .annotate import DEFAULT_CONF, DEFAULT_IMGSZ, DEFAULT_IOU, KEEP_CLASSES

        self.model = YOLO(weights)
        self.conf = DEFAULT_CONF if conf is None else conf
        self.iou = DEFAULT_IOU if iou is None else iou
        self.imgsz = DEFAULT_IMGSZ if imgsz is None else imgsz
        self.keep = KEEP_CLASSES

    # 같은 물체의 중복 검출로 볼 중심 거리(픽셀).
    # 실측: 아이스크림 한 개가 x=285,286,288 에 3중 검출됨(간격 3px).
    # 서로 다른 실제 물체는 50px 이상 떨어져 있다(같은 칸 안에서도 y 간격 ~55px).
    DEDUP_DIST = 30.0

    def count(self, frame_rgb: np.ndarray, roi, cls: str) -> int:
        """roi 안의 cls 개수. 가까이 겹친 중복 검출은 하나로 센다.

        NMS 의 iou 를 0.90 으로 올려 인접한 동종 물체가 병합되는 것을 막았는데,
        그 대가로 같은 물체가 여러 번 검출된다. 공간 거리로 한 번 더 묶어
        두 문제를 동시에 피한다.
        """
        import cv2

        from .geometry import in_roi

        bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        res = self.model.predict(bgr, conf=self.conf, iou=self.iou,
                                 imgsz=self.imgsz, classes=self.keep,
                                 verbose=False)[0]
        centers = []
        for b in res.boxes:
            if res.names[int(b.cls)] != cls:
                continue
            x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
            if in_roi(roi, cx, cy):
                centers.append((cx, cy, float(b.conf)))

        # 신뢰도 높은 것부터 채택하고, 그 근처의 나머지는 중복으로 버린다
        centers.sort(key=lambda t: -t[2])
        kept: list[tuple[float, float]] = []
        for cx, cy, _ in centers:
            if all((cx - kx) ** 2 + (cy - ky) ** 2 > self.DEDUP_DIST ** 2
                   for kx, ky in kept):
                kept.append((cx, cy))
        return len(kept)

    def count_stable(self, get_frame, center: int, roi, cls: str,
                     votes: int = 5, spread: int = 8) -> int:
        """center 프레임 주변 여러 장의 중위 개수.

        단일 프레임 판정은 깜빡임에 취약하다. 적재함 안(갈색 배경, 팔 가림)의
        검출은 진열대(흰 배경)보다 불안정해서, 놓기 성공한 물건을 한 프레임에서
        놓치면 거짓 실패가 된다. 실측: 진열대 18개 중 16개 검출.

        중위값을 쓰면 소수 프레임의 누락·오검출이 걸러진다.
        """
        offs = np.linspace(-spread * (votes // 2), spread * (votes // 2), votes)
        counts = []
        for o in offs:
            i = max(0, center + int(o))
            f = get_frame(i)
            if f is None:
                continue
            counts.append(self.count(f, roi, cls))
        if not counts:
            return 0
        return int(np.median(counts))


@dataclass
class Verdict:
    """한 번의 픽업 시도 판정 결과.

    주 신호는 진열대다. 적재함이 아니라 진열대를 기준으로 삼는 이유:

    1) 적재함은 탑뷰 프레임에서 잘린다. 실측 카드보드 영역이 (0,82)~(285,480)
       으로 프레임의 왼쪽·아래 경계에 닿는다. 상자 안쪽이 화면 밖으로 나간다.
    2) 진열대는 흰 스티로폼 배경에 물건이 칸별로 정렬돼 있어 검출이 훨씬
       안정적이다. 실측 18개 중 16개. 적재함 안은 갈색 배경 + 물건이 겹쳐
       쌓이고 팔이 가려서 더 나쁘다.
    3) 판정 시점(홈 복귀)에는 팔이 진열대에서 비켜나 있다.

    적재함은 보조 신호로만 쓴다 — 목적지(box1/box2)를 맞게 골랐는지는
    진열대만으로 알 수 없기 때문이다. 확인이 안 되면 dest_ok=None 이고,
    그 경우 성공 판정을 뒤집지 않는다.
    """

    finished: bool              # 홈으로 복귀했는가 (동작 종료)
    picked: bool                # 진열대에서 대상이 하나 사라졌는가 (주 신호)
    dest_ok: bool | None        # 지정된 적재함에 들어갔는가. None = 확인 불가
    frames: int
    shelf_before: int
    shelf_after: int
    dest_before: int = -1
    dest_after: int = -1
    reason: str = ""

    @property
    def success(self) -> bool:
        """집었고, 목적지가 틀렸다는 증거가 없으면 성공."""
        return self.picked and self.dest_ok is not False

    def __str__(self) -> str:
        d = {True: "목적지 확인", False: "목적지 오류", None: "목적지 미확인"}[self.dest_ok]
        return (f"{'성공' if self.success else '실패'}  "
                f"{self.frames}프레임 ({self.frames/30:.1f}초)  "
                f"진열대 {self.shelf_before}→{self.shelf_after}  {d}  {self.reason}")


def judge_sequence(states, get_frame, target: str, dest: str,
                   counter: BoxCounter, detector: HomeDetector | None = None,
                   timeout_frames: int = 2700, shelf_roi=None, dest_roi=None,
                   other_rois: dict | None = None,
                   check_dest: bool = True, votes: int = 5) -> Verdict:
    """관절 시퀀스로 종료 시점을 찾고, 그 시점의 탑뷰 프레임으로 판정한다.

    states      (N, 6) 관절 위치 시퀀스
    get_frame   get_frame(i) → 프레임 i 의 탑뷰 RGB uint8.
                관절 시퀀스로 종료 시점을 먼저 정한 뒤 필요한 프레임만 꺼내므로
                비싼 영상 디코딩을 최소화한다.
    shelf_roi   진열대 영역. None 이면 geometry.SHELF_ROI.
                리그가 다른 과거 데이터를 채점할 때 직접 넘긴다.
    dest_roi    목적지 적재함 영역. None 이면 geometry 에서 dest 로 찾는다.
    check_dest  False 면 적재함을 아예 보지 않는다 (진열대만으로 판정).

    오프라인(기록된 에피소드)과 온라인(실시간) 양쪽에서 쓸 수 있다.
    """
    from .geometry import SHELF_ROI, box_roi

    det = detector or HomeDetector()
    det.reset()
    finished = False
    n = 0
    for i, s in enumerate(states):
        n = i + 1
        if det.update(s):
            finished = True
            break
        if n >= timeout_frames:
            break

    sroi = shelf_roi if shelf_roi is not None else SHELF_ROI
    # 단일 프레임이 아니라 주변 여러 장의 중위값 — 깜빡임 방어
    sb = counter.count_stable(get_frame, 0, sroi, target, votes=votes)
    sa = counter.count_stable(get_frame, n - 1, sroi, target, votes=votes)

    if not finished:
        reason = "홈 복귀 없음 (타임아웃 또는 굳음)" if det.left_home else "홈을 떠나지 않음"
        return Verdict(False, False, None, n, sb, sa, reason=reason)

    # ── 주 신호: 진열대에서 하나 사라졌는가 ──────────────────────────
    delta = sb - sa
    if delta == 1:
        picked, reason = True, ""
    elif delta <= 0:
        return Verdict(True, False, None, n, sb, sa,
                       reason="헛집기 — 진열대 개수가 줄지 않음")
    else:
        return Verdict(True, False, None, n, sb, sa,
                       reason=f"진열대가 {delta}개 줄어듦 — 여러 개를 건드렸거나 검출 오류")

    # ── 보조 신호: 지정한 적재함에 들어갔는가 ────────────────────────
    dest_ok: bool | None = None
    db = da = -1
    if check_dest:
        droi = dest_roi if dest_roi is not None else box_roi(dest)
        if droi is not None:
            db = counter.count_stable(get_frame, 0, droi, target, votes=votes)
            da = counter.count_stable(get_frame, n - 1, droi, target, votes=votes)
            if da - db == 1:
                dest_ok, reason = True, ""
            else:
                # 다른 적재함에 들어갔는지 확인해야 '오류'라고 단정할 수 있다.
                other = "box2" if dest == "box1" else "box1"
                oroi = other_rois.get(other) if other_rois else box_roi(other)
                if oroi is not None:
                    ob = counter.count_stable(get_frame, 0, oroi, target, votes=votes)
                    oa = counter.count_stable(get_frame, n - 1, oroi, target, votes=votes)
                    if oa - ob == 1:
                        dest_ok = False
                        reason = f"목적지 오류 — {dest} 대신 {other} 에 넣음"
                if dest_ok is None:
                    reason = "적재함 확인 불가 (잘림·가림). 진열대 기준으로는 성공"

    return Verdict(True, picked, dest_ok, n, sb, sa, db, da, reason)


# ───────────────────────────────────────────────────────────────────────
def _calibrate(repo_id: str) -> None:
    """데이터셋에서 홈 자세와 허용 오차를 다시 계산해 출력한다."""
    import glob

    import pandas as pd

    root = f"/home/newuser/.cache/huggingface/lerobot/{repo_id}"
    ep = pd.concat([pd.read_parquet(f) for f in
                    sorted(glob.glob(f"{root}/meta/episodes/**/*.parquet", recursive=True))])
    data = pd.concat([pd.read_parquet(f) for f in
                      sorted(glob.glob(f"{root}/data/**/*.parquet", recursive=True))]).reset_index(drop=True)
    S = np.stack(data["observation.state"].values)
    start = np.stack([S[int(r["dataset_from_index"])] for _, r in ep.iterrows()])
    end = np.stack([S[int(r["dataset_to_index"]) - 1] for _, r in ep.iterrows()])

    mu, sd = start.mean(0), start.std(0)
    loop = np.abs(end.mean(0) - mu).mean()
    print(f"=== {repo_id}  ({len(ep)} 에피소드) ===")
    print(f"{'관절':16s} {'홈':>9s} {'σ':>7s} {'허용오차(3σ,최소5)':>18s}")
    for i, j in enumerate(JOINTS):
        print(f"{j:16s} {mu[i]:9.2f} {sd[i]:7.2f} {max(5.0, 3*sd[i]):18.1f}")
    print(f"\n시작↔종료 평균절대차 {loop:.2f}  → "
          f"{'✅ 홈 복귀 정상' if loop < 2 else '❌ 홈으로 복귀하지 않음 — 이 데이터셋에는 판정기를 쓸 수 없음'}")
    print("\nsuccess.py 에 붙여넣을 값:")
    print(f"HOME = np.array({np.round(mu,2).tolist()}, dtype=np.float32)")
    print(f"HOME_TOL = np.array({[round(max(5.0, 3*s),1) for s in sd]}, dtype=np.float32)")


if __name__ == "__main__":
    import argparse

    p = argparse.ArgumentParser(description="홈 자세 재계산")
    p.add_argument("--calibrate", required=True, help="기준으로 쓸 데이터셋 repo_id")
    _calibrate(p.parse_args().calibrate)
