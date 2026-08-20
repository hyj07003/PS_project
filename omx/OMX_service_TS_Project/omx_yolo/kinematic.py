"""관절 상태만으로 픽업 시도를 판정한다. 카메라를 쓰지 않는다.

왜 시각을 버렸나
────────────────────────────────────────────────────────────────────────
처음에는 탑뷰 프레임에서 물체 개수를 세어 성공을 판정하려 했다. 실측 결과
검출기 신뢰도가 판정을 지탱하지 못했다.

    현재 리그      진열대 18개 중 16개 검출
    prototype_4    진열대 샌드위치 2개 중 1개만 검출 (칸이 프레임 가장자리에 걸림)
    prototype_2    한 칸에 샌드위치 4개로 과다 검출

게다가 ROI 는 리그 배치가 조금만 바뀌어도 무효가 된다. 실제로 하루 사이에
카드보드 영역이 43px 이동해 판정이 깨졌다.

관절 신호는 이 모든 문제에서 자유롭다. 조명·배치·카메라와 무관하다.

세 가지 신호 (전부 사람 시연으로 실측 검증)
────────────────────────────────────────────────────────────────────────
1. 동작 종료 — 홈 자세 복귀
   prototype_2/4 에서 20/20 정확. success.HomeDetector 참조.

2. 파지 성공 — 그리퍼가 끝까지 닫히지 않음
   사람은 조종기 그리퍼를 끝까지 쥔다(지령 최소값 47.9 부근).
   그런데 실제 팔은 물체 두께에 막혀 덜 닫힌다. 성공 300건 실측:

       sandwich   52.62 ± 0.16   (51.9 ~ 52.9)
       icecream   53.77 ± 0.51   (51.9 ~ 54.3)
       milk       55.79 ± 0.94   (50.7 ~ 56.2)

   헛집기는 직접 실험해 관측했다 (kdy93/grasp_test):

       허공에서 꽉 쥠     48.99   (지령 47.64, 차이 +1.34)
       샌드위치를 쥠      52.58   (지령 47.91, 차이 +4.66)

   허공에서도 지령보다 1.34 덜 닫히는데, 이는 그리퍼 자체의 기구적 한계다.
   샌드위치의 52.58 은 기존 시연 100건의 52.62 ± 0.16 과 일치한다.

   smart_market_v1(현재 리그, 180 에피소드) 재보정:
       그리퍼 실제 최소  53.34 ± 1.21   (최소 49.9)
       그리퍼 지령 최소  47.94 ± 0.06

   ── 2026-08-19 재조정: 49.4 → 51.0 ────────────────────────────────
   첫 정책 롤아웃(roll→box1 5회)에서 49.4 가 **실패를 성공으로 통과**시켰다.
   사람이 눈으로 확인한 결과와 대조:

       ep3  52.8  집기 성공
       ep4  52.7  집기 성공
       ep5  52.8  집기 성공
       ep6  49.6  실패 — 물체보다 아래를 집어 핑거가 미끄러짐  ← 49.4 통과
       ep7  49.4  실패 — 같은 실패                            ← 49.4 통과

   미끄러진 파지는 헛집기(48.99)보다 조금 위, 성공(52.7~52.8)보다 한참 아래에
   찍힌다. 49.4 는 헛집기만 잡고 미끄러짐은 놓치는 값이었다.

   학습 데이터 180 에피소드(사람 시연 = 전부 성공)의 그리퍼 최소값 분포:

       biscuit  52.62 ± 0.16  (최소 52.28)    roll      52.82 ± 0.13  (최소 52.50)
       cake     52.70 ± 0.12  (최소 52.48)    sandwich  52.68 ± 0.10  (최소 52.41)
       icecream 53.58 ± 1.01  (최소 49.87)    milk      55.62 ± 0.93  (최소 52.43)
       전체     53.34 ± 1.21  (5%분위 52.43)

   51.0 을 쓰면 관측된 실패(49.6, 49.4)와 성공(52.3~52.8) 사이를 양쪽 1.3~1.4
   여유로 가른다. 학습 180개 중 오판은 2개(1.1%)뿐이고, 그 둘은 icecream
   49.87/50.3 으로 사람 시연에서도 미끄러졌을 가능성이 높다.

   더 정확히 하려면 상품별 경계를 두는 것이 맞다(milk 는 55.6 으로 한참 위).
   지금은 실패 표본이 roll 2건뿐이라 전역 값 하나로 둔다. 상품별 실패가
   더 모이면 나눌 것.

   GRASP_MAX 는 59.0. 열린 채 유지되는 값(59.3~59.5)만 걸러내되,
   가장 두꺼운 물체(milk 56.2)는 통과시켜야 한다.

   ⚠ 남은 한계: 헛집기 관측이 1회뿐이라 그 분산을 모른다. 여러 번 관측하면
     경계를 더 좁게 잡을 수 있다.

3. 목적지 — 놓는 순간의 shoulder_pan
   box1 과 box2 는 팔의 회전각이 다르다. 실측 분류 정확도:

       prototype_1   n=175   98.9%
       prototype_2   n=300   97.3%
       prototype_4   n= 20   90.0%   ← 오류 2건이 정확히 ep10/ep11

   prototype_4 의 오류는 이미 오염으로 지목한 그 두 에피소드다.
   서로 독립적인 두 방법이 같은 결론에 도달했다.

   ⚠ 경계값은 리그마다 다르다. 리그를 옮기면 calibrate() 로 다시 구할 것.
────────────────────────────────────────────────────────────────────────
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

GRIPPER = 5
SHOULDER_PAN = 0

# 파지 성공 판정은 구간이다. 하한만 걸면 "아예 닫지 않은" 경우도 통과한다.
#   완전 폐쇄(헛집기)  48.99      ← GRASP_MIN 아래
#   미끄러진 파지       49.4~49.6  ← GRASP_MIN 아래 (2026-08-19 실측 2건)
#   물체를 뭄           52.3~56.2  ← 이 구간
#   열린 채 그대로      59.3~59.5  ← GRASP_MAX 위 (정책이 아무 동작도 안 함)
GRASP_MIN = 51.0
GRASP_MAX = 59.0

# 목적지 경계 (shoulder_pan). smart_market_v1 = 현재 리그·현재 화각 기준.
# box1 -27.00 ± 2.50 / box2 -18.94 ± 2.74, 180 에피소드 분류 정확도 99.4%.
# 리그를 옮기면 --calibrate 로 다시 구할 것.
DEST_BOUNDARY = -22.97
DEST_BOX1_IS_LOWER = True     # boundary 보다 작은(더 음수) 쪽이 box1


@dataclass
class PickResult:
    finished: bool           # 홈으로 복귀했는가
    grasped: bool            # 물체를 물었는가
    dest_pred: str | None    # 관절로 추정한 목적지
    dest_ok: bool | None     # 지시한 목적지와 일치하는가
    frames: int
    grip_min: float          # 그리퍼가 도달한 최소값
    release_pan: float       # 놓는 순간 shoulder_pan
    reason: str = ""

    @property
    def success(self) -> bool:
        return self.finished and self.grasped and self.dest_ok is not False

    def __str__(self) -> str:
        d = {True: "목적지 O", False: "목적지 X", None: "목적지 ?"}[self.dest_ok]
        return (f"{'성공' if self.success else '실패'}  "
                f"{self.frames}프레임 ({self.frames/30:.1f}초)  "
                f"그리퍼 {self.grip_min:.1f}  pan {self.release_pan:+.1f}→{self.dest_pred}  "
                f"{d}  {self.reason}")


def find_release(states: np.ndarray) -> int | None:
    """그리퍼가 닫혔다가 열리는 마지막 지점 = 물건을 놓는 순간."""
    g = states[:, GRIPPER]
    if len(g) < 3:
        return None
    thr = (g.min() + g.max()) / 2
    closed = g < thr
    tr = [i for i in range(1, len(g)) if closed[i - 1] and not closed[i]]
    return tr[-1] if tr else None


@dataclass
class KinematicJudge:
    """관절 시퀀스 하나를 받아 픽업 성공을 판정한다."""

    grasp_min: float = GRASP_MIN
    grasp_max: float = GRASP_MAX
    dest_boundary: float = DEST_BOUNDARY
    box1_is_lower: bool = DEST_BOX1_IS_LOWER
    home_detector: object | None = None

    def classify_dest(self, pan: float) -> str:
        lower = pan < self.dest_boundary
        return "box1" if (lower == self.box1_is_lower) else "box2"

    def __call__(self, states: np.ndarray, dest: str | None = None,
                 timeout_frames: int = 2700) -> PickResult:
        from .success import HomeDetector

        states = np.asarray(states, dtype=np.float32)
        det = self.home_detector or HomeDetector()
        det.reset()

        finished, n = False, 0
        for i, s in enumerate(states):
            n = i + 1
            if det.update(s):
                finished = True
                break
            if n >= timeout_frames:
                break

        seg = states[:n]
        grip_min = float(seg[:, GRIPPER].min())
        grasped = self.grasp_min <= grip_min <= self.grasp_max
        never_closed = grip_min > self.grasp_max

        rel = find_release(seg)
        pan = float(seg[rel, SHOULDER_PAN]) if rel is not None else float("nan")
        dest_pred = self.classify_dest(pan) if rel is not None else None
        dest_ok = None if (dest is None or dest_pred is None) else (dest_pred == dest)

        if not finished:
            reason = "홈 복귀 없음" if det.left_home else "홈을 떠나지 않음"
        elif never_closed:
            reason = (f"동작 없음 — 그리퍼가 {grip_min:.1f} 로 열린 채 유지 "
                      f"(닫힘 상한 {self.grasp_max})")
        elif not grasped:
            kind = "헛집기" if grip_min < 49.2 else "미끄러짐"
            reason = f"{kind} — 그리퍼가 {grip_min:.1f} 까지 닫힘 (하한 {self.grasp_min})"
        elif rel is None:
            reason = "놓는 동작이 없음"
        elif dest_ok is False:
            reason = f"목적지 오류 — {dest} 지시인데 {dest_pred} 위치에서 놓음"
        else:
            reason = ""

        return PickResult(finished, grasped, dest_pred, dest_ok, n,
                          grip_min, pan, reason)


# ───────────────────────────────────────────────────────────────────────
def calibrate(repo_id: str) -> None:
    """데이터셋에서 파지·목적지 경계값을 다시 구한다."""
    import glob

    import pandas as pd

    root = f"/home/newuser/.cache/huggingface/lerobot/{repo_id}"
    ep = pd.concat([pd.read_parquet(f) for f in
                    sorted(glob.glob(f"{root}/meta/episodes/**/*.parquet", recursive=True))])
    data = pd.concat([pd.read_parquet(f) for f in
                      sorted(glob.glob(f"{root}/data/**/*.parquet", recursive=True))]).reset_index(drop=True)
    S = np.stack(data["observation.state"].values)
    A = np.stack(data["action"].values)

    grips, cmds, pans = [], [], {"box1": [], "box2": []}
    for _, r in ep.iterrows():
        a, b = int(r["dataset_from_index"]), int(r["dataset_to_index"])
        seg = S[a:b]
        grips.append(seg[:, GRIPPER].min())
        cmds.append(A[a:b, GRIPPER].min())
        t = str(r["tasks"][0])
        dest = "box1" if "box1" in t else ("box2" if "box2" in t else None)
        rel = find_release(seg)
        if dest and rel is not None:
            pans[dest].append(seg[rel, SHOULDER_PAN])

    g, c = np.array(grips), np.array(cmds)
    print(f"=== {repo_id}  ({len(ep)} 에피소드) ===")
    print(f"  그리퍼 실제 최소  {g.mean():6.2f} ± {g.std():.2f}   최소 {g.min():.1f}")
    print(f"  그리퍼 지령 최소  {c.mean():6.2f} ± {c.std():.2f}")
    print(f"  → GRASP_MIN 권장  {(g.min() + c.mean()) / 2:.1f}")
    b1, b2 = np.array(pans["box1"]), np.array(pans["box2"])
    if len(b1) and len(b2):
        mid = (b1.mean() + b2.mean()) / 2
        lower_is_b1 = b1.mean() < b2.mean()
        acc = (sum((x < mid) == lower_is_b1 for x in b1)
               + sum((x < mid) != lower_is_b1 for x in b2)) / (len(b1) + len(b2))
        print(f"  box1 pan  n={len(b1):3d}  {b1.mean():7.2f} ± {b1.std():.2f}")
        print(f"  box2 pan  n={len(b2):3d}  {b2.mean():7.2f} ± {b2.std():.2f}")
        print(f"  → DEST_BOUNDARY = {mid:.2f}   DEST_BOX1_IS_LOWER = {lower_is_b1}")
        print(f"    이 경계의 분류 정확도 {acc*100:.1f}%")


if __name__ == "__main__":
    import argparse

    p = argparse.ArgumentParser(description="파지·목적지 경계 재보정")
    p.add_argument("--calibrate", required=True)
    calibrate(p.parse_args().calibrate)
