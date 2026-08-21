"""에피소드 관절 궤적 기록과 분석.

무엇을 위한 것인가 — 포장 정책의 **종료 판정 방식**과 **홈 자세**를 정하기
위해서다. 둘 다 눈으로 봐서는 값이 안 나온다. 픽업에서 GRASP_MIN=51.0 을
정할 때도 관절 궤적을 모아 성공/실패 분포를 갈랐지, 팔을 보고 정하지
않았다(그렇게 했다가 49.4 로 잡아서 미끄러진 파지를 성공으로 세고 있었다).

한 번 돌려서 궤적을 남기면 세 가지가 동시에 나온다:

  · 담기가 끝난 뒤 팔이 멈추는가, 계속 움직이는가  → 종료 판정 방식
  · 에피소드 시작 시점의 자세                       → 홈 자세 후보
  · 그 자세가 에피소드마다 재현되는가               → 허용 오차(tol)

기록 형식은 에피소드당 .npz 하나다. 30fps · 60초면 1,800×6 float32 =
43KB 라 개수를 걱정할 필요가 없다.

    기록:  서버에 --trace-dir 를 주면 에피소드마다 자동으로 쌓인다
    분석:  python -m omx_pack.trace <디렉터리>
"""

from __future__ import annotations

import json
import re
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")


# ───────────────────────────────────────────────────────────────────────
#  기록
# ───────────────────────────────────────────────────────────────────────
@dataclass
class TraceWriter:
    """한 에피소드의 관절 궤적을 모아 .npz 로 저장한다.

    제어 루프 안에서 부르므로 append 는 절대 느리면 안 된다 — 리스트에
    쌓기만 하고 파일은 끝에서 한 번 쓴다. 33ms 주기를 놓치면 정책이
    학습 때와 다른 속도로 움직인다.
    """

    dir: Path
    job_id: str
    index: int
    basket: str
    fps: int = 30
    _t: list = field(default_factory=list, init=False)
    _s: list = field(default_factory=list, init=False)
    _a: list = field(default_factory=list, init=False)
    _t0: float = field(default_factory=time.perf_counter, init=False)

    def append(self, state: np.ndarray, action: np.ndarray | None = None) -> None:
        """관측된 자세와 **그 프레임에 내린 명령**을 함께 쌓는다.

        명령이 왜 필요한가 — 그리퍼 값만으로는 파지 성공을 알 수 없다.
        정책은 사람이 시연한 만큼만 여닫으므로(2026-08-21 실측: 정책 범위
        49.7~59.6 · 하드웨어 범위 48.3~82.0), 닫힌 값이 낮다고 해서 허공을
        문 것이 아니다. 그냥 거기까지만 닫으라고 명령한 것일 수 있다.

        판정은 **명령과 실제의 차이**로 해야 한다:
            명령 49.8 → 실제 49.8   막는 게 없었다 (실패)
            명령 49.8 → 실제 52.0   물건이 막았다 (성공)
        """
        self._t.append(time.perf_counter() - self._t0)
        self._s.append(np.asarray(state, dtype=np.float32))
        self._a.append(None if action is None
                       else np.asarray(action, dtype=np.float32))

    def close(self, meta: dict | None = None) -> Path | None:
        if not self._s:
            return None
        self.dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        path = self.dir / f"{stamp}_{self.job_id}_ep{self.index:02d}.npz"
        arrays = {}
        if any(a is not None for a in self._a):
            # 명령이 없는 프레임(관측 전용)은 NaN 으로 채워 길이를 맞춘다
            nan = np.full(len(JOINTS), np.nan, np.float32)
            arrays["actions"] = np.stack([nan if a is None else a
                                          for a in self._a])
        np.savez_compressed(
            path,
            states=np.stack(self._s),
            times=np.asarray(self._t, dtype=np.float32),
            joints=np.array(JOINTS),
            **arrays,
            meta=json.dumps({"jobId": self.job_id, "index": self.index,
                             "basket": self.basket, "fps": self.fps,
                             **(meta or {})}, ensure_ascii=False),
        )
        return path


# ───────────────────────────────────────────────────────────────────────
#  분석
# ───────────────────────────────────────────────────────────────────────
@dataclass
class EpisodeReport:
    path: Path
    meta: dict
    states: np.ndarray                 # (n, 6)
    times: np.ndarray                  # (n,)
    fps: int
    actions: np.ndarray | None = None  # (n, 6) 또는 None (명령 미기록)

    @property
    def seconds(self) -> float:
        return float(self.times[-1]) if len(self.times) else 0.0

    @property
    def motion(self) -> np.ndarray:
        """프레임 간 관절 변화량 합(도). 팔이 얼마나 움직였는지."""
        if len(self.states) < 2:
            return np.zeros(0, np.float32)
        return np.abs(np.diff(self.states, axis=0)).sum(1)

    def last_moving_frame(self, thresh: float = 0.5) -> int:
        """마지막으로 유의미하게 움직인 프레임. 없으면 -1."""
        m = self.motion
        idx = np.where(m >= thresh)[0]
        return int(idx[-1]) if len(idx) else -1

    def tail_still_sec(self, thresh: float = 0.5) -> float:
        """끝에서 몇 초나 정지해 있었는가. 종료 판정의 핵심 단서."""
        last = self.last_moving_frame(thresh)
        if last < 0:
            return self.seconds
        return float(len(self.states) - 1 - last) / self.fps

    def returned_home(self, tol: float = 5.0, hold_sec: float = 0.3) -> bool:
        """끝에서 시작 자세로 돌아와 **머물렀는가**.

        마지막 한 프레임만 보면 안 된다. 팔이 계속 왕복하고 있으면 그 궤도가
        시작 자세를 지나가는 순간이 있고, 그 순간을 끝으로 잡으면 "복귀했다"
        로 읽힌다(가짜 팔 keep-moving 에서 실제로 그렇게 오판했다).

        HomeFinish 가 요구하는 것도 '도달' 이 아니라 '도달해서 머무름' 이므로,
        판정 기준을 그쪽과 같게 맞춘다.
        """
        k = max(2, int(hold_sec * self.fps))
        if len(self.states) < k + 1:
            return False
        tail = self.states[-k:]
        return bool(np.all(np.abs(tail - self.states[0]) < tol))


# ───────────────────────────────────────────────────────────────────────
#  파지 주기 — 그리퍼 여닫음을 세고, 명령과 실제의 차이로 성공을 가른다
# ───────────────────────────────────────────────────────────────────────
GRIPPER = 5


def grasp_cycles(e: "EpisodeReport", open_th: float = 55.0,
                 close_th: float = 52.0, stall_min: float = 0.5) -> list[dict]:
    """그리퍼가 닫혔다 열린 구간을 하나의 파지 시도로 센다.

    임계 두 개(히스테리시스)를 쓰는 이유는 값이 경계에서 떨리면 한 번의
    여닫음이 여러 번으로 세지기 때문이다.

    각 시도의 성공 여부는 **명령과 실제의 차이(stall)** 로 본다. 그리퍼에
    물건이 끼면 명령한 곳까지 못 닫히고 그보다 큰 값에서 멈춘다. 절대값을
    쓰지 않는 이유는 정책이 매번 같은 깊이로 닫으라고 명령하지 않기
    때문이다 — 절대값으로 보면 명령이 얕았던 것과 물건에 막힌 것이 섞인다.
    """
    g = e.states[:, GRIPPER]
    cmd = None if e.actions is None else e.actions[:, GRIPPER]
    fps = e.fps
    out: list[dict] = []
    state = "open" if len(g) and g[0] > open_th else "closed"
    start = 0
    for i, v in enumerate(g):
        if state == "open" and v < close_th:
            state, start = "closed", i
        elif state == "closed" and v > open_th:
            out.append(_cycle(e, g, cmd, start, i, fps, stall_min))
            state = "open"
    if state == "closed" and start < len(g) - 1:
        c = _cycle(e, g, cmd, start, len(g), fps, stall_min)
        c["unfinished"] = True
        out.append(c)
    return out


def _cycle(e, g, cmd, start, end, fps, stall_min) -> dict:
    seg = g[start:end]
    k = int(np.argmin(seg))                  # 가장 깊이 닫힌 순간
    hold = float(seg[k])
    c = {"start_s": start / fps, "end_s": end / fps,
         "dur_s": (end - start) / fps, "hold": hold,
         "pan_at_release": float(e.states[min(end, len(e.states) - 1), 0])}
    if cmd is not None:
        commanded = float(cmd[start + k])
        stall = hold - commanded             # 양수면 명령보다 덜 닫혔다
        # blocked 는 "그리퍼가 명령한 곳까지 못 갔다" 는 뜻이다.
        # **물건을 담았다는 뜻이 아니다** — grasp_report 의 주석 참조.
        c.update(commanded=commanded, stall=stall,
                 blocked=bool(stall >= stall_min))
    return c


# 2026-08-21 실측으로 확인된 것 — 이 표를 읽는 사람이 반드시 알아야 한다.
GRASP_CAVEAT = """\
⚠ 아래 '막힘' 은 그리퍼가 명령한 위치까지 닫히지 못했다는 뜻일 뿐,
  **물건을 담았다는 뜻이 아니다.**

  정책은 그리퍼에 47.6 근처를 명령하는데 이는 그리퍼가 도달할 수 없는
  값이다(완전 닫힘 실측 48.27, 그것도 1.5초를 눌러야 나온다). 그래서
  물건이 있든 없든 차이가 항상 +2 근처로 뜬다.

  2026-08-21 대조: 실제로 담긴 개수 3 / 1 / 1 인 세 에피소드에서 이 방식은
  8 / 16 / 16 으로 셌다. 차이(stall)의 사분위 범위가 [+1.88, +2.12] 로 거의
  상수라 신호가 아니다. lift·절대값 등 여섯 가지 규칙을 대조했으나 세 회차를
  모두 맞히는 것은 없었다.

  **담긴 개수는 관절값으로 셀 수 없다.** 파지 '시도' 횟수와 타이밍을 보는
  용도로만 쓸 것."""


def grasp_report(eps: list[EpisodeReport]) -> str:
    L = [GRASP_CAVEAT, ""]
    for e in eps:
        cyc = grasp_cycles(e)
        has_cmd = e.actions is not None
        L.append(f"── {e.path.name}  ({e.seconds:.1f}초 · 파지 시도 {len(cyc)}회)")
        if not cyc:
            L.append("   그리퍼 여닫음이 없습니다.")
            continue
        head = f"   {'#':>2} {'시작':>7} {'닫힘':>6} {'실제':>7}"
        if has_cmd:
            head += f" {'명령':>7} {'차이':>7} {'막힘':>6}"
        L.append(head + f" {'놓을때 pan':>11}")
        for n, c in enumerate(cyc, 1):
            row = (f"   {n:2d} {c['start_s']:6.1f}s {c['dur_s']:5.1f}s "
                   f"{c['hold']:7.2f}")
            if has_cmd:
                row += (f" {c['commanded']:7.2f} {c['stall']:+7.2f} "
                        f"{'막힘' if c['blocked'] else '도달':>6}")
            row += f" {c['pan_at_release']:11.2f}"
            if c.get("unfinished"):
                row += "  (미완)"
            L.append(row)
        if has_cmd:
            ok = sum(1 for c in cyc if c["blocked"])
            L.append(f"   → 파지 시도 {len(cyc)}회 중 {ok}회에서 그리퍼가 "
                     "막혔습니다 (담긴 개수가 아닙니다)")
        else:
            L.append("   ⚠ 명령이 기록되지 않은 궤적입니다 — 성공 여부를 "
                     "판정할 수 없습니다.")
            L.append("     (2026-08-21 이전 궤적. 이후 궤적에는 명령이 함께 남습니다)")
    return "\n".join(L)


def load(path: Path) -> EpisodeReport:
    z = np.load(path, allow_pickle=False)
    meta = json.loads(str(z["meta"]))
    return EpisodeReport(path=path, meta=meta, states=z["states"],
                         times=z["times"], fps=int(meta.get("fps", 30)),
                         actions=z["actions"] if "actions" in z.files else None)


def load_dir(d: Path) -> list[EpisodeReport]:
    return [load(p) for p in sorted(Path(d).glob("*.npz"))]


def analyze(eps: list[EpisodeReport], still_thresh: float = 0.5) -> dict:
    """에피소드 묶음에서 홈 자세와 종료 판정 근거를 뽑는다."""
    if not eps:
        return {"episodes": 0}

    starts = np.stack([e.states[0] for e in eps])
    ends = np.stack([e.states[-1] for e in eps])
    tails = np.array([e.tail_still_sec(still_thresh) for e in eps])
    secs = np.array([e.seconds for e in eps])
    home_back = np.array([e.returned_home() for e in eps])

    # 홈 자세 후보 — 에피소드 시작 자세의 중앙값.
    # 평균이 아니라 중앙값인 이유: 한 에피소드가 엉뚱한 자세에서 시작해도
    # 값이 끌려가지 않는다. 표본이 적을 때 특히 중요하다.
    home = np.median(starts, axis=0)
    # 허용 오차 — 편차의 3배에 최소 2도. 픽업의 HOME_TOL(5~10도)과 비슷한
    # 자릿수가 나와야 정상이다.
    spread = starts.max(0) - starts.min(0)
    tol = np.maximum(spread * 1.5, 2.0)

    return {
        "episodes": len(eps),
        "seconds": {"mean": float(secs.mean()), "min": float(secs.min()),
                    "max": float(secs.max())},
        "home": home,
        "home_spread": spread,
        "home_tol": tol,
        "start_std": starts.std(0),
        "end_median": np.median(ends, axis=0),
        "tail_still_sec": {"mean": float(tails.mean()), "min": float(tails.min()),
                           "max": float(tails.max())},
        "returned_home_ratio": float(home_back.mean()),
    }


def _mixes(eps: list[EpisodeReport]) -> list[tuple[str, dict]]:
    """섞이면 안 되는 축이 실제로 섞였는지 본다. 섞인 축만 돌려준다."""
    import collections

    def source(e: EpisodeReport) -> str:
        """가짜 팔인지 실제 팔인지, 가짜라면 어떤 행동이었는지."""
        r = str(e.meta.get("reason", ""))
        if "가짜 팔" not in r:
            return "실제 팔"
        m = re.search(r"\(([^)]+)\)", r)
        return f"가짜 팔({m.group(1)})" if m else "가짜 팔"

    # 종료 사유 자체는 축으로 쓰지 않는다. 실기에서는 성공과 시간초과가
    # 섞이는 것이 정상이라, 그것으로 경고를 내면 매번 헛경고가 된다.
    # 섞이면 실제로 값이 망가지는 축만 본다.
    axes = {
        "바구니": [e.meta.get("basket", "?") for e in eps],
        "출처": [source(e) for e in eps],
        "fps": [str(e.meta.get("fps", "?")) for e in eps],
    }
    out = []
    for label, vals in axes.items():
        c = collections.Counter(vals)
        if len(c) > 1:
            out.append((label, dict(c)))
    return out


def _fmt(v: np.ndarray) -> str:
    return "[" + ", ".join(f"{x:7.2f}" for x in v) + "]"


def report(eps: list[EpisodeReport], still_thresh: float = 0.5) -> str:
    a = analyze(eps, still_thresh)
    if not a.get("episodes"):
        return "궤적 파일이 없습니다."

    L = [f"에피소드 {a['episodes']}개 · 길이 평균 {a['seconds']['mean']:.1f}초 "
         f"(최소 {a['seconds']['min']:.1f} · 최대 {a['seconds']['max']:.1f})", ""]

    # 섞인 궤적 경고.
    #
    # 한 디렉터리에 서로 다른 조건의 궤적이 쌓이면 분석기는 그것을 모르고
    # 하나로 평균 낸다. 바구니마다 홈 자세가 다를 수 있으므로, 노랑과 민트를
    # 섞어 놓고 뽑은 HOME 은 어느 쪽도 아닌 값이 된다. 예외도 경고도 없이
    # 그럴듯한 숫자가 나오는 것이 가장 나쁘다.
    mixes = _mixes(eps)
    if mixes:
        L.append("⚠ 서로 다른 조건의 궤적이 섞여 있습니다 — 아래 값은 신뢰할 수 없습니다.")
        for label, counts in mixes:
            detail = " · ".join(f"{k}×{v}" for k, v in counts.items())
            L.append(f"    {label}: {detail}")
        L.append("    → 조건별로 디렉터리를 나눠 다시 분석하십시오.")
        L.append("")

    L.append("── 에피소드별 ──────────────────────────────────────────")
    L.append(f"  {'파일':28s} {'초':>6s} {'끝정지':>7s} {'홈복귀':>7s}")
    for e in eps:
        L.append(f"  {e.path.name[:28]:28s} {e.seconds:6.1f} "
                 f"{e.tail_still_sec(still_thresh):6.1f}s "
                 f"{'예' if e.returned_home() else '아니오':>7s}")
    L.append("")

    obs_only = all(e.meta.get("observeOnly") for e in eps)
    L.append("── 홈 자세 후보 (시작 자세 중앙값) ─────────────────────"
             if not obs_only else
             "── 현재 팔 자세 (관측 전용 — 홈 자세라는 근거는 없음) ──")
    L.append(f"  관절      {'  '.join(f'{j[:9]:>9s}' for j in JOINTS)}")
    L.append(f"  HOME     {_fmt(a['home'])}")
    L.append(f"  편차폭   {_fmt(a['home_spread'])}")
    L.append(f"  HOME_TOL {_fmt(a['home_tol'])}")
    L.append(f"  끝 자세  {_fmt(a['end_median'])}")
    L.append("")

    # ── 판정 ────────────────────────────────────────────────────────
    tail = a["tail_still_sec"]
    ratio = a["returned_home_ratio"]
    L.append("── 판단 ────────────────────────────────────────────────")

    # 관측 전용 궤적으로는 종료 판정을 할 수 없다.
    #
    # 팔에 명령을 보내지 않았으므로 "끝까지 정지했다" 와 "시작 자세로
    # 돌아왔다" 가 언제나 참이다. 정책이 무엇을 하는지에 대해 아무것도
    # 말해 주지 않는데, 그럴듯한 숫자가 나오기 때문에 오히려 위험하다
    # (2026-08-21 첫 관측에서 hold_sec=30.1 이라는 무의미한 권장값이 나왔다).
    if all(e.meta.get("observeOnly") for e in eps):
        L.append("  · 관측 전용 궤적입니다 — 팔에 명령을 보내지 않았습니다.")
        L.append("    종료 판정(StallFinish/HomeFinish)은 이 자료로 정할 수 없습니다.")
        L.append("    아래 자세는 '지금 팔이 놓인 자리'이지 정책이 복귀하는 자리가")
        L.append("    아닙니다. 정책을 실제로 돌린 궤적이 필요합니다.")
        return "\n".join(L)
    # 정지 구간의 길이는 에피소드 길이에 비례해 늘어난다. 짧은 시험
    # 에피소드에서 "0.4초밖에 안 멈췄다" 는 것은 정책이 안 멈춘다는 뜻이
    # 아니라 에피소드가 짧다는 뜻이다. 그래서 절대 초와 비율을 함께 본다.
    frac = tail["min"] / max(a["seconds"]["mean"], 1e-6)
    if tail["max"] < 0.3:
        L.append("  · 끝까지 계속 움직였습니다 — 정책이 스스로 멈추지 않습니다.")
        L.append("    → StallFinish 는 쓸 수 없습니다. DurationFinish 를 유지하십시오.")
    elif tail["min"] >= 1.0:
        hold = max(0.5, tail["min"] * 0.5)
        L.append(f"  · 모든 에피소드가 끝에서 최소 {tail['min']:.1f}초 "
                 f"(길이의 {frac*100:.0f}%) 정지했습니다.")
        L.append(f"    → StallFinish 를 쓸 수 있습니다. hold_sec={hold:.1f} 권장 "
                 "(관측된 최소 정지의 절반).")
    elif frac >= 0.1:
        L.append(f"  · 끝에서 {tail['min']:.1f}~{tail['max']:.1f}초 정지합니다 "
                 f"(길이의 {frac*100:.0f}%). 멈추기는 하지만 표본 길이가 짧습니다.")
        L.append("    → 실제 길이(60초)로 몇 번 더 돌린 뒤 판단하십시오.")
    else:
        L.append(f"  · 정지 시간이 들쭉날쭉합니다 "
                 f"({tail['min']:.1f}~{tail['max']:.1f}초).")
        L.append("    → 표본을 더 모으십시오. 지금 StallFinish 를 쓰면 오판합니다.")

    if ratio >= 0.9:
        L.append(f"  · {ratio*100:.0f}% 가 시작 자세로 복귀했습니다.")
        L.append("    → HomeFinish 를 쓸 수 있습니다. 위 HOME/HOME_TOL 을 넣으십시오.")
    elif ratio <= 0.1:
        L.append("  · 시작 자세로 복귀하지 않습니다.")
        L.append("    → HomeFinish 는 쓸 수 없습니다(픽업과 다른 점입니다).")
    else:
        L.append(f"  · 복귀율이 {ratio*100:.0f}% 로 일관되지 않습니다.")
        L.append("    → 복귀하는 경우와 아닌 경우가 무엇이 다른지 봐야 합니다.")

    if float(np.max(a["home_spread"])) > 15.0:
        L.append(f"  · 시작 자세가 최대 {float(np.max(a['home_spread'])):.1f}도까지 "
                 "흩어져 있습니다 — 매번 같은 자리에서 시작하지 않았습니다.")
        L.append("    → 홈 자세로 쓰기에 부적합합니다. 팔을 같은 자세에 두고 다시 모으십시오.")
    return "\n".join(L)


def main() -> None:
    import argparse

    p = argparse.ArgumentParser(description="포장 에피소드 궤적 분석")
    p.add_argument("dir", help="궤적 .npz 가 들어 있는 디렉터리")
    p.add_argument("--still-thresh", type=float, default=0.5,
                   help="정지로 볼 프레임 간 관절 변화량 합(도)")
    p.add_argument("--grasp", action="store_true",
                   help="파지 주기와 성공 여부를 본다 (명령이 기록된 궤적에서)")
    a = p.parse_args()
    eps = load_dir(Path(a.dir))
    if a.grasp:
        print(grasp_report(eps))
        return
    print(report(eps, a.still_thresh))


if __name__ == "__main__":
    main()
