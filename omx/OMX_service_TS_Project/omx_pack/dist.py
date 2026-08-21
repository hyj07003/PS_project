"""시작 자세가 정책이 학습한 분포 안에 있는지 확인한다.

왜 필요한가 — 이 프로젝트에서 조용히 망가진 것들은 전부 같은 모양이었다.
예외도 로그도 없이 그럴듯하게 동작하는데 결과만 틀렸다. 지시문 표기가
어긋나 샌드위치를 집었고, 그리퍼 임계값이 낮아 미끄러진 파지를 성공으로
셌고, 정책이 순서를 어겨 학습에 없는 진열 상태를 만들고는 빈 칸을 헛집었다.

**정책은 학습 분포 밖 상태에서도 자신 있게 행동한다.** 그것이 위험한
이유다 — 못 하겠다고 말해 주지 않는다.

체크포인트에는 학습 데이터의 관절별 통계가 정규화용으로 들어 있다
(policy_preprocessor_step_3_normalizer_processor.safetensors). 그 값과
지금 팔의 자세를 대조하면, 정책을 돌리기 전에 "본 적 있는 상태인가" 를
물어볼 수 있다. 공짜로 얻는 안전장치다 — 별도 측정이 필요 없다.

2026-08-21 첫 하드웨어 연결에서 실제로 걸렸다. 그리퍼가 63.03 이었는데
학습 최대가 61.83 이었다 — 학습에서 한 번도 본 적 없이 벌어진 상태였다.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")

STATS_FILE = "policy_preprocessor_step_3_normalizer_processor.safetensors"

# 판정 등급
OK = "OK"           # q01~q99 안. 학습이 흔히 본 상태다
EDGE = "EDGE"       # min~max 안이지만 1% 꼬리. 본 적은 있으나 드물다
OUT = "OUT"         # min~max 밖. 학습에서 한 번도 본 적 없다


@dataclass
class JointVerdict:
    joint: str
    value: float
    grade: str
    lo: float
    hi: float
    q01: float
    q99: float

    @property
    def message(self) -> str:
        if self.grade == OUT:
            side = "아래" if self.value < self.lo else "위"
            edge = self.lo if self.value < self.lo else self.hi
            return (f"{self.joint} {self.value:.2f} 는 학습 범위 "
                    f"[{self.lo:.2f}, {self.hi:.2f}] 의 {side}입니다 "
                    f"({edge:.2f} 에서 {abs(self.value - edge):.2f} 벗어남)")
        if self.grade == EDGE:
            return (f"{self.joint} {self.value:.2f} 는 학습 분포의 1% 꼬리에 "
                    f"있습니다 (q01 {self.q01:.2f} · q99 {self.q99:.2f})")
        return f"{self.joint} {self.value:.2f} 정상"


class StateRange:
    """체크포인트에서 읽은 관절별 학습 분포."""

    def __init__(self, checkpoint: str | Path):
        path = Path(checkpoint) / STATS_FILE
        if not path.exists():
            raise FileNotFoundError(
                f"정규화 통계를 찾을 수 없습니다: {path}\n"
                "체크포인트 경로가 pretrained_model 디렉터리인지 확인하십시오.")
        from safetensors.numpy import load_file

        d = load_file(str(path))

        def g(key: str) -> np.ndarray:
            return np.asarray(d[f"observation.state.{key}"], np.float64).ravel()

        self.min, self.max = g("min"), g("max")
        self.q01, self.q99 = g("q01"), g("q99")
        self.checkpoint = str(checkpoint)

    def check(self, state: np.ndarray) -> list[JointVerdict]:
        out = []
        for i, j in enumerate(JOINTS):
            v = float(state[i])
            if v < self.min[i] or v > self.max[i]:
                grade = OUT
            elif v < self.q01[i] or v > self.q99[i]:
                grade = EDGE
            else:
                grade = OK
            out.append(JointVerdict(j, v, grade, float(self.min[i]),
                                    float(self.max[i]), float(self.q01[i]),
                                    float(self.q99[i])))
        return out

    def summary(self, state: np.ndarray) -> dict:
        """서버 응답에 실을 형태. 관제가 그대로 로그에 남길 수 있게 짧게."""
        vs = self.check(state)
        outs = [v for v in vs if v.grade == OUT]
        edges = [v for v in vs if v.grade == EDGE]
        grade = OUT if outs else (EDGE if edges else OK)
        return {
            "grade": grade,
            "out": [v.joint for v in outs],
            "edge": [v.joint for v in edges],
            "messages": [v.message for v in outs + edges],
            "state": [round(float(x), 2) for x in state],
        }


def format_report(vs: list[JointVerdict]) -> str:
    """사람이 읽는 표. 서버 기동 로그와 CLI 가 함께 쓴다."""
    L = [f"{'관절':16s} {'측정':>8s} {'학습 범위':>20s} {'판정':>6s}"]
    for v in vs:
        rng = f"[{v.lo:7.2f}, {v.hi:7.2f}]"
        mark = {OK: "정상", EDGE: "가장자리", OUT: "범위 밖"}[v.grade]
        L.append(f"{v.joint:16s} {v.value:8.2f} {rng:>20s} {mark:>6s}")
    return "\n".join(L)


def main() -> None:
    """CLI — 지금 팔 자세를 체크포인트 분포와 대조한다.

        python -m omx_pack.dist --basket yellow --robot-port /dev/omx_pack_follower
    """
    import argparse

    from .vocab import resolve_checkpoint

    ap = argparse.ArgumentParser(description="시작 자세 분포 확인")
    ap.add_argument("--basket", default="yellow")
    ap.add_argument("--checkpoint", default=None)
    ap.add_argument("--robot-port", default="/dev/omx_pack_follower")
    ap.add_argument("--state", nargs=6, type=float, default=None,
                    help="팔에 붙지 않고 값 6개를 직접 넣어 확인한다")
    a = ap.parse_args()

    rng = StateRange(a.checkpoint or resolve_checkpoint(a.basket))
    if a.state is not None:
        state = np.array(a.state, np.float64)
    else:
        # 팔에서 직접 읽는다 — 모터를 읽기만 하므로 움직이지 않는다.
        from lerobot.motors import Motor, MotorNormMode
        from lerobot.motors.dynamixel import DynamixelMotorsBus

        models = ("xl430-w250",) * 3 + ("xl330-m288",) * 3
        motors = {j: Motor(11 + i, models[i],
                           MotorNormMode.RANGE_0_100 if j == "gripper"
                           else MotorNormMode.DEGREES)
                  for i, j in enumerate(JOINTS)}
        bus = DynamixelMotorsBus(port=a.robot_port, motors=motors)
        bus.connect(handshake=False)
        state = np.array([bus.read("Present_Position", j) for j in JOINTS],
                         np.float64)
        bus.disconnect()

    vs = rng.check(state)
    print(format_report(vs))
    s = rng.summary(state)
    print()
    if s["grade"] == OK:
        print("모든 관절이 학습 분포 안에 있습니다.")
    else:
        for m in s["messages"]:
            print(f"  ⚠ {m}")


if __name__ == "__main__":
    main()
