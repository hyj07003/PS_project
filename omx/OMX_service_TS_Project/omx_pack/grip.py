"""그리퍼 기준값 측정 — 파지 성공을 판정할 임계값을 만든다.

무엇을 재는가 — 그리퍼를 닫았을 때 멈추는 위치다. 허공에서 닫으면 끝까지
닫히고, 물건을 물면 그 두께에서 멈춘다. 두 값이 갈리면 궤적만 보고도
"이번 파지가 물건을 잡았는가" 를 판정할 수 있다.

픽업에서 이렇게 얻은 값이 GRASP_MIN=51.0 이었다(허공 48.99 · 미끄러짐
49.2~49.6 · 성공 51.8~56.0). 처음에는 49.4 로 잡았다가 미끄러진 파지를
성공으로 세고 있었다 — 임계값을 눈대중으로 정하면 그렇게 된다.

포장은 아직 그 값이 없다. 2026-08-21 첫 롤아웃에서 파지 10회의 최소
그립값이 49.72~49.99 안에 전부 몰려 있어(편차 0.27도) 성공과 실패가
구분되지 않았다. 미니어처 물건이라 얇아서 차이가 작을 수 있다. 그래서
직접 재야 한다.

**어떻게 움직이는가** — 그리퍼 외의 관절은 지금 자세를 그대로 유지하도록
명령하고 그리퍼만 여닫는다. 정책이 쓰는 것과 같은 send_action 경로이므로
평소 동작과 같은 힘·같은 제어 모드다.

사용법
    # 1) 허공에서 (그리퍼 사이에 아무것도 없이)
    python -m omx_pack.grip --label air --trials 5

    # 2) 물건을 그리퍼 사이에 놓고 (사람이 들고 있어도 된다)
    python -m omx_pack.grip --label biscuit --trials 5
    python -m omx_pack.grip --label milk --trials 5

    # 3) 임계값 계산
    python -m omx_pack.grip --report

    # 물건을 끼우기 위해 열어두기
    python -m omx_pack.grip --open
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")

# 픽업의 models/rig_reference.json 과 같은 자리에 둔다 — 좌표·기준값처럼
# 작지만 없으면 재현이 불가능한 것들이 모이는 곳이다.
REF_PATH = Path("/home/newuser/il_ws/models/pack_grip_reference.json")

CLOSE_TARGET = 0.0      # 정규화 0~100 에서 완전 닫힘
OPEN_TARGET = 100.0
SETTLE_S = 1.5          # 명령 후 멈출 때까지 기다리는 시간
SAMPLE_S = 0.5          # 멈춘 뒤 평균을 낼 구간


def _connect(port: str, robot_id: str = "omx_pack_arm"):
    """카메라 없이 팔만 연결한다. 연결하는 순간 토크가 켜진다."""
    from lerobot.robots.omx_follower import OmxFollowerConfig
    from lerobot.robots.utils import make_robot_from_config

    robot = make_robot_from_config(
        OmxFollowerConfig(port=port, id=robot_id, cameras={}))
    robot.connect()
    return robot


def _state(robot) -> dict:
    obs = robot.get_observation()
    return {j: float(obs[f"{j}.pos"]) for j in JOINTS}


def _command(robot, hold: dict, gripper: float) -> None:
    """그리퍼만 목표를 바꾸고 나머지 관절은 붙잡아 둔다."""
    action = {f"{j}.pos": hold[j] for j in JOINTS}
    action["gripper.pos"] = gripper
    robot.send_action(action)


def measure(robot, target: float, fps: int = 30) -> dict:
    """그리퍼를 target 으로 보내고 멈춘 자리를 잰다."""
    hold = _state(robot)                       # 나머지 관절을 붙잡을 기준
    period = 1.0 / fps

    t0 = time.perf_counter()
    while time.perf_counter() - t0 < SETTLE_S:
        _command(robot, hold, target)
        time.sleep(period)

    vals = []
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < SAMPLE_S:
        _command(robot, hold, target)
        vals.append(_state(robot)["gripper"])
        time.sleep(period)

    a = np.array(vals, np.float64)
    return {"mean": float(a.mean()), "std": float(a.std()),
            "min": float(a.min()), "max": float(a.max()), "n": len(a)}


def load_ref() -> dict:
    if REF_PATH.exists():
        return json.loads(REF_PATH.read_text())
    return {"trials": []}


def save_ref(ref: dict) -> None:
    REF_PATH.parent.mkdir(parents=True, exist_ok=True)
    REF_PATH.write_text(json.dumps(ref, ensure_ascii=False, indent=2))


def report(ref: dict) -> str:
    trials = ref.get("trials", [])
    if not trials:
        return "측정된 값이 없습니다. --label 로 먼저 재십시오."

    by_label: dict[str, list[float]] = {}
    for t in trials:
        by_label.setdefault(t["label"], []).append(t["mean"])

    L = [f"측정 {len(trials)}회 · 항목 {len(by_label)}개", ""]
    L.append(f"{'항목':14s} {'횟수':>4s} {'평균':>8s} {'최소':>8s} {'최대':>8s} {'편차':>7s}")
    for label, vals in by_label.items():
        a = np.array(vals)
        L.append(f"{label:14s} {len(a):4d} {a.mean():8.2f} {a.min():8.2f} "
                 f"{a.max():8.2f} {a.std():7.3f}")

    air = np.array(by_label.get("air", []))
    objs = {k: np.array(v) for k, v in by_label.items() if k != "air"}
    L.append("")
    if not len(air):
        L.append("⚠ 'air' 측정이 없습니다. 허공에서 닫은 값이 있어야 임계값을 정할 수 있습니다.")
        return "\n".join(L)
    if not objs:
        L.append("⚠ 물건 측정이 없습니다. --label <물건이름> 으로 재십시오.")
        return "\n".join(L)

    all_obj = np.concatenate(list(objs.values()))
    gap = all_obj.min() - air.max()
    L.append(f"허공 최대 {air.max():.2f}  ·  물건 최소 {all_obj.min():.2f}  "
             f"·  간격 {gap:+.2f}도")
    L.append("")
    if gap <= 0:
        # 겹치면 이 방법으로는 못 가른다. 억지로 임계값을 정하면 픽업에서
        # 49.4 로 잡았을 때처럼 실패를 성공으로 센다.
        L.append("⚠ 허공과 물건의 범위가 겹칩니다 — 그립값만으로는 파지 성공을")
        L.append("  판정할 수 없습니다. 겹치는 항목:")
        for k, v in objs.items():
            if v.min() <= air.max():
                L.append(f"    {k}: 최소 {v.min():.2f} ≤ 허공 최대 {air.max():.2f}")
        L.append("  → 얇은 물건이라면 이 방법이 안 맞습니다. 탑뷰 영상으로")
        L.append("    바구니에 늘어난 개수를 세는 쪽을 검토하십시오.")
    else:
        # 픽업(kinematic.py)과 같은 방식 — 성공 최소와 허공 평균의 중간
        thr = (all_obj.min() + air.mean()) / 2
        L.append(f"→ GRASP_MIN 권장  {thr:.2f}")
        L.append(f"   (물건 최소 {all_obj.min():.2f} 와 허공 평균 {air.mean():.2f} 의 중간)")
        margin = min(all_obj.min() - thr, thr - air.max())
        L.append(f"   여유 {margin:.2f}도 — 0.3 미만이면 표본을 더 모으십시오.")
    return "\n".join(L)


def main() -> None:
    ap = argparse.ArgumentParser(description="포장 그리퍼 기준값 측정")
    ap.add_argument("--robot-port", default="/dev/omx_pack_follower")
    ap.add_argument("--robot-id", default="omx_pack_arm")
    ap.add_argument("--label", default=None,
                    help="측정 항목 이름. 허공은 반드시 'air' 로 재십시오")
    ap.add_argument("--trials", type=int, default=5)
    ap.add_argument("--open", action="store_true",
                    help="그리퍼를 열고 끝낸다 (물건을 끼울 때)")
    ap.add_argument("--report", action="store_true", help="측정값으로 임계값 계산")
    ap.add_argument("--reset", action="store_true", help="측정값을 모두 지운다")
    a = ap.parse_args()

    if a.reset:
        save_ref({"trials": []})
        print(f"측정값을 지웠습니다: {REF_PATH}")
        return

    if a.report:
        print(report(load_ref()))
        return

    if not a.open and not a.label:
        ap.error("--label 또는 --open 이 필요합니다 (--report 로 결과 확인)")

    print("팔에 연결합니다 — 연결하는 순간 토크가 켜집니다.")
    robot = _connect(a.robot_port, a.robot_id)
    try:
        hold = _state(robot)
        print(f"현재 그리퍼 {hold['gripper']:.2f}")

        if a.open:
            m = measure(robot, OPEN_TARGET)
            print(f"열었습니다 → {m['mean']:.2f}")
            return

        ref = load_ref()
        print(f"\n'{a.label}' {a.trials}회 측정")
        for i in range(1, a.trials + 1):
            op = measure(robot, OPEN_TARGET)
            cl = measure(robot, CLOSE_TARGET)
            ref["trials"].append({
                "label": a.label, "mean": cl["mean"], "std": cl["std"],
                "min": cl["min"], "max": cl["max"],
                "open_at": op["mean"], "at": time.strftime("%Y-%m-%d %H:%M:%S"),
            })
            print(f"  {i}/{a.trials}  닫힘 {cl['mean']:6.2f} "
                  f"(±{cl['std']:.3f})   열림 {op['mean']:6.2f}")
        save_ref(ref)
        print(f"\n{REF_PATH} 에 저장했습니다.")
        print("\n" + report(ref))
    finally:
        # 물건을 문 채로 두지 않는다. 다음 측정을 위해 열어 둔다.
        try:
            measure(robot, OPEN_TARGET)
        except Exception:                             # noqa: BLE001
            pass
        robot.disconnect()
        print("\n연결을 해제했습니다.")


if __name__ == "__main__":
    main()
