"""포장 팔의 홈(대기) 자세 — 저장·조회·복귀.

왜 별도로 두는가 — 포장 정책은 작업이 끝나도 **스스로 홈으로 돌아가지
않는다**(2026-08-21 실측: 적재함을 비운 뒤에도 빈 공간을 계속 집으려 들고,
멈춘 자리에 그대로 선다). 픽업 정책은 홈 복귀로 끝나서 그것을 종료 신호로
썼지만 포장은 그럴 수 없다.

그래서 서버가 끝난 뒤 직접 데려다 놓는다. 팔이 적재함 위에 서 있으면
탑뷰를 가려서 다음 판정을 방해하고, 사람이 물건을 채워 넣기도 불편하다.

**픽업의 홈 값을 쓰면 안 된다.** 다른 팔이고 다른 자리다(픽업 홈은
`[-1.04, -63.03, 54.08, 42.59, -4.14, 59.54]`, omx_yolo.success.HOME).
그래서 기본값을 비워 두고, 실제로 팔을 세워 둔 자세를 재서 쓴다.

    # 팔을 원하는 대기 자세에 두고
    python -m omx_pack.home --capture

    # 확인
    python -m omx_pack.home --show

측정된 자세는 `models/pack_home.json` 에 저장된다.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper")

HOME_PATH = Path("/home/newuser/il_ws/models/pack_home.json")


def load_home() -> np.ndarray | None:
    """저장된 홈 자세. 없으면 None — **픽업 값으로 대체하지 않는다.**"""
    if not HOME_PATH.exists():
        return None
    d = json.loads(HOME_PATH.read_text())
    return np.array([d["pose"][j] for j in JOINTS], np.float32)


def save_home(pose: np.ndarray, note: str = "") -> Path:
    HOME_PATH.parent.mkdir(parents=True, exist_ok=True)
    HOME_PATH.write_text(json.dumps({
        "pose": {j: round(float(v), 2) for j, v in zip(JOINTS, pose)},
        "measuredAt": time.strftime("%Y-%m-%d %H:%M:%S"),
        "note": note or "팔을 대기 자세에 두고 --capture 로 기록",
    }, ensure_ascii=False, indent=2))
    return HOME_PATH


def interpolate_home(robot, home: np.ndarray, seconds: float = 3.0,
                     fps: int = 30) -> dict:
    """현재 자세에서 홈까지 천천히 보간해서 이동한다.

    한 번에 목표를 던지지 않는 이유는 픽업과 같다 — 거리가 멀면 팔이
    급하게 튄다. 픽업 omx_yolo.server.go_home 과 같은 방식이다.
    """
    obs = robot.get_observation()
    cur = np.array([obs[f"{j}.pos"] for j in JOINTS], np.float32)
    names = list(robot.action_features)
    n = max(1, int(seconds * fps))
    for i in range(n):
        a = cur + (home - cur) * ((i + 1) / n)
        robot.send_action({k: float(v) for k, v in zip(names, a)})
        time.sleep(1.0 / fps)
    obs = robot.get_observation()
    end = np.array([obs[f"{j}.pos"] for j in JOINTS], np.float32)
    return {"ok": True,
            "target": [round(float(v), 2) for v in home],
            "reached": [round(float(v), 2) for v in end],
            "maxErrorDeg": round(float(np.abs(end - home).max()), 2)}


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(description="포장 팔 홈 자세")
    ap.add_argument("--capture", action="store_true",
                    help="지금 자세를 홈으로 저장한다")
    ap.add_argument("--show", action="store_true", help="저장된 값을 보여준다")
    ap.add_argument("--go", action="store_true", help="저장된 홈으로 이동한다")
    ap.add_argument("--robot-port", default="/dev/omx_pack_follower")
    ap.add_argument("--robot-id", default="omx_pack_arm")
    ap.add_argument("--note", default="")
    a = ap.parse_args()

    if a.show:
        if not HOME_PATH.exists():
            raise SystemExit(f"저장된 홈이 없습니다: {HOME_PATH}")
        print(HOME_PATH.read_text())
        return

    if not (a.capture or a.go):
        ap.error("--capture, --show, --go 중 하나가 필요합니다")

    print("팔에 연결합니다 — 토크가 켜집니다. 주변을 비우십시오.")
    from lerobot.robots.omx_follower import OmxFollowerConfig
    from lerobot.robots.utils import make_robot_from_config

    robot = make_robot_from_config(
        OmxFollowerConfig(port=a.robot_port, id=a.robot_id, cameras={}))
    robot.connect()
    try:
        if a.capture:
            obs = robot.get_observation()
            pose = np.array([obs[f"{j}.pos"] for j in JOINTS], np.float32)
            path = save_home(pose, a.note)
            print("\n홈 자세로 기록했습니다:")
            for j, v in zip(JOINTS, pose):
                print(f"  {j:16s} {v:8.2f}")
            print(f"\n{path}")

            # 학습 분포 안인지도 같이 알려 준다. 홈이 분포 밖이면 다음
            # 작업이 정책이 본 적 없는 상태에서 시작한다.
            try:
                from .dist import OUT, StateRange
                from .vocab import resolve_checkpoint

                s = StateRange(resolve_checkpoint("yellow")).summary(pose)
                if s["grade"] == OUT:
                    print("\n⚠ 이 자세는 학습 범위 밖입니다:")
                    for m in s["messages"]:
                        print(f"    {m}")
                    print("  이대로 두면 다음 작업이 학습에 없는 상태에서 시작합니다.")
                else:
                    print("\n학습 분포 안입니다 — 다음 작업 시작 자세로 적합합니다.")
            except Exception:                          # noqa: BLE001
                pass
        else:
            home = load_home()
            if home is None:
                raise SystemExit(f"저장된 홈이 없습니다: {HOME_PATH}\n"
                                 "먼저 --capture 로 기록하십시오.")
            print("홈으로 이동합니다...")
            r = interpolate_home(robot, home)
            print(f"  도착 오차 최대 {r['maxErrorDeg']:.2f}도")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
