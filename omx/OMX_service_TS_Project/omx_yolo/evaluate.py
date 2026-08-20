"""기록된 데이터셋을 조건별로 채점해 성공률 표를 만든다.

    # 정책 롤아웃 채점 (본래 용도)
    python -m omx_yolo.evaluate --repo-id kdy93/eval_smolvla_yolo

    # 판정기 자체 검증 — 사람이 시연한 데이터는 거의 100% 여야 한다
    python -m omx_yolo.evaluate --repo-id kdy93/smart_market_prototype_2 \
                                --single-only --limit 30

왜 필요한가
    "데이터가 충분한가"는 데이터를 봐서 알 수 없다. 조건별 롤아웃 성공률로만
    역산할 수 있다. 그런데 기존 eval_* 데이터셋 8개는 모두 0 에피소드다 —
    --dataset.episode_time_s=100000 으로 무한정 돌리다 Ctrl+C 로 죽여서
    아무것도 저장되지 않았다. 즉 지금 비교할 기준 숫자가 하나도 없다.

    조건을 쪼개 보는 이유: prototype_4 실측에서 box1 이 box2 보다 픽업 1회당
    평균 35% 느렸다(42초 vs 31초). 전체 성공률 70% 가 "모든 조건 70%" 일 수도
    있고 "box2 95% / box1 45%" 일 수도 있는데, 후자면 box1 데이터만 더 모으면
    된다. 합격선은 조건 평균 80% 이상, 어떤 조건도 50% 미만 없음.

롤아웃을 제대로 저장하는 명령
    python -m omx_yolo.evaluate --print-record-cmd
"""

from __future__ import annotations

import argparse
import sys
from collections import defaultdict

import numpy as np

from .annotate import parse_task
from .kinematic import KinematicJudge

RECORD_CMD = r"""
# 정책 롤아웃을 조건별로 기록하는 명령. 조건 하나당 N회 반복해서 돌린다.
#
# 핵심: --dataset.episode_time_s 를 현실적인 값으로 둘 것. 100000 으로 두면
# 무한정 돌다가 Ctrl+C 로 죽고, 그러면 LeRobot 은 아무것도 저장하지 않는다.
# 에피소드는 → (오른쪽 화살표) 로 정상 종료해야 저장된다.
# 90초는 prototype_2 최장 에피소드(92초)를 기준으로 잡은 값이다.

export PYTHONPATH=/home/newuser/il_ws/src:$PYTHONPATH
export YOLO_AUTOINSTALL=false

lerobot-record \
  --robot.type=omx_follower --robot.port=/dev/omx_follower --robot.id=omx_follower_arm \
  --robot.cameras="{
      front: {type: yolo_opencv, index_or_path: /dev/omx_cam_top,
              width: 640, height: 480, fps: 30, fourcc: MJPG, warmup_s: 2,
              weights: /home/newuser/il_ws/models/omx_goods_yolo11n.pt},
      wrist: {type: opencv, index_or_path: /dev/omx_cam_hand,
              width: 640, height: 480, fps: 30, fourcc: MJPG}
   }" \
  --policy.path=outputs/train/<학습결과>/checkpoints/last/pretrained_model \
  --dataset.repo_id=kdy93/eval_<이름> \
  --dataset.single_task="Pick up sandwich and place it in the box1" \
  --dataset.num_episodes=10 \
  --dataset.episode_time_s=90 \
  --dataset.reset_time_s=15 \
  --dataset.push_to_hub=false \
  --display_data=true

# 조건 격자: (상품 3종 × 적재함 2개) = 6조건, 조건당 10회 → 60 에피소드.
# 잔여수량 3/2/1 을 고루 섞으려면 리필 주기를 3회로 맞춘다.
"""


def main() -> None:
    p = argparse.ArgumentParser(description="조건별 성공률 채점")
    p.add_argument("--repo-id", help="채점할 데이터셋")
    p.add_argument("--weights", default="/home/newuser/il_ws/models/omx_goods_yolo11n.pt")
    p.add_argument("--single-only", action="store_true",
                   help="단일 픽업 에피소드만 채점 (다중 픽업은 홈 복귀가 여러 번)")
    p.add_argument("--limit", type=int, help="앞 N개 에피소드만")
    p.add_argument("--episodes", help="채점할 에피소드 (예: 0-29)")
    p.add_argument("--print-record-cmd", action="store_true",
                   help="롤아웃 기록 명령을 출력하고 종료")
    p.add_argument("--auto-roi", action="store_true",
                   help="(사용 안 함) 관절 판정으로 전환되어 무시된다")
    p.add_argument("--no-dest-check", action="store_true",
                   help="(사용 안 함) 관절 판정으로 전환되어 무시된다")
    p.add_argument("--verbose", action="store_true", help="에피소드별 상세 출력")
    a = p.parse_args()

    if a.print_record_cmd:
        print(RECORD_CMD)
        return
    if not a.repo_id:
        sys.exit("--repo-id 가 필요합니다 (또는 --print-record-cmd)")

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    from .convert import parse_ranges, video_path

    ds = LeRobotDataset(a.repo_id)
    keys = list(ds.meta.video_keys)
    top = "observation.images.front" if "observation.images.front" in keys else keys[0]
    print(f"=== {a.repo_id} ===")
    print(f"  에피소드 {ds.num_episodes}  프레임 {ds.num_frames}  탑뷰 스트림 {top}\n")

    want = set(range(ds.num_episodes))
    if a.episodes:
        want &= parse_ranges(a.episodes)
    todo = sorted(want)
    if a.limit:
        todo = todo[: a.limit]

    judge = KinematicJudge()
    print(f"판정 기준 (관절 전용, 카메라 미사용)")
    print(f"  파지 성공  그리퍼 >= {judge.grasp_min}")
    print(f"  목적지     shoulder_pan 경계 {judge.dest_boundary}"
          f" ({'낮은쪽' if judge.box1_is_lower else '높은쪽'}=box1)\n")

    results: dict[tuple[str, str], list] = defaultdict(list)
    skipped = 0
    for ep_i in todo:
        row = ds.meta.episodes[ep_i]
        lo, hi = int(row["dataset_from_index"]), int(row["dataset_to_index"])
        task = str(row["tasks"][0])
        target, dest = parse_task(task)
        if target is None or dest is None:
            skipped += 1
            continue
        multi = any(w in task.lower() for w in (" 2 ", " 3 ")) and \
            not any(o in task.lower() for o in ("1st", "2nd", "3rd"))
        if multi and a.single_only:
            skipped += 1
            continue

        states = np.stack(ds.hf_dataset[lo:hi]["observation.state"]).astype(np.float32)
        v = judge(states, dest)
        results[(target, dest)].append(v)
        if a.verbose:
            print(f"  ep{ep_i:4d} {target:9s}→{dest}  {v}")

    if not results:
        sys.exit("채점할 에피소드가 없습니다.")

    # ── 조건별 표 ─────────────────────────────────────────────────────
    print(f"{'조건':22s} {'n':>3s} {'성공':>4s} {'성공률':>7s} {'종료':>4s} "
          f"{'파지':>4s} {'목적지OK':>8s} {'평균초':>7s}  실패 사유")
    print("-" * 104)
    tot_n = tot_ok = 0
    worst = (1.1, None)
    for (target, dest), vs in sorted(results.items()):
        n = len(vs)
        ok = sum(v.success for v in vs)
        fin = sum(v.finished for v in vs)
        dok = sum(v.dest_ok is True for v in vs)
        grab = sum(v.grasped for v in vs)
        dunk = sum(v.dest_ok is None for v in vs)
        sec = np.mean([v.frames for v in vs]) / 30
        reasons = defaultdict(int)
        for v in vs:
            if not v.success:
                reasons[v.reason] += 1
        rs = ", ".join(f"{k}×{c}" for k, c in sorted(reasons.items(), key=lambda x: -x[1]))
        rate = ok / n
        dcol = f"{dok}/{n-dunk}" if n - dunk else "—"
        print(f"{target+' → '+dest:22s} {n:3d} {ok:4d} {rate*100:6.1f}% {fin:4d} "
              f"{grab:4d} {dcol:>8s} {sec:7.1f}  {rs}")
        tot_n += n
        tot_ok += ok
        if rate < worst[0]:
            worst = (rate, f"{target} → {dest}")

    print("-" * 92)
    print(f"{'전체':22s} {tot_n:3d} {tot_ok:4d} {tot_ok/tot_n*100:6.1f}%")
    if skipped:
        print(f"  (건너뜀 {skipped}개 — 다중 픽업 또는 task 파싱 실패)")

    print("\n=== 합격 판정 ===")
    avg = tot_ok / tot_n
    print(f"  조건 평균 80% 이상 : {avg*100:.1f}%  {'✅' if avg >= 0.8 else '❌'}")
    print(f"  최저 조건 50% 이상 : {worst[0]*100:.1f}% ({worst[1]})  "
          f"{'✅' if worst[0] >= 0.5 else '❌'}")
    if worst[0] < 0.5:
        print(f"  → 가장 약한 조건은 '{worst[1]}' 입니다. 이 조건 데이터를 더 모으십시오.")


if __name__ == "__main__":
    main()
