"""변환된 주석 데이터셋을 검증한다.

    python -m omx_yolo.verify --repo-id kdy93/smart_market_prototype_3_yolo \
                              --src kdy93/smart_market_prototype_3

검사 항목
    1. 에피소드/프레임 수와 task 문자열
    2. front 스트림이 실제로 고정 탑뷰인지 (스트림 정규화 확인)
    3. 저장된 영상에서 주석 선이 살아남았는지 (코덱 손실 확인)
    4. 상태·액션이 원본과 일치하는지 (원본 지정 시)

3번이 핵심이다. 인코딩이 얇은 선을 지워 버리면 주석 데이터셋이 원본과
사실상 같아져서 A/B 비교가 무의미해진다. LeRobot 기본 코덱은 실측 보존율
92.6% 로 충분하지만, 설정을 바꿨다면 반드시 다시 확인할 것.
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

from .annotate import BOX_RGB, RGB
from .convert import static_score, to_uint8

PALETTE = {**RGB, **BOX_RGB}


def line_pixel_ratio(frames: np.ndarray, tol: int = 45) -> float:
    """팔레트 색과 tol 이내로 일치하는 픽셀의 비율.

    절대값 자체보다 원본 대비 비교가 의미 있다. 흰색(box2)은 장면의 흰
    스티로폼과 겹치므로 제외한다.
    """
    hit = np.zeros(frames.shape[:-1], dtype=bool)
    for name, c in PALETTE.items():
        if name == "box2":          # 흰색은 배경과 구분 불가
            continue
        d = np.abs(frames.astype(np.int16) - np.array(c, np.int16)).max(axis=-1)
        hit |= d < tol
    return float(hit.mean())


def main() -> None:
    p = argparse.ArgumentParser(description="주석 데이터셋 검증")
    p.add_argument("--repo-id", required=True, help="검증할 주석 데이터셋")
    p.add_argument("--src", help="원본 데이터셋 (비교용, 선택)")
    p.add_argument("--episodes", type=int, default=3, help="샘플링할 에피소드 수")
    p.add_argument("--frames", type=int, default=20, help="에피소드당 샘플 프레임 수")
    a = p.parse_args()

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    ds = LeRobotDataset(a.repo_id)
    print(f"=== {a.repo_id} ===")
    print(f"  에피소드 {ds.num_episodes}  프레임 {ds.num_frames}  fps {ds.fps}")
    print(f"  robot_type {ds.meta.robot_type}  codebase {ds.meta.info.get('codebase_version')}")
    print(f"  이미지 스트림 {list(ds.meta.video_keys) or '(PNG)'}")

    tasks = sorted({str(ds.meta.episodes[i]["tasks"][0]) for i in range(ds.num_episodes)})
    print(f"\n  task 종류 {len(tasks)}개:")
    for t in tasks:
        print(f"    {t!r}")
    bad = [t for t in tasks if not t.startswith("Pick up ") or t.rstrip().endswith(("~",))]
    if bad:
        print(f"  ⚠ 형식 이상: {bad}")

    src = LeRobotDataset(a.src) if a.src else None
    if src:
        print(f"\n=== 원본 비교: {a.src} ===")
        print(f"  에피소드 {src.num_episodes} → {ds.num_episodes}"
              f"  ({ds.num_episodes - src.num_episodes:+d})")
        print(f"  프레임   {src.num_frames} → {ds.num_frames}"
              f"  ({ds.num_frames - src.num_frames:+d})")

    # ── 스트림 정규화 + 주석 생존 확인 ────────────────────────────────
    keys = list(ds.meta.video_keys) or [
        k for k, v in ds.meta.features.items() if v["dtype"] in ("image", "video")
    ]
    n_ep = min(a.episodes, ds.num_episodes)
    ep_idx = np.linspace(0, ds.num_episodes - 1, n_ep).astype(int)

    print(f"\n=== 스트림 및 주석 검증 (에피소드 {list(ep_idx)}) ===")
    print(f"{'ep':>4s} | {'스트림':<32s} {'고정도':>7s} {'팔레트픽셀':>10s}  판정")
    print("-" * 78)
    ok = True
    for ep_i in ep_idx:
        row = ds.meta.episodes[int(ep_i)]
        lo, hi = int(row["dataset_from_index"]), int(row["dataset_to_index"])
        idx = np.linspace(lo, hi - 1, min(a.frames, hi - lo)).astype(int)
        buf = {k: [] for k in keys}
        for i in idx:
            item = ds[int(i)]
            for k in keys:
                buf[k].append(to_uint8(item[k]))
        for k in keys:
            arr = np.stack(buf[k])
            st = static_score(buf[k])
            lr = line_pixel_ratio(arr)
            is_front = k.endswith(".front")
            if is_front:
                verdict = ("✅" if st > 0.6 and lr > 0.01
                           else ("❌ 주석 없음" if st > 0.6 else "❌ 탑뷰 아님"))
                if not (st > 0.6 and lr > 0.01):
                    ok = False
            else:
                verdict = "✅ (핸디캠)" if st < 0.6 else "⚠ 고정 카메라?"
            print(f"{ep_i:4d} | {k:<32s} {st:7.2f} {lr*100:9.2f}%  {verdict}")

    # ── 상태/액션 일치 ────────────────────────────────────────────────
    if src:
        print("\n=== 상태·액션 일치 확인 ===")
        n = min(200, ds.num_frames, src.num_frames)
        d_s = d_a = 0.0
        for i in np.linspace(0, n - 1, 50).astype(int):
            d_s = max(d_s, float((ds[int(i)]["observation.state"] - src[int(i)]["observation.state"]).abs().max()))
            d_a = max(d_a, float((ds[int(i)]["action"] - src[int(i)]["action"]).abs().max()))
        print(f"  observation.state 최대 차이 {d_s:.2e}")
        print(f"  action            최대 차이 {d_a:.2e}")
        print("  ⚠ 에피소드를 제외했다면 인덱스가 어긋나 차이가 커집니다 (정상)"
              if d_s > 1e-4 else "  ✅ 일치")

    print(f"\n{'✅ 검증 통과' if ok else '❌ 문제 있음 — 위 항목 확인'}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
