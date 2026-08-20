"""기존 LeRobotDataset → YOLO 주석 데이터셋 오프라인 변환.

원본은 절대 수정하지 않는다. 새 repo_id 로 별개 데이터셋을 만든다.

    python -m omx_yolo.convert \
        --src kdy93/smart_market_prototype_3 \
        --dst kdy93/smart_market_prototype_3_yolo

이 스크립트가 하는 일 네 가지
────────────────────────────────────────────────────────────────────────
1. 탑뷰 스트림에 YOLO 검출 박스를 렌더링한다.
   annotate.Annotator 를 그대로 쓴다 — 추론 경로(camera.py)와 완전히 같은 코드.

2. 스트림 이름을 정규화한다 (--canonical, 기본 켜짐).
   출력의 observation.images.front 는 항상 탑뷰(주석됨),
   observation.images.wrist 는 항상 핸디캠(원본)이 되도록 맞춘다.
   prototype_4 ep0~11 처럼 입력이 뒤바뀐 구간도 자동으로 교정된다.

3. task 문자열을 prototype_4 형식으로 정규화한다 (--task-policy).
   "Pick up 1 sandwich and place it in the box1"   → "Pick up sandwich and place it in the box1"
   "Pick up 2nd milk carton and place it in the box2" → "Pick up milk and place it in the box2"
   기존 오타 두 건("...box2~", "ick up ...")도 함께 고친다.

   ⚠ 수량 2 이상 에피소드는 단일 픽업 형식으로 바꿀 수 없다. 한 에피소드에
   픽업이 2~3회 들어 있으므로, 단일 픽업 프롬프트로 라벨을 바꾸면 정책이
   "한 번 집어라"는 지시에 세 번 집도록 학습된다. 기본 정책은 그런
   에피소드를 원본 라벨 그대로 둔다(--task-policy normalize). 버리려면
   single-only 를 쓴다.

4. 특정 에피소드를 제외한다 (--skip-episodes).
   예: prototype_4 는 ep0~11 이 카메라 뒤바뀜 + 라벨 오염 구간이다.
────────────────────────────────────────────────────────────────────────

비디오 코덱에 대하여
    LeRobot 기본값(libsvtav1 / yuv420p / crf30)으로 90프레임을 왕복 측정한
    결과 그려 넣은 선 신호의 92.6% 가 보존되었다(선 픽셀 평균 색오차 12.8/255).
    4px 두께가 크로마 서브샘플링을 견딘다. 기본값으로 충분하다.

    더 높은 충실도가 필요하면 --hq 로 h264 / yuv444p / crf10 을 쓴다
    (보존율 99.9%, 용량 약 2.2배). LeRobot 이 pix_fmt·crf 를 노출하지 않으므로
    런타임 몽키패치로 처리한다 — lerobot 소스는 건드리지 않는다.

    --png 은 무손실이지만 프레임 하나당 약 270KB 라 대형 데이터셋에는 비현실적이다
    (prototype_2 라면 200GB 이상). 소규모 검증용으로만 쓸 것.
"""

from __future__ import annotations

import argparse
import re
import sys
import time

import numpy as np

from .annotate import Annotator, build_task, parse_task


# 기존 데이터셋의 알려진 오타
TYPO_FIXES = [
    (re.compile(r"^ick up "), "Pick up "),      # prototype_2: P 누락
    (re.compile(r"~+\s*$"), ""),                # prototype_1: box2~
]

ORDINAL = re.compile(r"\b([123])(?:st|nd|rd)\b")
QUANTITY = re.compile(r"\bup\s+([123])\s")


def fix_typos(task: str) -> str:
    for pat, rep in TYPO_FIXES:
        task = pat.sub(rep, task)
    return task.strip()


def classify_task(task: str) -> tuple[str, int, str | None, str | None]:
    """(정리된 문자열, 픽업 횟수, 상품 클래스, 목적지) 를 돌려준다.

    픽업 횟수: 1 이면 단일 픽업(정규화 가능), 2 이상이면 다중 픽업.
    """
    clean = fix_typos(task)
    target, dest = parse_task(clean)
    if ORDINAL.search(clean):
        n = 1                      # "2nd sandwich" 는 한 번 집는 것
    else:
        m = QUANTITY.search(clean)
        n = int(m.group(1)) if m else 1
    return clean, n, target, dest


def normalize_task(target: str, dest: str) -> str:
    """annotate.build_task 를 그대로 쓴다 — 표기가 흩어지면 어긋난다."""
    return build_task(target, dest)


def parse_ranges(spec: str) -> set[int]:
    """"0-9,12,15-17" → {0..9, 12, 15,16,17}"""
    out: set[int] = set()
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-", 1)
            out.update(range(int(a), int(b) + 1))
        else:
            out.add(int(part))
    return out


def to_uint8(chw_float) -> np.ndarray:
    """LeRobotDataset 이 주는 CHW float32 [0,1] → HWC uint8 RGB (무손실)."""
    return (chw_float.permute(1, 2, 0).numpy() * 255.0).round().astype(np.uint8)


def static_score(frames: list[np.ndarray]) -> float:
    """중위 프레임 근처에 머무는 픽셀 비율. 고정 카메라면 높다(>0.6)."""
    g = np.stack([f.mean(axis=2) for f in frames]).astype(np.float32)
    return float((np.abs(g - np.median(g, axis=0)) < 18).mean())


def detect_topview(ds, a: int, b: int, keys: list[str], n: int = 10) -> str:
    """에피소드 [a,b) 에서 어느 스트림이 고정 탑뷰인지 판별."""
    idx = np.linspace(a, b - 1, min(n, b - a)).astype(int)
    samples: dict[str, list[np.ndarray]] = {k: [] for k in keys}
    for i in idx:
        item = ds[int(i)]
        for k in keys:
            samples[k].append(to_uint8(item[k]))
    scores = {k: static_score(v) for k, v in samples.items()}
    return max(scores, key=scores.get), scores


# 프레임 간격의 절반. decode_video_frames 의 타임스탬프 허용 오차.
# src.tolerance_s(기본 1e-4)는 너무 촘촘해 배치 디코딩에서 실패한다.
DECODE_TOL = 0.02
DECODE_CHUNK = 250


def video_path(src, key: str, ep_row) -> "object":
    return src.root / src.meta.info["video_path"].format(
        video_key=key,
        chunk_index=int(ep_row[f"videos/{key}/chunk_index"]),
        file_index=int(ep_row[f"videos/{key}/file_index"]),
    )


def iter_episode_images(src, ep_row, keys: list[str], chunk: int = DECODE_CHUNK):
    """에피소드의 두 스트림 프레임을 배치 디코딩으로 순차 산출한다.

    ds[i] 개별 접근은 매 프레임마다 AV1 디코딩을 다시 시작해 느리다
    (실측 26 fps). decode_video_frames 로 범위를 한 번에 디코딩하면
    110 fps 로 약 4.2배 빠르다.

    yield: (offset, {key: (m, H, W, 3) uint8 RGB})
    """
    from lerobot.datasets.video_utils import decode_video_frames

    paths = {k: video_path(src, k, ep_row) for k in keys}
    starts = {k: float(ep_row[f"videos/{k}/from_timestamp"]) for k in keys}
    n = int(ep_row["length"])
    for off in range(0, n, chunk):
        m = min(chunk, n - off)
        out = {}
        for k in keys:
            ts = [starts[k] + (off + i) / src.fps for i in range(m)]
            t = decode_video_frames(paths[k], ts, tolerance_s=DECODE_TOL)
            out[k] = (t.permute(0, 2, 3, 1).numpy() * 255.0).round().astype(np.uint8)
        yield off, out


# ───────────────────────────────────────────────────────────────────────
def main() -> None:
    p = argparse.ArgumentParser(description="LeRobotDataset → YOLO 주석 데이터셋")
    p.add_argument("--src", required=True, help="원본 repo_id")
    p.add_argument("--dst", required=True, help="출력 repo_id (원본과 달라야 함)")
    p.add_argument("--weights", default="/home/newuser/il_ws/models/omx_goods_yolo11n.pt")
    p.add_argument("--topview", choices=["auto", "front", "wrist"], default="auto",
                   help="탑뷰 스트림. auto 는 에피소드별 자동 판별 (기본)")
    p.add_argument("--canonical", action="store_true", default=True,
                   help="출력의 front 를 항상 탑뷰로 맞춘다 (기본 켜짐)")
    p.add_argument("--no-canonical", dest="canonical", action="store_false")
    p.add_argument("--task-policy", choices=["keep", "normalize", "single-only"],
                   default="normalize",
                   help="keep=오타만 수정 | normalize=단일 픽업만 _4 형식으로 "
                        "(다중은 원본 유지) | single-only=다중 픽업 에피소드 제외")
    p.add_argument("--episodes", help="변환할 에피소드 (예: 0-59,70)")
    p.add_argument("--skip-episodes", help="제외할 에피소드 (예: 0-11)")
    p.add_argument("--min-frames", type=int, default=0,
                   help="이보다 짧은 에피소드는 제외. 300 권장 (10초 미만은 픽업 완주 불가). "
                        "기본 0 = 제외하지 않고 경고만")
    p.add_argument("--limit", type=int, help="앞 N개 에피소드만 (테스트용)")
    p.add_argument("--hq", action="store_true",
                   help="h264/yuv444p/crf10 로 인코딩 (선 보존 99.9%%, 용량 2.2배)")
    p.add_argument("--png", action="store_true", help="PNG 무손실 저장 (용량 주의)")
    p.add_argument("--writer-threads", type=int, default=4,
                   help="중간 PNG 쓰기 스레드 수 (0=동기). 기본 4")
    p.add_argument("--dry-run", action="store_true", help="쓰지 않고 계획만 출력")
    a = p.parse_args()

    if a.src == a.dst:
        sys.exit("--src 와 --dst 가 같습니다. 원본을 덮어쓸 수 없습니다.")

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    if a.hq:
        _patch_encoder()

    src = LeRobotDataset(a.src)
    keys = list(src.meta.video_keys) or [
        k for k, v in src.meta.features.items() if v["dtype"] in ("image", "video")
    ]
    if len(keys) != 2:
        sys.exit(f"이미지 스트림이 2개가 아닙니다: {keys}")
    print(f"원본  {a.src}  에피소드 {src.num_episodes}  프레임 {src.num_frames}  fps {src.fps}")
    print(f"이미지 스트림: {keys}")

    want = set(range(src.num_episodes))
    if a.episodes:
        want &= parse_ranges(a.episodes)
    if a.skip_episodes:
        want -= parse_ranges(a.skip_episodes)
    todo = sorted(want)
    if a.limit:
        todo = todo[: a.limit]

    # ── 계획 수립 (에피소드별 탑뷰 판별 + task 결정) ──────────────────
    print(f"\n{'ep':>4s} {'프레임':>6s} {'탑뷰':>6s} {'점수':>11s} {'픽업':>4s}  task")
    print("-" * 100)
    plan = []
    for ep_i in todo:
        row = src.meta.episodes[ep_i]
        lo, hi = int(row["dataset_from_index"]), int(row["dataset_to_index"])
        clean, npick, target, dest = classify_task(str(row["tasks"][0]))
        length = hi - lo

        if length < max(a.min_frames, 300):
            note = "⏭ 너무 짧아 제외" if length < a.min_frames else "⚠ 비정상적으로 짧음"
            print(f"{ep_i:4d} {length:6d} {'':6s} {'':11s} {npick:4d}  {note}: {clean!r}")
            if length < a.min_frames:
                continue

        if a.topview == "auto":
            top, scores = detect_topview(src, lo, hi, keys)
            sc = " ".join(f"{k.split('.')[-1][:2]}:{v:.2f}" for k, v in scores.items())
        else:
            top = f"observation.images.{a.topview}"
            sc = "(지정)"

        if target is None or dest is None:
            print(f"{ep_i:4d} {hi-lo:6d} {'':6s} {'':11s} {'':4s}  ⚠ 파싱 실패, 건너뜀: {clean!r}")
            continue

        if a.task_policy == "keep":
            task = clean
        elif npick == 1:
            task = normalize_task(target, dest)
        elif a.task_policy == "single-only":
            print(f"{ep_i:4d} {hi-lo:6d} {'':6s} {'':11s} {npick:4d}  ⏭ 다중 픽업 제외: {clean!r}")
            continue
        else:
            task = clean       # 다중 픽업은 원본 라벨 유지

        plan.append((ep_i, lo, hi, top, task, npick))
        print(f"{ep_i:4d} {hi-lo:6d} {top.split('.')[-1]:>6s} {sc:>11s} {npick:4d}  {task}")

    other = {k for k in keys}
    print(f"\n변환 대상 {len(plan)} 에피소드 / 총 {sum(h-l for _,l,h,_,_,_ in plan):,} 프레임")
    tops = {t for _, _, _, t, _, _ in plan}
    if len(tops) > 1:
        print(f"⚠ 탑뷰 스트림이 에피소드마다 다릅니다: {sorted(tops)}")
        print("  --canonical 이 켜져 있으면 출력에서 교정됩니다."
              if a.canonical else "  --canonical 을 켜서 교정하십시오.")
    norm = sum(1 for *_, n in plan if n == 1)
    print(f"단일 픽업 {norm} / 다중 픽업 {len(plan)-norm}")

    if a.dry_run:
        print("\n--dry-run: 아무것도 쓰지 않았습니다.")
        return
    if not plan:
        sys.exit("변환할 에피소드가 없습니다.")

    # ── 출력 데이터셋 생성 ────────────────────────────────────────────
    from lerobot.datasets.utils import DEFAULT_FEATURES

    feats = {k: v for k, v in src.meta.features.items() if k not in DEFAULT_FEATURES}
    if a.png:
        feats = {k: ({**v, "dtype": "image"} if v["dtype"] == "video" else v)
                 for k, v in feats.items()}

    dst = LeRobotDataset.create(
        repo_id=a.dst, fps=src.fps, features=feats,
        robot_type=src.meta.robot_type,
        use_videos=not a.png,
        vcodec="h264" if a.hq else "libsvtav1",
        image_writer_threads=a.writer_threads,   # PNG 쓰기를 비동기로
    )
    print(f"\n출력  {a.dst}  →  {dst.root}")
    print(f"인코딩: {'PNG 무손실' if a.png else ('h264/yuv444p/crf10 (--hq)' if a.hq else 'libsvtav1/yuv420p/crf30 (기본)')}\n")

    ann = Annotator(a.weights)
    t0 = time.time()
    done = frames_done = 0
    for ep_i, lo, hi, top, task, _ in plan:
        ann.reset()                    # 에피소드 경계에서 트래커 초기화
        hand = next(k for k in keys if k != top)
        target, dest = parse_task(task)
        ep_row = src.meta.episodes[ep_i]

        # 상태·액션은 parquet 배치 접근 (1000행에 13ms — 사실상 공짜)
        cols = src.hf_dataset[lo:hi]
        states = cols["observation.state"]
        actions = cols["action"]

        for off, imgs in iter_episode_images(src, ep_row, keys):
            for j in range(len(imgs[top])):
                i = off + j
                top_img = ann(imgs[top][j], target, dest)
                hand_img = imgs[hand][j]
                if a.canonical:
                    frame = {"observation.images.front": top_img,
                             "observation.images.wrist": hand_img}
                else:
                    frame = {top: top_img, hand: hand_img}
                frame["observation.state"] = np.asarray(states[i], dtype=np.float32)
                frame["action"] = np.asarray(actions[i], dtype=np.float32)
                frame["task"] = task
                dst.add_frame(frame)
        dst.save_episode()
        done += 1
        frames_done += hi - lo
        el = time.time() - t0
        print(f"  ep{ep_i:4d} 완료 ({hi-lo:5d} 프레임)  "
              f"[{done}/{len(plan)}]  {frames_done/el:5.1f} fps  "
              f"경과 {el/60:.1f}분  예상 잔여 {el/done*(len(plan)-done)/60:.1f}분")

    print(f"\n완료: {done} 에피소드 {frames_done:,} 프레임, "
          f"{time.time()-t0:.0f}초 ({frames_done/(time.time()-t0):.1f} fps)")
    print(f"검증:  python -m omx_yolo.verify --repo-id {a.dst} --src {a.src}")


def _patch_encoder() -> None:
    """--hq: pix_fmt / crf 를 런타임에 덮어쓴다. lerobot 소스는 수정하지 않는다.

    LeRobotDataset.create 는 vcodec 만 노출하고 pix_fmt='yuv420p', crf=30 을
    하드코딩한다(lerobot_dataset.py:1708-1711). 인코딩 함수를 감싸서 우회한다.
    """
    from lerobot.datasets import lerobot_dataset as ld

    orig = ld.encode_video_frames

    def patched(*args, **kwargs):
        kwargs["pix_fmt"] = "yuv444p"
        kwargs["crf"] = 10
        return orig(*args, **kwargs)

    ld.encode_video_frames = patched
    print("--hq: 인코딩을 h264 / yuv444p / crf10 으로 덮어썼습니다.")


if __name__ == "__main__":
    main()
