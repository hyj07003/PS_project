"""리그 배치가 기준과 같은지 10초 만에 확인한다.

    # 기준 저장 (리그를 고정한 직후 한 번)
    python -m omx_yolo.checkrig --save-reference

    # 매 수집·추론 세션 시작 전에
    python -m omx_yolo.checkrig

    # 과거 데이터셋과 비교
    python -m omx_yolo.checkrig --against-dataset kdy93/smart_market_prototype_4 --episode 15

왜 필요한가
────────────────────────────────────────────────────────────────────────
카메라나 진열대가 조금만 움직여도, 그 전에 모은 데이터로 학습한 정책은
에러 없이 조용히 실패한다. 그런데 움직였는지 알 방법이 없으면 원인을
찾을 수도 없다.

실제로 하루 사이에 배치가 43px 이동해 판정기가 깨졌고, 원인을 찾는 데
시간을 썼다. 그때 이 도구가 있었다면 10초에 알았을 것이다.

무엇을 비교하는가
────────────────────────────────────────────────────────────────────────
검정 테이프로 나뉜 진열대 흰 칸 6개의 중심 좌표를 쓴다.

이게 올바른 기준인 이유 (사용자 확인):
  · 검정 테이프 경계선의 위치는 데이터셋을 통틀어 변하지 않는다.
  · 반면 물체는 데이터 다양성을 위해 칸 안에서 각도·위치를 조금씩 바꿔
    배치했다. 다만 자기 칸을 벗어나지는 않는다.

즉 물체는 움직이고 칸은 고정이므로, 칸 위치가 달라졌다면 그건 물체 배치
때문이 아니라 카메라나 진열대 판 자체가 움직인 것이다.

이 값이 중요한 이유: 팔이 실제로 손을 뻗어야 하는 곳이 바로 이 칸들이다.
칸이 움직였다면 예전 시연의 관절 궤적이 더 이상 맞지 않는다.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

from .geometry import SHELF_ROI, in_roi

REF_PATH = Path("/home/newuser/il_ws/models/rig_reference.json")

# 흰 칸 검출 임계
WHITE_LO = (0, 0, 140)
WHITE_HI = (180, 70, 255)
MIN_CELL_AREA = 2500
MAX_CELL_AREA = 40000
N_ROWS, N_COLS = 2, 3          # 진열대 격자 (2행 × 3열)
N_CELLS = N_ROWS * N_COLS
ROW_TOL = 60         # 같은 행으로 묶을 y 허용 오차(px)

# 합격 기준 (픽셀)
TOL_GOOD = 8.0
TOL_WARN = 20.0


def find_cells(rgb: np.ndarray) -> list[tuple[float, float]]:
    """진열대 흰 칸들의 중심 좌표. 물체 유무와 무관하게 판의 구조만 본다.

    후보는 SHELF_ROI 안으로 제한한다. 적재함이 비면 카드보드 바닥이 넓은
    밝은 덩어리로 잡혀 흰 칸으로 오인되기 때문이다(2026-08-19 실측: 6번 칸이
    x=592 에서 x=78 로 515px 튐). 진열대 칸은 정의상 SHELF_ROI 안에 있으므로
    이 제한으로 잃는 것이 없다.
    """
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    m = cv2.inRange(hsv, WHITE_LO, WHITE_HI)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, _, stats, cent = cv2.connectedComponentsWithStats(m, 8)
    cand = []
    for i in range(1, n):
        area = int(stats[i][4])
        w, h = int(stats[i][2]), int(stats[i][3])
        if not (MIN_CELL_AREA < area < MAX_CELL_AREA):
            continue
        if w < 25 or h < 25:
            continue
        if max(w, h) / max(min(w, h), 1) > 3.5:       # 지나치게 길쭉한 것 제외
            continue
        cx, cy = float(cent[i][0]), float(cent[i][1])
        if not in_roi(SHELF_ROI, cx, cy):              # 적재함 쪽 오검출 차단
            continue
        cand.append((cx, cy, area))

    return _pick_grid(cand)


def _pick_grid(cand: list[tuple[float, float, int]]) -> list[tuple[float, float]]:
    """후보 중에서 2행 × 3열 격자를 이루는 6개를 고른다.

    면적만으로는 고를 수 없다. 프레임 상단의 흰 배경 조각들이 진열대 칸과
    면적이 비슷하게 잡히기 때문이다(실측: 가짜 7979·6669 vs 진짜 9675·9498).

    구분되는 성질은 x 간격의 규칙성이다. 진열대 한 행의 세 칸은 등간격이지만
    (실측 127, 124), 배경 조각들은 그렇지 않다(272, 208).
    """
    if len(cand) <= N_CELLS:
        return sorted([(x, y) for x, y, _ in cand], key=lambda c: (round(c[1] / 60), c[0]))

    # y 로 행 묶기
    rows: list[list[tuple[float, float, int]]] = []
    for c in sorted(cand, key=lambda c: c[1]):
        if rows and abs(c[1] - np.mean([p[1] for p in rows[-1]])) < ROW_TOL:
            rows[-1].append(c)
        else:
            rows.append([c])

    def regularity(row) -> float:
        """x 간격이 고를수록 0 에 가깝다. 칸이 3개가 아니면 큰 값."""
        if len(row) < N_COLS:
            return 1e9
        xs = sorted(p[0] for p in row)[:N_COLS]
        gaps = np.diff(xs)
        return float(np.std(gaps) / max(np.mean(gaps), 1e-6))

    good = sorted((r for r in rows if len(r) >= N_COLS), key=regularity)[:N_ROWS]
    if len(good) < N_ROWS:                       # 격자를 못 찾으면 원래 방식으로
        med = float(np.median([c[2] for c in cand]))
        cand = sorted(cand, key=lambda c: abs(c[2] - med))[:N_CELLS]
        return sorted([(x, y) for x, y, _ in cand], key=lambda c: (round(c[1] / 60), c[0]))

    cells = []
    for row in good:
        for x, y, _ in sorted(row, key=lambda p: p[0])[:N_COLS]:
            cells.append((x, y))
    return sorted(cells, key=lambda c: (round(c[1] / 60), c[0]))


def grab(device: str = "/dev/omx_cam_top", warmup_s: float = 2.0) -> np.ndarray:
    import time

    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        sys.exit(f"카메라를 열 수 없습니다: {device}")
    last = None
    t0 = time.time()
    while time.time() - t0 < warmup_s:
        ok, f = cap.read()
        if ok:
            last = f
    cap.release()
    if last is None:
        sys.exit("프레임을 읽지 못했습니다.")
    return cv2.cvtColor(last, cv2.COLOR_BGR2RGB)


def draw_cells(rgb: np.ndarray, cells: list, path: str,
               ref: list | None = None, title: str = "") -> None:
    """검출된 칸을 그려 저장한다. 눈으로 확인하라고 만드는 파일이다.

    빨강 = 이번에 검출된 칸, 파랑 = 기준으로 저장된 칸.
    둘이 겹쳐 보이면 리그가 그대로다.

    title 은 반드시 ASCII 로 줄 것 — cv2.putText 는 한글을 ???? 로 그린다.
    """
    vis = rgb.copy()
    if ref:
        for rx, ry in ref:
            cv2.drawMarker(vis, (int(rx), int(ry)), (60, 130, 255),
                           cv2.MARKER_CROSS, 26, 2, cv2.LINE_AA)
    rows = {}
    for x, y in cells:
        rows.setdefault(round(y / ROW_TOL), []).append((x, y))
    for r in rows.values():                        # 같은 행끼리 선으로 연결
        r = sorted(r)
        for a_, b_ in zip(r, r[1:]):
            cv2.line(vis, (int(a_[0]), int(a_[1])), (int(b_[0]), int(b_[1])),
                     (255, 60, 60), 1, cv2.LINE_AA)
    for i, (x, y) in enumerate(cells):
        cv2.circle(vis, (int(x), int(y)), 15, (255, 40, 40), 3, cv2.LINE_AA)
        cv2.putText(vis, str(i + 1), (int(x) + 18, int(y) + 6),
                    cv2.FONT_HERSHEY_SIMPLEX, .62, (255, 40, 40), 2, cv2.LINE_AA)
    if title:
        cv2.rectangle(vis, (0, 0), (vis.shape[1], 26), (20, 20, 20), -1)
        cv2.putText(vis, title, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, .5,
                    (240, 240, 240), 1, cv2.LINE_AA)
    cv2.imwrite(path, cv2.cvtColor(vis, cv2.COLOR_RGB2BGR))
    print(f"\n확인용 이미지: {path}")
    print("   빨간 원 6개가 진열대 6칸 한가운데에 찍혔는지 눈으로 보십시오."
          + ("\n   파란 십자 = 기준 위치. 빨간 원과 겹쳐야 정상입니다." if ref else ""))


def sanity(cells: list) -> list[str]:
    """검출 결과가 2행×3열 격자답게 생겼는지 자가 점검."""
    msgs = []
    if len(cells) != N_CELLS:
        msgs.append(f"칸이 {len(cells)}개입니다 (6개여야 함)")
        return msgs
    ys = sorted(y for _, y in cells)
    groups = 1 + sum(1 for a_, b_ in zip(ys, ys[1:]) if b_ - a_ > ROW_TOL)
    if groups != N_ROWS:
        msgs.append(f"행이 {groups}개로 나뉩니다 (2개여야 함) — 진열대 밖을 잡았을 수 있습니다")
    for ri in (0, 1):
        row = sorted(x for x, y in cells if (y > sorted(ys)[2]) == bool(ri))
        if len(row) == N_COLS:
            gaps = np.diff(row)
            if np.std(gaps) / max(np.mean(gaps), 1e-6) > 0.25:
                msgs.append(f"{ri+1}행 칸 간격이 불규칙합니다 {[round(g) for g in gaps]}")
    return msgs


def compare(cur: list, ref: list) -> tuple[float, float, float]:
    """가장 가까운 칸끼리 짝지어 평균/최대 이동량과 매칭률을 낸다."""
    if not cur or not ref:
        return float("nan"), float("nan"), 0.0
    d = []
    for rx, ry in ref:
        best = min(((cx - rx) ** 2 + (cy - ry) ** 2) ** 0.5 for cx, cy in cur)
        d.append(best)
    d = np.array(d)
    return float(d.mean()), float(d.max()), float((d < TOL_WARN).mean())


def main() -> None:
    p = argparse.ArgumentParser(description="리그 배치 확인")
    p.add_argument("--camera", default="/dev/omx_cam_top")
    p.add_argument("--image", help="카메라 대신 이미지 파일")
    p.add_argument("--save-reference", action="store_true", help="현재 배치를 기준으로 저장")
    p.add_argument("--against-dataset", help="과거 데이터셋과 비교")
    p.add_argument("--episode", type=int, default=0)
    p.add_argument("--ref", default=str(REF_PATH))
    p.add_argument("--viz", default="/home/newuser/il_ws/models/rig_check.png",
                   help="검출 결과 확인용 이미지 저장 경로")
    a = p.parse_args()

    if a.against_dataset:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        from lerobot.datasets.video_utils import decode_video_frames

        from .convert import DECODE_TOL, video_path

        ds = LeRobotDataset(a.against_dataset)
        key = ("observation.images.front" if "observation.images.front" in ds.meta.video_keys
               else ds.meta.video_keys[0])
        row = ds.meta.episodes[a.episode]
        fr = decode_video_frames(video_path(ds, key, row),
                                 [float(row[f"videos/{key}/from_timestamp"]) + 1.0],
                                 tolerance_s=DECODE_TOL)
        img = (fr[0].permute(1, 2, 0).numpy() * 255).round().astype(np.uint8)
        ref_cells = find_cells(img)
        label = f"{a.against_dataset} ep{a.episode}"
    else:
        if a.save_reference:
            img = grab(a.camera) if not a.image else \
                cv2.cvtColor(cv2.imread(a.image), cv2.COLOR_BGR2RGB)
            cells = find_cells(img)
            Path(a.ref).parent.mkdir(parents=True, exist_ok=True)
            json.dump({"cells": cells}, open(a.ref, "w"), indent=2)
            print(f"기준 저장: {a.ref}   흰 칸 {len(cells)}개")
            for i, (x, y) in enumerate(cells):
                print(f"   칸{i+1}  ({x:6.1f}, {y:6.1f})")
            for w in sanity(cells):
                print(f"\n⚠ {w}")
            draw_cells(img, cells, a.viz,
                       title="SAVED REFERENCE  -  6 shelf cells detected")
            return
        if not Path(a.ref).exists():
            sys.exit(f"기준이 없습니다: {a.ref}\n먼저 --save-reference 로 저장하십시오.")
        ref_cells = [tuple(c) for c in json.load(open(a.ref))["cells"]]
        label = a.ref

    cur_img = (grab(a.camera) if not a.image
               else cv2.cvtColor(cv2.imread(a.image), cv2.COLOR_BGR2RGB))
    cur_cells = find_cells(cur_img)

    mean_d, max_d, match = compare(cur_cells, ref_cells)
    print(f"기준: {label}   칸 {len(ref_cells)}개")
    print(f"현재: 칸 {len(cur_cells)}개\n")
    for w in sanity(cur_cells):
        print(f"  ⚠ {w}")
    print(f"  평균 이동량 {mean_d:6.1f} px")
    print(f"  최대 이동량 {max_d:6.1f} px")
    print(f"  매칭률      {match*100:5.1f} %\n")
    draw_cells(cur_img, cur_cells, a.viz, ref=ref_cells,
               title=f"RED=now  BLUE=reference   mean shift {mean_d:.1f}px")

    if mean_d < TOL_GOOD:
        print("  ✅ 기준과 일치합니다. 그대로 진행하십시오.")
    elif mean_d < TOL_WARN:
        print("  ⚠ 약간 어긋났습니다. 정밀도가 필요한 작업이면 맞추십시오.")
    else:
        print("  ❌ 크게 어긋났습니다.")
        print("     이 상태로 모은 데이터는 기준 데이터와 섞이지 않습니다.")
        print("     맞추거나, 지금 배치를 새 기준으로 저장하고 처음부터 다시 모으십시오.")


if __name__ == "__main__":
    main()
