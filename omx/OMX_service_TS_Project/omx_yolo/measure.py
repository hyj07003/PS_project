"""탑뷰 카메라에서 진열대/적재함 ROI 를 실측하는 도구.

opencv-python-headless 환경이라 imshow 를 쓸 수 없다. 대신 자동 분할로
좌표를 제안하고 미리보기 PNG 를 저장하니, 그 이미지를 열어 확인한 뒤
geometry.py 에 값을 옮겨 적으면 된다.

사용법
    # 실제 카메라에서 한 프레임 잡아 측정
    python -m omx_yolo.measure --camera /dev/omx_cam_top

    # 이미 있는 프레임으로 측정
    python -m omx_yolo.measure --image frame.png

    # 경계 y 와 box1 위치를 직접 지정
    python -m omx_yolo.measure --image frame.png --divider-y 281 --box1 lower

    # 현재 geometry.py 상수를 프레임에 그려서 확인만
    python -m omx_yolo.measure --image frame.png --verify

주의: 팔이 적재함을 가리지 않은 프레임을 쓸 것. 에피소드 시작 직후가 좋다.
"""

from __future__ import annotations

import argparse
import sys

import cv2
import numpy as np

from . import geometry
from .annotate import BOX_RGB

# 카드보드 갈색 HSV 범위 — 조명이 바뀌면 조정이 필요할 수 있다
BROWN_LO = (5, 55, 45)
BROWN_HI = (28, 255, 225)
MIN_AREA = 8000


def grab_frame(source: str) -> np.ndarray:
    """카메라 또는 파일에서 RGB 프레임 하나."""
    if source.startswith("/dev/") or source.isdigit():
        dev = int(source) if source.isdigit() else source
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, geometry.FRAME_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, geometry.FRAME_H)
        if not cap.isOpened():
            sys.exit(f"카메라를 열 수 없습니다: {source}")
        for _ in range(10):        # 자동 노출 안정화
            cap.read()
        ok, bgr = cap.read()
        cap.release()
        if not ok:
            sys.exit(f"프레임을 읽을 수 없습니다: {source}")
    else:
        bgr = cv2.imread(source)
        if bgr is None:
            sys.exit(f"이미지를 읽을 수 없습니다: {source}")
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def cardboard_mask(rgb: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    m = cv2.inRange(hsv, BROWN_LO, BROWN_HI)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((13, 13), np.uint8))
    return cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))


def find_union(mask: np.ndarray) -> tuple[int, int, int, int]:
    n, _, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    big = [stats[i] for i in range(1, n) if stats[i][4] > MIN_AREA]
    if not big:
        sys.exit("카드보드 영역을 찾지 못했습니다. BROWN_LO/HI 범위를 조정하세요.")
    x0 = min(s[0] for s in big)
    y0 = min(s[1] for s in big)
    x1 = max(s[0] + s[2] for s in big)
    y1 = max(s[1] + s[3] for s in big)
    return int(x0), int(y0), int(x1), int(y1)


def find_divider(rgb: np.ndarray, union: tuple[int, int, int, int]) -> int:
    """두 적재함 사이의 어두운 가로 경계를 찾는다.

    카드보드는 색이 연속이라 갈색 프로파일로는 경계가 잡히지 않는다.
    대신 어두운 픽셀이 가로로 많은 행을 찾는다.
    """
    x0, y0, x1, y1 = union
    gray = cv2.cvtColor(rgb[:, x0:x1], cv2.COLOR_RGB2GRAY)
    dark = (gray < 70).sum(axis=1).astype(np.float32)
    dark = cv2.GaussianBlur(dark.reshape(-1, 1), (1, 9), 0).ravel()
    lo, hi = y0 + 40, y1 - 40
    if hi <= lo:
        return (y0 + y1) // 2
    return int(np.argmax(dark[lo:hi])) + lo


def draw_preview(rgb, shelf, b1, b2, path) -> None:
    out = rgb.copy()
    for roi, color, label in (
        (shelf, (255, 220, 0), "SHELF_ROI"),
        (b1, BOX_RGB["box1"], "BOX1_ROI"),
        (b2, BOX_RGB["box2"], "BOX2_ROI"),
    ):
        x0, y0, x1, y1 = roi
        cv2.rectangle(out, (x0, y0), (x1, y1), color, 3, lineType=cv2.LINE_8)
        cv2.putText(out, label, (x0 + 5, max(16, y0 + 18)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    cv2.imwrite(path, cv2.cvtColor(out, cv2.COLOR_RGB2BGR))
    print(f"\n미리보기 저장: {path}\n  이 이미지를 열어 세 영역이 맞는지 눈으로 확인하십시오.")


def main() -> None:
    p = argparse.ArgumentParser(description="탑뷰 ROI 실측")
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--camera", help="/dev/omx_cam_top 또는 인덱스")
    src.add_argument("--image", help="프레임 이미지 경로")
    p.add_argument("--divider-y", type=int, help="두 적재함 사이 경계 y (자동 탐지 무시)")
    p.add_argument("--box1", choices=["upper", "lower"], default="lower",
                   help="box1 이 상단인지 하단인지 (기본: lower)")
    p.add_argument("--out", default="roi_preview.png", help="미리보기 저장 경로")
    p.add_argument("--verify", action="store_true",
                   help="측정하지 않고 현재 geometry.py 상수만 그려서 확인")
    a = p.parse_args()

    rgb = grab_frame(a.camera or a.image)
    h, w = rgb.shape[:2]
    if (w, h) != (geometry.FRAME_W, geometry.FRAME_H):
        print(f"경고: 프레임이 {w}x{h} 입니다. geometry.py 는 "
              f"{geometry.FRAME_W}x{geometry.FRAME_H} 를 전제합니다.")

    if a.verify:
        print("현재 geometry.py 상수:")
        print(f"  SHELF_ROI = {geometry.SHELF_ROI}")
        print(f"  BOX1_ROI  = {geometry.BOX1_ROI}")
        print(f"  BOX2_ROI  = {geometry.BOX2_ROI}")
        if geometry.warn_unverified():
            print(f"  미검증: {', '.join(geometry.warn_unverified())}")
        draw_preview(rgb, geometry.SHELF_ROI, geometry.BOX1_ROI,
                     geometry.BOX2_ROI, a.out)
        return

    union = find_union(cardboard_mask(rgb))
    ux0, uy0, ux1, uy1 = union
    div = a.divider_y if a.divider_y is not None else find_divider(rgb, union)

    shelf = (ux1, 0, w, h)
    upper = (ux0, uy0, ux1, div)
    lower = (ux0, div, ux1, uy1)
    b1, b2 = (lower, upper) if a.box1 == "lower" else (upper, lower)

    print("=== 측정 결과 ===")
    print(f"  카드보드 합집합   {union}")
    print(f"  적재함 경계 y     {div}" + ("  (자동 탐지)" if a.divider_y is None else "  (지정값)"))
    print(f"  box1 위치         {a.box1}")
    print("\n=== geometry.py 에 붙여넣을 값 ===")
    print(f"CARDBOARD_UNION = {union}")
    print(f"SHELF_ROI = {shelf}")
    print(f"BOX1_ROI = {b1}")
    print(f"BOX2_ROI = {b2}")
    print("\nVERIFIED = {")
    for k in ("CARDBOARD_UNION", "SHELF_ROI", "BOX1_ROI", "BOX2_ROI"):
        print(f'    "{k}": True,')
    print("}")
    draw_preview(rgb, shelf, b1, b2, a.out)
    print("\n★ box1/box2 가 뒤바뀌어 보이면 --box1 을 반대로 주고 다시 실행하십시오.")


if __name__ == "__main__":
    main()
