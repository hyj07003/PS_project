#!/usr/bin/env python3
"""탑뷰 카메라를 기준 배치에 맞출 때 쓰는 실시간 표시기.

    PYTHONPATH=~/il_ws/src ~/venv/il/bin/python ~/il_ws/scripts/align_rig.py

카메라를 조금씩 움직이면서 화면의 숫자를 보고 맞춘다. checkrig 를 반복해서
치는 것보다 훨씬 빠르다.

왜 카메라를 움직이는가 — 진열대는 책상에 테이프로 고정돼 있다. 그런데도
화면에서 진열대와 적재함이 **함께** 밀렸다면 움직인 것은 카메라다
(2026-08-21: 진열대 +15,+20 · 적재함 +18,+8 — 원근 차이를 감안하면 같은 방향).

진열대만 맞추면 안 된다. 적재함 ROI(geometry.py)와 정책이 보는 주석이 함께
어긋나 있으므로, 카메라를 되돌려야 셋이 한 번에 맞는다.
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np


def main() -> None:
    ap = argparse.ArgumentParser(description="탑뷰 카메라 정렬 도우미")
    ap.add_argument("--camera", default="/dev/omx_cam_top")
    ap.add_argument("--ref", default="/home/newuser/il_ws/models/rig_reference.json")
    ap.add_argument("--interval", type=float, default=0.8)
    ap.add_argument("--good", type=float, default=8.0,
                    help="이 값 아래면 맞은 것으로 본다 (checkrig 와 같은 기준)")
    a = ap.parse_args()

    import cv2
    import json
    from omx_yolo.checkrig import compare, find_cells

    ref = [tuple(c) for c in json.load(open(a.ref))["cells"]]
    cap = cv2.VideoCapture(a.camera, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    print("카메라를 조금씩 움직이십시오. Ctrl+C 로 종료합니다.")
    print(f"목표: 평균 이동량 {a.good:.0f}px 아래\n")
    print(f"{'평균':>7} {'최대':>7} {'매칭':>6}  {'가로':>7} {'세로':>7}  진행")

    best = None
    try:
        while True:
            t0 = time.time()
            frame = None
            while time.time() - t0 < 0.4:      # 워밍업이 필요한 카메라다
                ok, f = cap.read()
                if ok:
                    frame = f
            if frame is None:
                print("  프레임 없음 — 다른 프로세스가 카메라를 잡고 있습니까?")
                time.sleep(a.interval)
                continue

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            try:
                cur = find_cells(rgb)
            except Exception as exc:                  # noqa: BLE001
                print(f"  칸 검출 실패: {exc}")
                time.sleep(a.interval)
                continue
            if len(cur) != len(ref):
                print(f"  칸 {len(cur)}개 검출 (기준 {len(ref)}개) — 화면을 가리는 것이 있습니까?")
                time.sleep(a.interval)
                continue

            mean, mx, match = compare(cur, ref)
            # 어느 쪽으로 밀렸는지도 알려 준다. 짝지어진 순서가 같다고 보고
            # 평균 변위를 낸다 — 방향만 보면 되므로 이 정도로 충분하다.
            d = np.array(cur, float)[:len(ref)] - np.array(ref, float)
            dx, dy = d[:, 0].mean(), d[:, 1].mean()

            arrow = ("←" if dx > 2 else "→" if dx < -2 else "·") + \
                    ("↑" if dy > 2 else "↓" if dy < -2 else "·")
            hint = f"카메라를 {arrow} 로 옮기십시오" if mean >= a.good else "맞았습니다"
            if best is None or mean < best:
                best = mean
            mark = "  ★최고" if mean == best else ""
            print(f"{mean:7.1f} {mx:7.1f} {match*100:5.0f}%  "
                  f"{dx:+7.1f} {dy:+7.1f}  {hint}{mark}")
            time.sleep(a.interval)
    except KeyboardInterrupt:
        print(f"\n종료. 이번 세션 최소 평균 이동량 {best:.1f}px" if best else "\n종료.")
    finally:
        cap.release()


if __name__ == "__main__":
    main()
