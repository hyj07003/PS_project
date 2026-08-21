"""적재함이 비었는지 판정한다 — 포장 작업의 종료 조건.

왜 이 방식인가 — 담긴 **개수**를 세려던 시도는 실패했다(2026-08-21).
관절값으로는 물건을 문 것과 박스 벽을 문 것이 갈리지 않고, 정책이 명령하는
그리퍼 위치가 도달 불가능한 값이라 차이(stall)가 상수처럼 나온다. 여섯 가지
규칙을 정답과 대조했으나 하나도 맞지 않았다.

대신 질문을 바꾼다:

    "몇 개 담았는가"  →  "적재함이 비었는가"

이진 판단이라 훨씬 견고하고, **그게 곧 종료 조건**이다. 개수를 못 세도
작업은 완결된다.

검출기는 픽업에서 쓰던 것을 그대로 쓴다(`models/omx_goods_yolo11n.pt`).
그 검출기는 포장 탑뷰에서 분류를 자주 틀린다 — 소시지를 biscuit 으로 읽는
식이다. 하지만 **여기서는 그래도 된다.** 무엇인지가 아니라 있는지만 물으므로
오분류가 답을 바꾸지 않는다. 개수 세기에 필요했던 정확도가 필요 없어지면서
쓸 수 없던 검출기가 쓸 수 있게 됐다.

2026-08-21 실측: 물건 2개가 든 적재함에서 biscuit 0.99 / 0.91, 빈 적재함에서
검출 0개. conf 0.5 와 0.7 모두 같은 답을 냈다.

**가림 문제** — 팔이 적재함 위에 있으면 안을 볼 수 없다. 팔이 가린 상태를
"비었다" 로 읽으면 작업을 중간에 끊는다. 그래서 판정은 팔이 적재함에서
벗어난 순간에만 하고(`arm_is_clear`), 연속으로 여러 번 같은 답이 나와야
받아들인다.
"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

logger = logging.getLogger(__name__)

WEIGHTS = "/home/newuser/il_ws/models/omx_goods_yolo11n.pt"
# 픽업 annotate.KEEP_CLASSES 와 같다 — coke(2) 와 yogurt(7) 는 오검출이 많아 뺀다.
KEEP_CLASSES = [0, 1, 3, 4, 5, 6]

# 적재함 내부 ROI (탑뷰 픽셀, x1,y1,x2,y2).
#
# 2026-08-21 프레임에서 눈으로 읽은 잠정값이다. 카메라나 상자가 움직이면
# 다시 잡아야 한다 — `python -m omx_pack.boxcheck --measure` 로 지금 프레임에
# ROI 를 그려 확인할 수 있다.
#
# 2026-08-21 사용자 확인: 물건이 들어 있던 위쪽 상자가 box1 이다.
# box1 = cart-1 = 팔에 가까운 적재함(→ 픽업 쪽 vocab 과 같은 규칙).
ROI_PATH = Path("/home/newuser/il_ws/models/pack_box_roi.json")
DEFAULT_ROIS: dict[str, tuple[int, int, int, int]] = {
    "box1": (395, 130, 585, 292),
    "box2": (383, 296, 585, 462),
}

# 관제 deviceCode 로도 부를 수 있게 한다 — 서버가 요청의 deviceCode 를 그대로
# 넘기면 되도록. vocab.CONTROLLER_DEVICE_BASKET 와 짝이 맞아야 한다.
DEVICE_BOX = {"cart-1": "box1", "cart-2": "box2"}

# ── 가림 판정 ──────────────────────────────────────────────────────────
#
# 팔이 적재함 위에 있으면 안이 안 보인다. 그 상태를 "비었다" 로 읽으면
# 작업을 중간에 끊는다.
#
# 처음에는 관절로 대신 판단했다(shoulder_pan 이 음수면 팔이 바구니 쪽이니
# 안 가린다고 봄). 되기는 했지만 두 가지가 나빴다. 적재함이 비면 팔이
# 바구니로 갈 일이 없어 판정 기회가 3% 밖에 안 생겼고(실측: 첫 기회 33.3초),
# 애초에 "팔이 어디 있는가" 는 "가리는가" 의 대용품일 뿐이다.
#
# 지금은 **화면에서 직접 잰다.** ROI 안의 어두운 픽셀 비율이다 — 팔과
# 그리퍼가 검은색이라 가리면 이 값이 올라간다.
#
# 2026-08-21 실측(비스킷 3개가 든 적재함, 40프레임):
#     오판("비었음")이 난 16장   어두운 픽셀 36.4% ~ 65.6%
#     30% 미만인 18장            전부 "물건 있음" 으로 정답
# 30% 를 기준으로 삼으면 오판 최소치와 6.4포인트 여유가 있고, 판정 기회는
# 프레임의 45% 로 늘어난다.
DARK_LEVEL = 70          # 이보다 어두우면 팔·그리퍼로 본다 (0~255)
MAX_DARK = 0.30          # ROI 의 이 비율을 넘게 가리면 판정하지 않는다

# 옛 관절 기준. 더 쓰지 않지만 궤적을 다시 볼 때 참고가 되므로 남긴다.
CLEAR_PAN_MAX = -5.0


def load_rois() -> dict[str, tuple[int, int, int, int]]:
    if ROI_PATH.exists():
        return {k: tuple(v) for k, v in json.loads(ROI_PATH.read_text()).items()}
    return dict(DEFAULT_ROIS)


def resolve_box(name: str) -> str:
    """box1/box2 또는 cart-1/cart-2 를 ROI 이름으로 바꾼다."""
    key = (name or "").strip().lower()
    return DEVICE_BOX.get(key, key)


def save_rois(rois: dict) -> None:
    ROI_PATH.parent.mkdir(parents=True, exist_ok=True)
    ROI_PATH.write_text(json.dumps({k: list(v) for k, v in rois.items()},
                                   ensure_ascii=False, indent=2))


def arm_is_clear(state: np.ndarray) -> bool:
    """[구식] 관절로 가림을 짐작한다. roi_is_visible 을 쓸 것."""
    return float(state[0]) <= CLEAR_PAN_MAX


def occlusion(frame_rgb: np.ndarray, roi: tuple[int, int, int, int]) -> float:
    """ROI 안에서 어두운 픽셀이 차지하는 비율 (0~1). 팔이 가리면 올라간다."""
    import cv2

    x1, y1, x2, y2 = roi
    crop = np.asarray(frame_rgb)[y1:y2, x1:x2]
    if crop.size == 0:
        return 1.0
    gray = cv2.cvtColor(crop, cv2.COLOR_RGB2GRAY)
    return float((gray < DARK_LEVEL).mean())


def roi_is_visible(frame_rgb: np.ndarray, roi: tuple[int, int, int, int],
                   max_dark: float = MAX_DARK) -> tuple[bool, float]:
    """적재함 안을 판정해도 될 만큼 보이는가. (보이는가, 가림비율)."""
    d = occlusion(frame_rgb, roi)
    return d <= max_dark, d


@dataclass
class BoxChecker:
    """적재함 ROI 안에 상품이 보이는지 판정한다."""

    roi: tuple[int, int, int, int]
    conf: float = 0.5
    weights: str = WEIGHTS
    _model: object = field(default=None, init=False)

    def _load(self):
        if self._model is None:
            from ultralytics import YOLO

            self._model = YOLO(self.weights)
        return self._model

    def detect(self, frame_rgb: np.ndarray) -> list[dict]:
        """ROI 안의 검출만 돌려준다. 입력은 RGB(카메라가 주는 형식)."""
        import cv2

        model = self._load()
        bgr = cv2.cvtColor(np.asarray(frame_rgb), cv2.COLOR_RGB2BGR)
        r = model.predict(bgr, classes=KEEP_CLASSES, conf=self.conf,
                          verbose=False)[0]
        x1, y1, x2, y2 = self.roi
        out = []
        for b in r.boxes:
            bx1, by1, bx2, by2 = b.xyxy[0].tolist()
            cx, cy = (bx1 + bx2) / 2, (by1 + by2) / 2
            if x1 <= cx <= x2 and y1 <= cy <= y2:
                out.append({"cls": r.names[int(b.cls)], "conf": float(b.conf),
                            "center": (round(cx, 1), round(cy, 1))})
        return out

    def is_empty(self, frame_rgb: np.ndarray) -> tuple[bool, list[dict]]:
        det = self.detect(frame_rgb)
        return (not det), det


def main() -> None:
    import argparse

    import cv2

    ap = argparse.ArgumentParser(description="적재함 비었는지 판정 / ROI 확인")
    ap.add_argument("--image", default=None, help="프레임 파일로 판정")
    ap.add_argument("--camera", default="/dev/omx_cam_pack_top",
                    help="카메라에서 한 프레임 잡아 판정")
    ap.add_argument("--conf", type=float, default=0.5)
    ap.add_argument("--measure", action="store_true",
                    help="ROI 를 그린 미리보기를 저장한다 (좌표 확인용)")
    ap.add_argument("--out", default="/tmp/pack_box_check.jpg")
    a = ap.parse_args()

    if a.image:
        bgr = cv2.imread(a.image)
    else:
        import time

        cap = cv2.VideoCapture(a.camera, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        t0, bgr = time.time(), None
        while time.time() - t0 < 4.0:          # 워밍업이 필요한 카메라다
            ok, f = cap.read()
            if ok:
                bgr = f
        cap.release()
    if bgr is None:
        raise SystemExit("프레임을 얻지 못했습니다.")

    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    rois = load_rois()
    vis = bgr.copy()
    for name, roi in rois.items():
        ck = BoxChecker(roi=roi, conf=a.conf)
        empty, det = ck.is_empty(rgb)
        x1, y1, x2, y2 = roi
        color = (90, 200, 90) if empty else (60, 140, 255)
        cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
        cv2.putText(vis, f"{name}: {'EMPTY' if empty else f'{len(det)} items'}",
                    (x1, max(y1 - 8, 14)), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    color, 2, cv2.LINE_AA)
        print(f"{name:8s} ROI {roi} → "
              + ("비었음" if empty else f"물건 {len(det)}개  "
                 + ", ".join(f"{d['cls']} {d['conf']:.2f}" for d in det)))
        for d in det:
            cx, cy = d["center"]
            cv2.circle(vis, (int(cx), int(cy)), 5, color, -1)
    cv2.imwrite(a.out, vis)
    print(f"\n미리보기: {a.out}")


if __name__ == "__main__":
    main()
