"""학습 데이터 변환과 실시간 추론이 공유하는 단일 주석 구현.

이 파일의 로직을 다른 곳에 복제하지 말 것. 학습 때의 주석과 추론 때의 주석이
한 픽셀이라도 다르면 정책은 예외 없이 조용히 실패한다. 변환 스크립트
(convert.py)와 카메라 래퍼(camera.py)가 모두 여기의 Annotator 를 import 해서
쓰는 것이 이 구조의 핵심이다.

시각 인코딩 규약 (변경하면 기존 주석 데이터셋 전부 무효)
    선 두께      타겟 4px / 비타겟 2px
                 SmolVLA 가 640x480 을 512x512 로 패딩 리사이즈하므로
                 (스케일 0.8) 각각 3.2px / 1.6px 로 정책에 들어간다.
    안티앨리어싱  끔 (LINE_8). 반투명 경계 픽셀은 영상 압축에서 먼저 소실된다.
    채우기        금지. 작은 미니어처의 유효 픽셀을 가리면 손해가 크다.
    텍스트        금지. 512 리사이즈 후 판독 불가.
    색            클래스 전용 채널. 강조는 두께로만 한다.
"""

from __future__ import annotations

import cv2
import numpy as np
from ultralytics import YOLO

from .geometry import BOX1_ROI, BOX2_ROI, SHELF_ROI, in_roi

# ── 상품 6종 (RGB). coke / yogurt 는 이 프로젝트에 없으므로 의도적으로 제외 ──
RGB: dict[str, tuple[int, int, int]] = {
    "sandwich": (220, 84, 32),
    "milk": (246, 176, 28),
    "icecream": (40, 96, 188),
    "cake": (150, 62, 196),
    "biscuit": (54, 168, 96),
    "roll": (23, 176, 184),
}

# 적재함 색 — 상품 6색과 겹치지 않고 갈색 카드보드 위에서 잘 보이는 두 색
BOX_RGB: dict[str, tuple[int, int, int]] = {
    "box1": (235, 60, 170),    # 마젠타
    "box2": (250, 250, 250),   # 흰색
}

# ── 자연어 지시문에 쓸 상품 표기 ─────────────────────────────────────
# 이 표가 유일한 출처다. 수집(사람이 --dataset.single_task 에 입력),
# 변환(convert.normalize_task), 서버(server.build_task) 가 전부 여기를 본다.
# 한 곳이라도 다르면 정책이 학습한 적 없는 문장을 받아 성능이 떨어진다.
#
# 왼쪽 = YOLO 클래스명(내부용), 오른쪽 = 지시문에 쓸 표기.
#
# 표기를 바꾸려면 여기만 고치면 되지만, 이미 수집을 시작했다면 바꾸지 말 것 —
# 기존 에피소드의 task 문자열과 어긋난다.
PRODUCT_PHRASE: dict[str, str] = {
    "sandwich": "sandwich",
    "milk": "milk carton",
    "icecream": "icecream",
    "cake": "cake",
    "biscuit": "biscuit",
    "roll": "roll",
}


# ── 관제 서버(controller-server) 의 상품 slug ↔ OMX 클래스명 ──────────
# 관제는 DB 의 products.slug 로 상품을 지목한다(seed.py). 그 표기가 OMX 의
# 클래스명과 다르므로 여기서 한 번만 변환한다.
#
#   관제 slug   웨이포인트   OMX 클래스   지시문 표기
#   sandwich    W3          sandwich     "sandwich"
#   milk        W5          milk         "milk carton"     ← 표기 다름
#   ice-cream   W4          icecream     "icecream"        ← 하이픈
#   roll-cake   W2          roll         "roll"            ← 이름 다름
#   cake        W1          cake         "cake"
#   biscuit     (신규)       biscuit      "biscuit"
#
# cola(W6) 는 없다. 검출기의 KEEP_CLASSES 에서 coke 를 오검출 때문에 제외했고
# 정책도 학습한 적이 없다. 관제 카탈로그에서 cola 를 빼고 biscuit 을 넣기로
# 합의했다(2026-08-20). 그래도 요청이 들어오면 400 으로 거절한다.
#
# 이 표가 유일한 출처다. 2026-08-19 에 "milk" 와 "milk carton" 이 어긋나
# 정책이 학습한 적 없는 지시문을 받고 5 에피소드를 통째로 버렸다.
CONTROLLER_SLUG: dict[str, str] = {
    "sandwich": "sandwich",
    "milk": "milk",
    "ice-cream": "icecream",
    "roll-cake": "roll",
    "cake": "cake",
    "biscuit": "biscuit",
}

# 관제의 카트 코드 ↔ OMX 적재함. box1 이 로봇 팔과 가까운 쪽이다.
CONTROLLER_DEVICE_BOX: dict[str, str] = {
    "cart-1": "box1",
    "cart-2": "box2",
}


def resolve_slug(slug: str) -> str:
    """관제 slug 를 OMX 클래스명으로. 모르는 값이면 ValueError."""
    key = str(slug).strip().lower()
    if key in CONTROLLER_SLUG:
        return CONTROLLER_SLUG[key]
    if key in PRODUCT_PHRASE:            # OMX 클래스명을 그대로 준 경우도 허용
        return key
    raise ValueError(
        f"지원하지 않는 상품입니다: {slug!r} "
        f"(가능: {sorted(set(CONTROLLER_SLUG) | set(PRODUCT_PHRASE))})")


def resolve_device(device_code: str) -> str:
    """관제 카트 코드를 적재함 이름으로. box1/box2 를 직접 줘도 허용."""
    key = str(device_code).strip().lower()
    if key in CONTROLLER_DEVICE_BOX:
        return CONTROLLER_DEVICE_BOX[key]
    if key in ("box1", "box2"):
        return key
    raise ValueError(
        f"지원하지 않는 카트입니다: {device_code!r} "
        f"(가능: {sorted(CONTROLLER_DEVICE_BOX)} 또는 box1/box2)")


def build_task(product: str, box: str) -> str:
    """(클래스명, 적재함) → 지시문. 프로젝트 전체가 이 함수만 쓴다."""
    if product not in PRODUCT_PHRASE:
        raise ValueError(f"모르는 상품: {product!r} (가능: {list(PRODUCT_PHRASE)})")
    if box not in ("box1", "box2"):
        raise ValueError(f"모르는 적재함: {box!r}")
    return f"Pick up {PRODUCT_PHRASE[product]} and place it in the {box}"

# best.pt 의 클래스 인덱스 중 사용할 것만
# 0 biscuit  1 cake  2 coke(제외)  3 icecream  4 milk  5 roll  6 sandwich  7 yogurt(제외)
KEEP_CLASSES = [0, 1, 3, 4, 5, 6]

TARGET_THICKNESS = 4
OTHER_THICKNESS = 2

# 실제 리그(진열대 6종 × 3개 = 18개, 적재함 비움)에서 conf × iou 격자 탐색으로
# 결정. conf 를 0.30 이하로 내리면 딸기케잌이 milk 로 오검출되고(milk 5개),
# iou 를 0.95 로 올리면 같은 물체가 중복 검출된다(biscuit 6개).
#   conf=0.35 iou=0.90 → 진열대 16/18, 오차합 2
DEFAULT_CONF = 0.35
DEFAULT_IOU = 0.90    # NMS 임계. 인접한 동종 물체가 병합되는 것을 줄인다
DEFAULT_IMGSZ = 640   # best.pt 학습값과 일치시켜야 한다


class Annotator:
    """탑뷰 프레임에 검출 박스와 적재함 영역을 렌더링한다.

    사용 예
        ann = Annotator("/home/newuser/models/omx_goods_yolo11n.pt")
        out = ann(frame_rgb, target="sandwich", dest="box1")
    """

    def __init__(
        self,
        weights: str,
        conf: float = DEFAULT_CONF,
        iou: float = DEFAULT_IOU,
        imgsz: int = DEFAULT_IMGSZ,
        draw_boxes: bool = True,
        track: bool = True,
    ):
        self.model = YOLO(weights)
        self.conf = conf
        self.iou = iou
        self.imgsz = imgsz
        self.draw_boxes = draw_boxes
        self.track = track

    # ────────────────────────────────────────────────────────────────
    def reset(self) -> None:
        """에피소드 경계에서 호출. 트래커 상태가 다음 에피소드로 새는 것을 막는다.

        persist=True 는 상태를 가지므로, 리셋하지 않으면 이전 에피소드의
        트랙이 새 에피소드 첫 프레임에 유령 박스로 나타난다.
        """
        predictor = getattr(self.model, "predictor", None)
        for tracker in getattr(predictor, "trackers", None) or []:
            tracker.reset()

    # ────────────────────────────────────────────────────────────────
    def __call__(
        self,
        frame_rgb: np.ndarray,
        target: str | None = None,
        dest: str | None = None,
    ) -> np.ndarray:
        """주석된 프레임을 반환한다. 입력은 건드리지 않는다.

        target  이번에 집을 상품 클래스명 ("sandwich" 등). 진열대 ROI 안의
                해당 클래스만 굵게 그린다. 적재함에 이미 담긴 동종 상품을
                타겟으로 표시하면 정책이 그걸 다시 집으러 갈 수 있다.
        dest    목적지 적재함 ("box1" / "box2"). 해당 적재함만 굵게 그린다.
        """
        # ultralytics 는 numpy 입력을 BGR 로 간주한다.
        # LeRobot 의 OpenCVCameraConfig.color_mode 기본값은 RGB 이므로 변환 필수.
        bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        kwargs = dict(
            conf=self.conf,
            iou=self.iou,
            imgsz=self.imgsz,
            classes=KEEP_CLASSES,
            verbose=False,
        )
        if self.track:
            result = self.model.track(
                bgr, persist=True, tracker="bytetrack.yaml", **kwargs
            )[0]
        else:
            result = self.model.predict(bgr, **kwargs)[0]

        out = frame_rgb.copy()

        # 적재함 영역을 먼저 그린다 (상품 박스가 위에 오도록)
        if self.draw_boxes:
            for name, roi in (("box1", BOX1_ROI), ("box2", BOX2_ROI)):
                x0, y0, x1, y1 = roi
                cv2.rectangle(
                    out, (x0, y0), (x1, y1), BOX_RGB[name],
                    TARGET_THICKNESS if name == dest else OTHER_THICKNESS,
                    lineType=cv2.LINE_8,
                )

        for box in result.boxes:
            cls = result.names[int(box.cls)]
            color = RGB.get(cls)
            if color is None:
                continue
            x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
            is_target = cls == target and in_roi(SHELF_ROI, cx, cy)
            cv2.rectangle(
                out, (x1, y1), (x2, y2), color,
                TARGET_THICKNESS if is_target else OTHER_THICKNESS,
                lineType=cv2.LINE_8,
            )

        return out


def parse_task(task: str) -> tuple[str | None, str | None]:
    """LeRobot task 문자열에서 (타겟 클래스, 목적지) 를 뽑는다.

    "Pick up sandwich and place it in the box1" -> ("sandwich", "box1")

    데이터셋의 표기가 모델 클래스명과 다른 경우가 있어 별칭을 매핑한다
    ("milk carton" -> "milk", "ice cream" -> "icecream").
    """
    t = task.lower()
    aliases = {
        "milk carton": "milk", "milk": "milk",
        "ice cream": "icecream", "icecream": "icecream",
        "roll cake": "roll", "roll": "roll",
        "sandwich": "sandwich", "cake": "cake", "biscuit": "biscuit",
    }
    target = None
    for phrase in sorted(aliases, key=len, reverse=True):
        if phrase in t:
            target = aliases[phrase]
            break
    dest = "box1" if "box1" in t else "box2" if "box2" in t else None
    return target, dest
