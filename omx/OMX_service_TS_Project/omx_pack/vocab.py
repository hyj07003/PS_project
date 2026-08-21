"""관제 어휘 ↔ 포장 리그 매핑.

픽업(omx_yolo.annotate)과 같은 역할이지만 축이 다르다. 픽업은 **상품**이
축이었다 — 무엇을 집을지가 지시문으로 정책에 전달됐다. 포장 정책(ACT)은
언어 조건이 아예 없다. `input_features` 에 지시문 자리가 없어서, 무엇을
어디에 담을지는 **어느 체크포인트를 올리느냐로만** 결정된다.

그래서 이 파일의 유일한 축은 바구니다:

    관제 deviceCode → 바구니 색 → 체크포인트

체크포인트를 잘못 고르면 팔이 엉뚱한 바구니로 간다. 픽업에서 지시문 표기가
어긋나 샌드위치를 집었던 것과 같은 종류의 사고이고, 마찬가지로 예외도 로그도
없이 조용히 잘못 움직인다. 그래서 매핑을 여기 한 곳에만 둔다.
"""

from __future__ import annotations

# 관제 deviceCode → 포장 바구니 색.
#
# 픽업 쪽 매핑(cart-1 → box1, cart-2 → box2)과 짝을 이룬다. 같은 카트에서
# 온 물건이 같은 바구니로 들어가야 하므로 이 둘은 항상 함께 바뀌어야 한다.
#
#   cart-1 → box1(팔에 가까운 적재함) → YELLOW 바구니
#   cart-2 → box2(먼 적재함)          → MINT 바구니
CONTROLLER_DEVICE_BASKET: dict[str, str] = {
    "cart-1": "yellow",
    "cart-2": "mint",
}

# 바구니 색 → 체크포인트 경로.
#
# 두 모델 다 ACT 50,000 스텝이고 구조가 같다. 다른 것은 가중치와 정규화
# 통계뿐이다(2026-08-21 확인). 학습 통계상 MINT 쪽이 41 에피소드로 YELLOW
# 21 에피소드를 포함하는 것처럼 보이는데, 그렇다면 MINT 모델이 양쪽을 다
# 학습했다는 뜻이다. 팀원 확인 전까지는 바구니별 전용으로 취급한다 —
# 겸용이라고 가정했다가 틀리면 엉뚱한 바구니로 간다.
_OUT = "/home/newuser/il_ws/src/lerobot/outputs/train"
DEFAULT_CHECKPOINTS: dict[str, str] = {
    "yellow": f"{_OUT}/my_act_20260819_125841-CART_YELLOW_MODEL"
              f"/my_act_20260819_125841/checkpoints/last/pretrained_model",
    "mint": f"{_OUT}/my_act_20260820_123027_CART_MINT_MODEL"
            f"/my_act_20260820_123027/checkpoints/last/pretrained_model",
}

# 적재함 한 칸에 들어가는 최대 개수. 픽업 서버의 SHELF_CAPACITY 와 같은
# 이유로 둔다 — 관제가 그보다 많이 요청하면 받아 봐야 실패한다.
BOX_CAPACITY = 3


def resolve_basket(device_code: str) -> str:
    """관제 deviceCode 를 바구니 색으로 바꾼다. 모르면 ValueError."""
    key = (device_code or "").strip().lower()
    if key not in CONTROLLER_DEVICE_BASKET:
        known = ", ".join(sorted(CONTROLLER_DEVICE_BASKET))
        raise ValueError(
            f"모르는 deviceCode 입니다: {device_code!r} (아는 것: {known})")
    return CONTROLLER_DEVICE_BASKET[key]


def resolve_checkpoint(basket: str, overrides: dict[str, str] | None = None) -> str:
    """바구니 색에 해당하는 체크포인트 경로."""
    table = dict(DEFAULT_CHECKPOINTS)
    if overrides:
        table.update({k.lower(): v for k, v in overrides.items()})
    key = (basket or "").strip().lower()
    if key not in table:
        raise ValueError(
            f"모르는 바구니입니다: {basket!r} (아는 것: {', '.join(sorted(table))})")
    return table[key]
