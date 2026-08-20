"""탑뷰 카메라 기준 고정 영역 좌표.

전부 640x480 탑뷰 프레임의 픽셀 좌표 (x0, y0, x1, y1) 이다.

────────────────────────────────────────────────────────────────────────
출처 — 2026-08-17 실측, 전 상수 검증됨
────────────────────────────────────────────────────────────────────────
측정 프레임
    /dev/omx_cam_top (Jieli 4c4a:4a55) 에서 워밍업 후 직접 캡처.
    두 적재함이 모두 비어 있고 진열대는 6구역 × 3개로 완전히 채워진 상태.
    팔은 프레임 우측 상단에 위치.

방법
    HSV 갈색 분할로 카드보드 합집합을 구하고, 그 안에서 어두운 픽셀이 가로로
    가장 많은 행을 두 적재함의 경계로 잡았다 (omx_yolo.measure).
    미리보기 이미지로 세 영역을 육안 확인함.

box1 / box2 배정
    사용자 확인: "로봇 팔과 가까운 쪽이 box1, 먼 쪽이 box2".
    팔이 우측 상단에 있으므로 상단 박스가 box1, 하단이 box2.

    독립 교차 검증: prototype_4 의 ep09 (task = box1) 종료 프레임에서 샌드위치가
    상단 박스 (139, 223) 에서 검출되었다. 상단 = box1 과 일치한다.
    (같은 검증에서 ep10 은 task 가 box2 인데 샌드위치가 상단(141, 211) 에
     놓였다 — 이 에피소드가 오염되었다는 별개 증거와 부합한다. README 참조.)

주의
    카메라를 움직이거나 적재함·진열대를 옮기면 이 값 전부 무효다.
    재측정:  python -m omx_yolo.measure --camera /dev/omx_cam_top --box1 upper
    현재값 확인:  python -m omx_yolo.measure --camera /dev/omx_cam_top --verify

    워밍업: 이 카메라는 첫 프레임의 화이트밸런스가 크게 녹색으로 치우친다.
    약 30프레임(1초) 후 안정되고 그 뒤로는 완전히 일정하다. lerobot 설정에서
    warmup_s 를 1 이상 유지할 것.
────────────────────────────────────────────────────────────────────────
"""

from __future__ import annotations

# 프레임 크기 — 이 값이 바뀌면 아래 좌표 전부 무효
FRAME_W, FRAME_H = 640, 480

# 2026-08-18 카메라 화각 조정 후 재측정.
# 적재함 2개가 모두 프레임에 들어오도록 카메라를 뒤로 뺀 상태의 값이다.
# 이전 값들은 전부 무효 — 화각이 바뀌면 좌표도 전부 바뀐다.
#
# 측정 방법: 진열대 흰 칸 6개(x 352~592)를 먼저 찾고, 그 좌측 경계를 기준으로
# 적재함 영역을 갈색 분할했다. 책상도 갈색이라 전역 분할은 실패한다.
CARDBOARD_UNION = (0, 56, 279, 480)

# 진열대 = 카드보드 오른쪽 전체. 6칸이 모두 이 안에 들어온다.
SHELF_ROI = (279, 0, 640, 480)

# 적재함 2개. 경계 y = 268 (두 상자 사이 어두운 가로선).
BOX1_ROI = (0, 56, 279, 268)    # 상단 — 로봇 팔과 가까운 쪽
BOX2_ROI = (0, 268, 279, 480)   # 하단 — 로봇 팔과 먼 쪽

VERIFIED = {
    "CARDBOARD_UNION": True,
    "SHELF_ROI": True,
    "BOX1_ROI": True,
    "BOX2_ROI": True,
}


def in_roi(roi: tuple[int, int, int, int], cx: float, cy: float) -> bool:
    """중심점이 ROI 안에 있는가."""
    x0, y0, x1, y1 = roi
    return x0 <= cx <= x1 and y0 <= cy <= y1


def box_roi(name: str) -> tuple[int, int, int, int] | None:
    return {"box1": BOX1_ROI, "box2": BOX2_ROI}.get(name)


def warn_unverified() -> list[str]:
    """미검증 상수 목록. 실행 시 경고를 띄우는 데 쓴다."""
    return [k for k, v in VERIFIED.items() if not v]
