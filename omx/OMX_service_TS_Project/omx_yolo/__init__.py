"""OMX-AI 픽업 정책용 YOLO 주석 계층.

LeRobot 을 수정하지 않고, 탑뷰 프레임에 검출 박스를 렌더링해 SmolVLA 의
시각 입력을 보강한다. 학습 데이터 변환과 실시간 추론이 annotate.Annotator
하나를 공유하는 것이 설계의 핵심이다.

    import omx_yolo   # 이 import 가 'yolo_opencv' 카메라 타입을 등록한다

주의: camera 모듈에서 노출하는 이름을 여기서 그대로 내보내야 한다.
LeRobot 의 make_device_from_device_class 폴백이 부모 패키지에서
YoloOpenCVCamera 를 찾기 때문이다. 자세한 이유는 camera.py 주석 참조.
"""

from .annotate import RGB, BOX_RGB, KEEP_CLASSES, Annotator, parse_task
from .camera import YoloOpenCVCamera, YoloOpenCVCameraConfig
from .geometry import BOX1_ROI, BOX2_ROI, CARDBOARD_UNION, SHELF_ROI, warn_unverified

__all__ = [
    "Annotator",
    "parse_task",
    "RGB",
    "BOX_RGB",
    "KEEP_CLASSES",
    "YoloOpenCVCamera",
    "YoloOpenCVCameraConfig",
    "SHELF_ROI",
    "BOX1_ROI",
    "BOX2_ROI",
    "CARDBOARD_UNION",
    "warn_unverified",
]
