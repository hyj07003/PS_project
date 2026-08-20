"""LeRobot 에 'yolo_opencv' 카메라 타입을 추가한다.

LeRobot 소스는 한 줄도 수정하지 않는다. CameraConfig 가 draccus.ChoiceRegistry
기반이므로 서브클래스를 등록하면 CLI 에서 바로 쓸 수 있다.

    --robot.cameras="{ front: {type: yolo_opencv, index_or_path: /dev/omx_cam_top,
                               width: 640, height: 480, fps: 30, fourcc: MJPG,
                               weights: /home/newuser/il_ws/models/omx_goods_yolo11n.pt} }"

주석을 정책 전처리가 아니라 카메라 계층에 넣는 이유: 수집(lerobot-record),
재생, 추론이 전부 Camera.read() 를 거치므로 한 곳만 감싸면 모든 경로가
동일한 주석을 받는다. 학습/추론 불일치가 구조적으로 불가능해진다.

── 주의: 패키지 레이아웃 제약 ──────────────────────────────────────────
make_cameras_from_configs (lerobot/cameras/utils.py) 는 레지스트리 조회가
아니라 하드코딩된 if/elif 체인이다. opencv / intelrealsense / reachy2_camera
/ zmq 만 분기가 있고, 그 밖의 타입은 else 의 make_device_from_device_class
폴백으로 간다. 그 폴백은 이름 규칙으로 클래스를 찾는다:

    YoloOpenCVCameraConfig  →  "Config" 제거  →  YoloOpenCVCamera
                            →  설정 모듈의 부모 패키지(omx_yolo)에서 검색

따라서 __init__.py 에서 YoloOpenCVCamera 를 노출해야 한다. 노출하지 않으면
"Error creating camera ..." 로 실패한다.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass

from lerobot.cameras.configs import CameraConfig
from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig

from .annotate import DEFAULT_CONF, DEFAULT_IOU, DEFAULT_IMGSZ, Annotator
from .geometry import warn_unverified

logger = logging.getLogger(__name__)


@CameraConfig.register_subclass("yolo_opencv")
@dataclass
class YoloOpenCVCameraConfig(OpenCVCameraConfig):
    """OpenCVCameraConfig + YOLO 주석 설정."""

    weights: str = "/home/newuser/il_ws/models/omx_goods_yolo11n.pt"
    conf: float = DEFAULT_CONF
    iou: float = DEFAULT_IOU
    imgsz: int = DEFAULT_IMGSZ
    draw_boxes: bool = True
    track: bool = True

    # 이번 롤아웃의 지시문. 여기서 타겟 상품과 목적지 적재함을 뽑는다.
    #
    # lerobot-record 는 카메라에 set_task() 를 호출해 주지 않는다. 그런데
    # 학습 데이터(convert.py)에는 타겟이 굵게, 목적지 적재함이 굵게 그려져
    # 있다. 추론에서 그 강조가 빠지면 학습/추론 불일치가 된다.
    # 그래서 CLI 에서 지시문을 직접 받는다:
    #
    #   camera1: {type: yolo_opencv, ...,
    #             task: "Pick up sandwich and place it in the box1"}
    #
    # --dataset.single_task 와 같은 문장을 주면 된다.
    task: str = ""


class YoloOpenCVCamera(OpenCVCamera):
    """read() / async_read() 결과를 Annotator 로 통과시키는 얇은 래퍼."""

    def __init__(self, config: YoloOpenCVCameraConfig):
        super().__init__(config)
        self._ann = Annotator(
            config.weights,
            conf=config.conf,
            iou=config.iou,
            imgsz=config.imgsz,
            draw_boxes=config.draw_boxes,
            track=config.track,
        )
        self.target: str | None = None
        self.dest: str | None = None
        if config.task:
            from .annotate import parse_task

            self.target, self.dest = parse_task(config.task)
            if self.target is None or self.dest is None:
                logger.warning("task 문자열에서 타겟/목적지를 못 뽑았습니다: %r "
                               "→ 타겟 강조 없이 그립니다", config.task)
            else:
                logger.info("주석 타겟=%s 목적지=%s (task=%r)",
                            self.target, self.dest, config.task)
        else:
            logger.warning(
                "task 가 비어 있어 타겟 강조 없이 그립니다. 학습 데이터에는 "
                "타겟이 굵게 그려져 있으므로 불일치가 생깁니다. "
                "camera 설정에 task: \"Pick up ...\" 를 넣으십시오.")

        unverified = warn_unverified()
        if config.draw_boxes and unverified:
            logger.warning(
                "omx_yolo.geometry 의 미검증 상수를 사용합니다: %s. "
                "python -m omx_yolo.measure 로 실측하십시오.",
                ", ".join(unverified),
            )

    # ────────────────────────────────────────────────────────────────
    def set_task(self, target: str | None, dest: str | None = None) -> None:
        """다음 프레임부터 적용할 타겟/목적지를 지정한다.

        중앙 관제 서버가 픽업 명령을 내릴 때마다 호출하고,
        에피소드 경계에서는 reset_tracker() 도 함께 호출한다.
        """
        self.target = target
        self.dest = dest

    def reset_tracker(self) -> None:
        self._ann.reset()

    # ────────────────────────────────────────────────────────────────
    def read(self, *args, **kwargs):
        return self._ann(super().read(*args, **kwargs), self.target, self.dest)

    def async_read(self, *args, **kwargs):
        return self._ann(super().async_read(*args, **kwargs), self.target, self.dest)

    def read_latest(self, *args, **kwargs):
        """읽기 경로 세 번째. 이것을 빠뜨리면 주석이 통째로 사라진다.

        omx_follower.get_observation() 은 read() 도 async_read() 도 아닌
        read_latest() 를 부른다 (robots/omx_follower/omx_follower.py:179).
        2026-08-19 첫 롤아웃에서 확인: read()/async_read() 만 감싸 두었더니
        정책이 주석 없는 원본 프레임을 받았다. 학습 데이터에는 박스가 그려져
        있으므로 입력 분포가 어긋나 파지가 계속 빗나갔다.

        Camera 기반 클래스가 읽기 메서드를 늘리면 여기도 함께 늘려야 한다.
        현재 기반 클래스의 공개 읽기 메서드는 read / async_read / read_latest
        세 개다 (lerobot/cameras/camera.py).
        """
        return self._ann(super().read_latest(*args, **kwargs), self.target, self.dest)
