from __future__ import annotations

import atexit
import logging

from flask import Flask
from flask_cors import CORS

from modules.robot import PinkyRobot

from .config import (
    get_backend,
    get_controller_url,
    get_device_code,
    load_env,
    should_start_sensor_publisher,
)
from .controller_client import ControllerClient
from .routes import bp

logger = logging.getLogger("pinky")


def _apply_boot_lcd(
    robot: PinkyRobot,
    backend: str,
    *,
    background: bool = False,
) -> None:
    """run.py 기동 후 LCD를 pinky_charging으로 전환 (pinky_pro 수정 없이)."""
    import os
    import threading
    import time

    emotion = (os.environ.get("PINKY_EMOTION_BOOT") or "pinky_charging").strip()
    if not emotion:
        return

    attempts = max(1, int(os.environ.get("PINKY_EMOTION_BOOT_TRIES", "12")))
    gap = max(0.2, float(os.environ.get("PINKY_EMOTION_BOOT_GAP_SEC", "0.75")))

    def _try_set() -> None:
        last: dict | None = None
        for attempt in range(1, attempts + 1):
            try:
                last = robot.lcd.set_emotion(emotion)
                if last.get("success"):
                    logger.info(
                        "boot LCD emotion=%s ok (try=%s backend=%s)",
                        emotion,
                        attempt,
                        backend,
                    )
                    return
            except Exception as exc:
                last = {"success": False, "message": str(exc)}
            time.sleep(gap)
        logger.warning("boot LCD emotion=%s failed: %s", emotion, last)

    if background:
        threading.Thread(
            target=_try_set, name="boot-lcd-charging", daemon=True
        ).start()
        return
    _try_set()


def create_app() -> Flask:
    load_env()
    app = Flask(__name__)
    CORS(app, origins="*")

    device_code = get_device_code()
    backend = get_backend()

    # pinky_pro bringup + Nav2 (서브프로세스) — 센서 publisher 보다 먼저
    pro_launcher = None
    if backend == "ros2":
        try:
            from controllers.pro_launch import get_pro_launcher

            pro_launcher = get_pro_launcher()
            pro_launcher.start()
        except Exception as exc:
            logger.exception("pinky_pro auto-launch failed: %s", exc)
            pro_launcher = None

    # LCD emotion_server (/set_emotion) — Flask 전에 기동
    emotion_launcher = None
    if backend == "ros2":
        try:
            from controllers.emotion_launch import get_emotion_launcher

            emotion_launcher = get_emotion_launcher()
            emotion_launcher.start()
        except Exception as exc:
            logger.exception("emotion_server auto-launch failed: %s", exc)
            emotion_launcher = None

    sensor_publisher = None
    if should_start_sensor_publisher():
        try:
            from controllers import SensorPublisherController

            sensor_publisher = SensorPublisherController()
            sensor_publisher.start()
            logger.info(
                "sensor publisher started: %s",
                "; ".join(sensor_publisher.hardware_status),
            )
            import time as _time

            _time.sleep(0.5)
        except Exception as exc:
            logger.exception("sensor publisher failed to start: %s", exc)
            sensor_publisher = None

    robot = PinkyRobot(backend=backend, device_code=device_code)
    robot.start()

    # sllidar 가 /scan 만 만들고 데이터를 안 줄 때 LidarReader 로 복구
    if backend == "ros2":
        try:
            from controllers.lidar_recovery import ensure_lidar

            ensure_lidar(robot)
        except Exception as exc:
            logger.exception("lidar recovery error: %s", exc)

    # 부팅 직후 LCD → pinky_charging (emotion_server 기동 직후 재시도 포함)
    _apply_boot_lcd(robot, backend, background=True)

    app.extensions["robot"] = robot
    app.extensions["device_code"] = device_code
    app.extensions["controller"] = ControllerClient(get_controller_url())
    app.extensions["sensor_publisher"] = sensor_publisher
    app.extensions["pro_launcher"] = pro_launcher
    app.extensions["emotion_launcher"] = emotion_launcher

    def _shutdown() -> None:
        try:
            if sensor_publisher is not None:
                sensor_publisher.stop()
        except Exception:
            pass
        try:
            robot.stop()
        except Exception:
            pass
        try:
            if emotion_launcher is not None:
                emotion_launcher.stop()
        except Exception:
            pass
        try:
            if pro_launcher is not None:
                pro_launcher.stop()
        except Exception:
            pass
        if backend == "ros2":
            try:
                from modules.backends import ros2_runtime

                ros2_runtime.shutdown()
            except Exception:
                pass

    atexit.register(_shutdown)

    app.register_blueprint(bp)
    return app
