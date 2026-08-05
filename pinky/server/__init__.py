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

    app.extensions["robot"] = robot
    app.extensions["device_code"] = device_code
    app.extensions["controller"] = ControllerClient(get_controller_url())
    app.extensions["sensor_publisher"] = sensor_publisher
    app.extensions["pro_launcher"] = pro_launcher

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
