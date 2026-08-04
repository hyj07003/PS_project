#!/usr/bin/env python3
from __future__ import annotations

import logging
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from server import create_app
from server.config import (
    get_backend,
    get_controller_url,
    get_device_code,
    get_host,
    get_port,
    load_env,
    should_start_sensor_publisher,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)


def main() -> None:
    loaded = load_env()
    backend = get_backend()
    publish = should_start_sensor_publisher()
    env_note = ", ".join(loaded) if loaded else "(none — defaults / process env only)"
    print(
        f"[pinky] env_files={env_note}\n"
        f"[pinky] backend={backend} sensor_publisher={'on' if publish else 'off'} "
        f"device={get_device_code()} controller={get_controller_url()}\n"
        f"[pinky] host={get_host()} port={get_port()}"
    )
    if backend == "mock":
        print(
            "[pinky] WARNING: PINKY_BACKEND=mock — 모니터링에 더미 센서가 표시됩니다.\n"
            "         실기에서는 pinky.env 에 PINKY_BACKEND=ros2 를 넣고 "
            "~/pinky 에서 재시작하세요."
        )
    app = create_app()
    # Flask 개발 서버 reloader는 ROS 노드를 이중 기동하므로 사용하지 않음
    app.run(host=get_host(), port=get_port(), debug=False, threaded=True, use_reloader=False)


if __name__ == "__main__":
    main()
