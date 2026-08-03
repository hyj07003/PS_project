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
    load_env()
    backend = get_backend()
    publish = should_start_sensor_publisher()
    print(
        f"[pinky] backend={backend} sensor_publisher={'on' if publish else 'off'} "
        f"host={get_host()} port={get_port()}"
    )
    app = create_app()
    # Flask 개발 서버 reloader는 ROS 노드를 이중 기동하므로 사용하지 않음
    app.run(host=get_host(), port=get_port(), debug=False, threaded=True, use_reloader=False)


if __name__ == "__main__":
    main()
