#!/usr/bin/env python3
from __future__ import annotations

from app import create_app
from app.config import get_host, get_port, load_env


def main() -> None:
    load_env()
    app = create_app()
    app.run(host=get_host(), port=get_port(), debug=False, threaded=True)


if __name__ == "__main__":
    main()
