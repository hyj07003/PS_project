from __future__ import annotations

import os
from pathlib import Path


def load_env() -> None:
    """Load .env from cwd or server root without overwriting existing env."""
    try:
        from dotenv import dotenv_values
    except ImportError:
        return

    candidates = [
        Path.cwd() / ".env",
        Path.cwd().parent.parent / ".env",  # apps/controller-server → server/
        Path(__file__).resolve().parents[2] / ".env",
    ]
    for path in candidates:
        if not path.is_file():
            continue
        for key, value in dotenv_values(path).items():
            if key and value is not None and key not in os.environ:
                os.environ[key] = value


def get_database_path() -> Path:
    configured = os.environ.get("DATABASE_PATH", "./data/smartshop.db")
    path = Path(configured)
    if not path.is_absolute():
        path = Path.cwd() / path
    return path.resolve()


def get_host() -> str:
    return "127.0.0.1"


def get_port() -> int:
    return int(os.environ.get("CONTROLLER_PORT", "4100"))
