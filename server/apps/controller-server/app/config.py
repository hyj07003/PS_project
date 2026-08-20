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
    # 같은 공유기(Wi‑Fi)의 로봇에서 접속하려면 0.0.0.0
    return os.environ.get("CONTROLLER_HOST", "0.0.0.0")


def get_port() -> int:
    return int(os.environ.get("CONTROLLER_PORT", "4100"))


def get_omx_url() -> str | None:
    raw = (os.environ.get("OMX_URL") or "").strip().rstrip("/")
    if not raw:
        return None
    if not raw.startswith(("http://", "https://")):
        raw = f"http://{raw}"
    return raw


def get_omx_connect_timeout() -> float:
    return float(os.environ.get("OMX_CONNECT_TIMEOUT_SEC", "5"))
