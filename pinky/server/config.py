from __future__ import annotations

import os
from pathlib import Path


def _parse_env_file(path: Path) -> dict[str, str]:
    """python-dotenv 없이도 KEY=VALUE 파일을 읽는다."""
    try:
        from dotenv import dotenv_values

        out: dict[str, str] = {}
        for key, value in dotenv_values(path).items():
            if key and value is not None:
                out[key] = value
        return out
    except ImportError:
        pass

    out = {}
    try:
        text = path.read_text(encoding="utf-8")
    except OSError:
        return out
    for line in text.splitlines():
        trimmed = line.strip()
        if not trimmed or trimmed.startswith("#"):
            continue
        if trimmed.startswith("export "):
            trimmed = trimmed[7:].strip()
        idx = trimmed.index("=") if "=" in trimmed else -1
        if idx < 0:
            continue
        key = trimmed[:idx].strip()
        value = trimmed[idx + 1 :].strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
            value = value[1:-1]
        if key:
            out[key] = value
    return out


def load_env() -> list[str]:
    """
    환경 파일 로드.
    로봇 전송 시 `.env`(숨김)는 업로드가 막히는 경우가 많아
    일반 파일명 `pinky.env`도 지원한다. 먼저 찾은 키는 덮어쓰지 않음.
    로드한 파일 경로 목록을 반환한다.
    """
    root = Path(__file__).resolve().parents[1]
    candidates = [
        # 업로드·배포용 (숨김 파일 아님)
        Path.cwd() / "pinky.env",
        root / "pinky.env",
        # 로컬 개발용
        Path.cwd() / ".env",
        root / ".env",
        Path(__file__).resolve().parents[2] / "server" / ".env",
    ]
    seen: set[Path] = set()
    loaded: list[str] = []
    for path in candidates:
        try:
            resolved = path.resolve()
        except OSError:
            continue
        if resolved in seen or not path.is_file():
            continue
        seen.add(resolved)
        values = _parse_env_file(path)
        if not values:
            continue
        applied = False
        for key, value in values.items():
            if key and key not in os.environ:
                os.environ[key] = value
                applied = True
        if applied or values:
            loaded.append(str(resolved))
    return loaded


def get_host() -> str:
    return os.environ.get("PINKY_HOST", "0.0.0.0")


def get_port() -> int:
    return int(os.environ.get("PINKY_PORT", "4200"))


def get_backend() -> str:
    return os.environ.get("PINKY_BACKEND", "mock")


def get_device_code() -> str:
    return os.environ.get("PINKY_DEVICE_CODE", "cart-1")


def get_controller_url() -> str:
    return os.environ.get("CONTROLLER_URL", "http://127.0.0.1:4100").rstrip("/")


def should_start_sensor_publisher() -> bool:
    """
    ROS2 백엔드일 때 기본으로 센서 publisher 컨트롤러를 기동.
    PINKY_SENSOR_PUBLISHER=0 으로 끌 수 있음.
    """
    flag = os.environ.get("PINKY_SENSOR_PUBLISHER", "auto").lower().strip()
    if flag in ("0", "false", "off", "no"):
        return False
    if flag in ("1", "true", "on", "yes"):
        return True
    return get_backend() == "ros2"
