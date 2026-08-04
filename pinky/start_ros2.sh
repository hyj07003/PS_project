#!/usr/bin/env bash
# 라즈베리(실기)에서 Pinky를 ROS2 백엔드로 기동
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"

ENV_FILE=""
if [[ -f "$ROOT/pinky.env" ]]; then
  ENV_FILE="$ROOT/pinky.env"
elif [[ -f "$ROOT/.env" ]]; then
  ENV_FILE="$ROOT/.env"
fi

if [[ -n "$ENV_FILE" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "$ENV_FILE"
  set +a
  echo "[start] loaded $ENV_FILE"
else
  echo "[start] WARNING: pinky.env / .env 없음 — 기본값 사용"
fi

# 실기 기본: ros2 (파일이 없거나 mock으로 남아 있어도 강제)
export PINKY_BACKEND="${PINKY_BACKEND:-ros2}"
if [[ "$PINKY_BACKEND" == "mock" ]]; then
  echo "[start] WARNING: PINKY_BACKEND=mock → ros2 로 강제 변경"
  export PINKY_BACKEND=ros2
fi

export PINKY_SENSOR_PUBLISHER="${PINKY_SENSOR_PUBLISHER:-auto}"
export PINKY_HOST="${PINKY_HOST:-0.0.0.0}"
export PINKY_PORT="${PINKY_PORT:-4200}"

# ROS2 (환경에 맞게 수정)
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "[start] WARNING: /opt/ros/*/setup.bash 없음 — rclpy import 실패 시 mock/에러"
fi

if [[ -x "$ROOT/.venv/bin/python" ]]; then
  PY="$ROOT/.venv/bin/python"
else
  PY="python3"
fi

echo "[start] stopping old run.py (if any)"
pkill -f "$ROOT/.venv/bin/python run.py" 2>/dev/null || true
pkill -f "python3 run.py" 2>/dev/null || true
sleep 0.5

echo "[start] backend=$PINKY_BACKEND publisher=$PINKY_SENSOR_PUBLISHER port=$PINKY_PORT"
exec "$PY" "$ROOT/run.py"
