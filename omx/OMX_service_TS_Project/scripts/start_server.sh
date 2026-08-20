#!/usr/bin/env bash
# OMX 픽업 서버 기동 (로봇팔 PC에서 실행)
#
# 사용:
#   export POLICY=/path/to/checkpoint
#   ./scripts/start_server.sh
#
# 또는:
#   POLICY=/path/to/checkpoint OMX_PORT=8080 ./scripts/start_server.sh

set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

if [ -f .env ]; then
  set -a
  # shellcheck disable=SC1091
  source .env
  set +a
fi

POLICY="${POLICY:-${1:-}}"
if [ -z "$POLICY" ]; then
  echo "POLICY 체크포인트 경로 필요" >&2
  echo "  POLICY=/path/to/checkpoint $0" >&2
  exit 1
fi

HOST="${OMX_HOST:-0.0.0.0}"
PORT="${OMX_PORT:-8080}"

echo "OMX server bind ${HOST}:${PORT}"
echo "관제 PC server/.env 예: OMX_URL=http://<이 PC LAN IP>:${PORT}"

exec python -m omx_yolo.server \
  --policy "$POLICY" \
  --host "$HOST" \
  --port "$PORT" \
  "${@:2}"
