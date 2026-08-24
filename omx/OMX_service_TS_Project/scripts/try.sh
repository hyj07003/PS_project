#!/usr/bin/env bash
# 픽업·포장 서버를 관제 없이 단위 시험한다.
#
#   ./try.sh health                              두 서버 상태
#   ./try.sh pick sandwich cart-1 [개수] [재시도]  픽업 (끝날 때까지 대기)
#   ./try.sh pack cart-1 [재시도]                 포장 (적재함 비우기)
#   ./try.sh state pick|pack                     지금 상태만
#   ./try.sh stop  pick|pack [mode]              정지 (기본 immediate)
#
# 요청을 보낸 뒤 끝날 때까지 폴링하면서 진행을 한 줄씩 찍는다. 손으로
# curl 을 반복하는 것보다 빠르고, 무엇이 언제 바뀌었는지 남는다.
set -uo pipefail

PICK=${PICK_URL:-http://localhost:8080}
PACK=${PACK_URL:-http://localhost:8081}

j() { python3 -c "import sys,json;print(json.dumps(json.load(sys.stdin),ensure_ascii=False,indent=1))"; }

post() {  # post <url> <json>
  curl -s --max-time 20 -X POST "$1" -H 'Content-Type: application/json' -d "$2"
}

# 진행 상태가 바뀔 때만 한 줄 찍고, 끝나면 결과를 요약한다.
watch_job() {  # watch_job <state_url> <pick|pack>
  local url=$1 kind=$2 prev="" line
  while true; do
    line=$(curl -s --max-time 5 "$url" | python3 -c "
import sys,json
s=json.load(sys.stdin)
st=s.get('status','?')
if '$kind'=='pick':
    prog=f\"{s.get('done','?')}/{s.get('total','?')}\"
    extra=f\" · 재시도 {s.get('retries',0)}/{s.get('maxRetries',0)}\" if s.get('maxRetries') else ''
else:
    be=s.get('boxEmpty')
    prog=f\"시도 {s.get('attempt','?')}/{s.get('maxAttempts','?')}\"
    extra=' · 적재함 ' + ('비움' if be is True else '물건남음' if be is False else '확인전')
print(f\"{st} · {prog}{extra} · {s.get('elapsedSec',0)}s|{st}|{s.get('message','')}\")
" 2>/dev/null) || { echo "  서버 응답 없음"; return 1; }
    local body=${line%%|*} rest=${line#*|} st=${rest%%|*} msg=${rest#*|}
    [ "$body" != "$prev" ] && { echo "  $body"; prev=$body; }
    case "$st" in
      DONE|FAILED|ABORTED|IDLE)
        echo
        [ -n "$msg" ] && echo "  메시지: $msg"
        curl -s "$url" | python3 -c "
import sys,json
s=json.load(sys.stdin)
for r in s.get('results',[]):
    bits=[f\"ep{r.get('index')}\", f\"{r.get('seconds')}초\", r.get('reason','')]
    if 'grasped' in r: bits.insert(2, '파지 O' if r['grasped'] else '파지 X')
    if r.get('retried'): bits.append(f\"(재시도 {r['retried']})\")
    if r.get('boxView'): bits.append(r['boxView'])
    print('   ', ' · '.join(str(b) for b in bits))
"
        return 0 ;;
    esac
    sleep 0.5
  done
}

cmd=${1:-help}
case "$cmd" in
  health)
    for u in "$PICK" "$PACK"; do
      echo "── $u"
      curl -s --max-time 5 "$u/health" | python3 -c "
import sys,json
try: h=json.load(sys.stdin)
except Exception: print('   응답 없음'); raise SystemExit
print(f\"   robotConnected={h.get('robotConnected')} busy={h.get('busy')} status={h.get('status')}\")
if h.get('baskets'): print('   바구니', ', '.join(h['baskets']))
if h.get('message'): print('   ⚠', h['message'])
" 2>/dev/null || echo "   응답 없음"
    done ;;

  pick)
    slug=${2:?상품 slug 이 필요합니다 (예: sandwich)}
    dev=${3:-cart-1}; qty=${4:-1}; ret=${5:-}
    body="{\"orderId\":1,\"deviceCode\":\"$dev\",\"slug\":\"$slug\",\"quantity\":$qty"
    [ -n "$ret" ] && body="$body,\"retries\":$ret"
    body="$body}"
    echo "▸ POST $PICK/pick  $body"
    r=$(post "$PICK/pick" "$body")
    echo "$r" | j | head -6
    echo "$r" | grep -q '"success": true' || exit 1
    echo; watch_job "$PICK/pick/state" pick ;;

  pack)
    dev=${2:-cart-1}; att=${3:-3}
    body="{\"orderId\":1,\"deviceCode\":\"$dev\",\"maxAttempts\":$att}"
    echo "▸ POST $PACK/pack  $body"
    r=$(post "$PACK/pack" "$body")
    echo "$r" | j | head -6
    echo "$r" | grep -q '"success": true' || exit 1
    echo; watch_job "$PACK/pack/state" pack ;;

  state)
    case ${2:-pick} in
      pick) curl -s "$PICK/pick/state" | j ;;
      pack) curl -s "$PACK/pack/state" | j ;;
    esac ;;

  stop)
    mode=${3:-immediate}
    case ${2:-pick} in
      pick) post "$PICK/pick/stop" "{\"mode\":\"$mode\"}" | j ;;
      pack) post "$PACK/pack/stop" "{\"mode\":\"$mode\"}" | j ;;
    esac ;;

  *) sed -n '2,12p' "$0" | sed 's/^# \?//' ;;
esac
