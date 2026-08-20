#!/usr/bin/env bash
# 12개 조건(상품 6 × 적재함 2)을 순서대로 돌린다.
#
#   ./rollout_all.sh [조건당횟수]      기본 5
#
# 조건당 5회 = 60 에피소드. 에피소드 90초 + 리셋 20초 기준 약 110분.
# 조건당 10회로 올리면 120 에피소드 / 약 3.7시간이다.
#
# 한 번에 다 돌릴 필요는 없다. 조건 사이에서 멈추므로 언제든 Ctrl+C 로
# 나갔다가 나중에 rollout.sh 로 남은 조건만 이어서 하면 된다.
#
# 순서를 상품별이 아니라 적재함별로 묶은 이유: 적재함을 바꾸는 것보다
# 진열대 리필이 잦으므로, 같은 상품을 연속으로 두면 리필 동선이 짧다.
set -euo pipefail
cd "$(dirname "$0")"
source ./rollout_env.sh

N=${1:-5}

echo "조건 12개 × ${N}회 = $((12 * N)) 에피소드"
echo "예상 소요 약 $(( 12 * N * (EPISODE_S + RESET_S) / 60 ))분 (대기 시간 제외)"
echo

preflight || { echo "사전 점검 실패 — 중단합니다."; exit 1; }

i=0
for product in "${PRODUCTS[@]}"; do
  for box in box1 box2; do
    i=$((i + 1))
    echo
    echo "══════════════════════════════════════════════════"
    echo " 조건 $i/12   $product → $box"
    echo "══════════════════════════════════════════════════"
    ./rollout.sh "$product" "$box" "$N"
    echo
    echo "조건 $i/12 완료. 다음 조건 전에 진열대를 채우십시오."
    if [ $i -lt 12 ]; then
      read -r -p "다음 조건으로 넘어가려면 Enter (중단은 Ctrl+C) " _
    fi
  done
done

echo
echo "전체 완료 → $EVAL_REPO"
echo
$PY -m omx_yolo.evaluate --repo-id "$EVAL_REPO"
