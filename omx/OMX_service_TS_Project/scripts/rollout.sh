#!/usr/bin/env bash
# 조건 하나를 롤아웃해 기록한다.
#
#   ./rollout.sh <상품> <box1|box2> [횟수]
#
#   ./rollout.sh sandwich box1 10
#   ./rollout.sh "milk carton" box2 5
#
# 상품 표기는 annotate.PRODUCT_PHRASE 와 같아야 한다:
#   sandwich / milk carton / icecream / cake / biscuit / roll
#
# 환경변수로 덮어쓸 수 있는 값: CKPT EVAL_REPO EPISODE_S RESET_S DISPLAY_DATA
set -euo pipefail
cd "$(dirname "$0")"
source ./rollout_env.sh

PRODUCT=${1:-}
BOX=${2:-}
N=${3:-10}

if [ -z "$PRODUCT" ] || [ -z "$BOX" ]; then
  echo "사용법: $0 <상품> <box1|box2> [횟수]"
  echo "  상품: ${PRODUCTS[*]}"
  exit 2
fi
case "$BOX" in box1|box2) ;; *) echo "적재함은 box1 또는 box2"; exit 2;; esac

# 상품 표기를 annotate.PRODUCT_PHRASE 기준으로 정규화한다.
# 여기서 막지 않으면 정책이 학습한 적 없는 지시문을 받는다.
PRODUCT=$(normalize_product "$PRODUCT") || {
  echo "상품명을 확인하십시오. 롤아웃을 중단합니다."
  exit 2
}

TASK=$(build_task "$PRODUCT" "$BOX")

preflight || { echo "사전 점검 실패 — 중단합니다."; exit 1; }

# 저장소가 이미 있으면 이어 붙인다. 조건마다 새 저장소를 만들지 않는다.
#
# 디렉터리 존재만으로 판단하면 안 된다. 기록이 시작되자마자 죽으면
# meta/info.json 하나만 남은 껍데기가 생기는데, 그 상태로 --resume=true 를
# 주면 LeRobot 이 meta/tasks.parquet 을 못 찾고 허브로 폴백해 404 로 죽는다:
#
#     FileNotFoundError: .../meta/tasks.parquet
#     RepositoryNotFoundError: 404 ... /api/datasets/<repo>/refs
#
# 실제로 에피소드가 하나라도 저장되었는지를 tasks.parquet 존재로 판단한다.
DS_DIR="$HOME/.cache/huggingface/lerobot/$EVAL_REPO"
RESUME=false
if [ -f "$DS_DIR/meta/tasks.parquet" ]; then
  RESUME=true
elif [ -d "$DS_DIR" ]; then
  # 껍데기 — 지우지 않고 옆으로 치운다. 되돌릴 수 있게.
  STASH="$DS_DIR.aborted.$(date +%H%M%S)"
  mv "$DS_DIR" "$STASH"
  echo "⚠ 중단된 빈 저장소를 발견해 옆으로 옮겼습니다 (에피소드 0개):"
  echo "    $STASH"
  echo "  필요 없으면 지우십시오:  rm -rf \"$STASH\""
  echo
fi

cat <<EOF

┌────────────────────────────────────────────────────────
│ 지시문   $TASK
│ 횟수     $N 회   (에피소드 ${EPISODE_S}s / 리셋 ${RESET_S}s)
│ 저장소   $EVAL_REPO   (이어쓰기: $RESUME)
│ 정책     $CKPT
└────────────────────────────────────────────────────────

▶ 키 조작 — 이것이 핵심이다
  →  (오른쪽)  지금 에피소드를 끝내고 저장, 다음으로 넘어간다
  ←  (왼쪽)    지금 에피소드를 버리고 다시 찍는다
  ESC          전체 기록을 정상 종료한다
  Ctrl+C 는 쓰지 말 것 — 아무것도 저장되지 않는다

▶ 언제 → 를 누르는가
  정책은 종료 신호를 배우지 않았다. 한 번 집어 넣고 홈으로 돌아온 뒤에도
  빈 공간을 계속 집으려 든다. 그러니 사람이 끊어 줘야 한다.

      팔이 물건을 넣고 홈 자세로 돌아온 그 순간  →  를 누른다

  이렇게 해야 "에피소드 1개 = 픽업 시도 1회" 가 되어 채점이 성립한다.
  90초는 안전 상한일 뿐이고, 그때까지 두면 한 에피소드에 여러 번 집게 된다.

  파지에 실패했거나 팔이 이상하게 움직이면 ← 로 다시 찍는다.

▶ 리셋 20초 동안 할 일
  1. 적재함에 들어간 물건을 빼낸다
  2. 진열대는 3회마다 채운다 — 잔여 3/2/1 을 고루 겪게 하려는 것이다
     (1회차 3개 → 2회차 2개 → 3회차 1개 → 리필)

  ★ 남은 물체는 반드시 "뒤쪽부터" 채워진 상태로 둘 것 ★
     수집 때 사람은 항상 가까운 것부터 집었다. 그래서 학습 데이터에
     존재하는 진열 상태는 이 셋뿐이다:

         [1][2][3]      [ ][2][3]      [ ][ ][3]
         (3개)          (2개)          (1개)

     정책이 순서를 어겨 가운데 것을 집으면 [1][ ][3] 같은 상태가 되는데,
     이건 학습에 없는 상황이다. 그대로 다음 회차를 찍으면 정책이 늘 그랬듯
     "뒤쪽"으로 손을 뻗어 빈 칸을 헛집는다 (2026-08-19 milk box2 ep45·46).

     정책이 순서를 어겼으면 다음 회차 전에 남은 물체를 뒤쪽으로 밀어
     위 세 형태 중 하나로 맞춰 놓을 것.

EOF
read -r -p "준비되면 Enter (중단은 Ctrl+C) " _

$PY -m omx_yolo.record \
  "${ROBOT_ARGS[@]}" \
  --robot.cameras="$(cameras_arg "$TASK")" \
  --policy.path="$CKPT" \
  --dataset.repo_id="$EVAL_REPO" \
  --dataset.single_task="$TASK" \
  --dataset.num_episodes="$N" \
  --dataset.episode_time_s="$EPISODE_S" \
  --dataset.reset_time_s="$RESET_S" \
  --dataset.push_to_hub=false \
  --display_data="$DISPLAY_DATA" \
  --resume="$RESUME"

echo
echo "기록 완료 → $EVAL_REPO"
echo "채점:  PYTHONPATH=$WS/src $PY -m omx_yolo.evaluate --repo-id $EVAL_REPO"
