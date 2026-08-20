#!/usr/bin/env bash
# 롤아웃 공통 설정. 다른 스크립트가 source 한다. 직접 실행하지 않는다.
#
# 여기 값 하나가 학습 때와 어긋나면 정책이 배운 적 없는 입력을 받는다.
# 특히 카메라 키(camera1/camera2)와 주석 설정은 학습 시점 값과 반드시 같아야 한다.

export PYTHONPATH=/home/newuser/il_ws/src:${PYTHONPATH:-}
export YOLO_AUTOINSTALL=false        # 제어 루프 중 pip install 방지

PY=/home/newuser/venv/il/bin/python
WS=/home/newuser/il_ws
WEIGHTS=$WS/models/omx_goods_yolo11n.pt

# ── 정책 체크포인트 ──────────────────────────────────────────────
# 학습 완료 후 last, 중간 평가는 checkpoints/030000 처럼 지정 가능
CKPT=${CKPT:-$WS/src/lerobot/outputs/train/v1_yolo/checkpoints/last/pretrained_model}

# ── 기록 대상 ────────────────────────────────────────────────────
# 조건 12개를 한 저장소에 이어 붙인다. evaluate.py 가 에피소드별 task 로
# 알아서 조건을 갈라 채점하므로 저장소를 나눌 이유가 없다.
HF_USER=${HF_USER:-kdy93}
EVAL_REPO=${EVAL_REPO:-$HF_USER/eval_v1_yolo}

# ── 학습과 반드시 일치해야 하는 값 ──────────────────────────────
# 학습은 --rename_map 으로 front→camera1, wrist→camera2 로 바꿔 돌렸다.
# 그래서 정책이 아는 이름은 camera1 / camera2 다.
#
# 롤아웃에서는 rename_map 을 쓰지 않고 **카메라를 처음부터 그 이름으로 만든다.**
# lerobot_record.py:505 가 make_policy(cfg.policy, ds_meta=dataset.meta) 를
# rename_map 없이 호출하기 때문이다. factory.py:525 는
#
#     if not rename_map:
#         validate_visual_features_consistency(cfg, features)
#
# 이므로, rename_map 을 넘기지 않은 record 경로에서는 검증이 그대로 돈다.
# 검증은 rename 전 이름(front/wrist)을 정책 이름(camera1/2/3)과 비교해 죽는다:
#
#     ValueError: Feature mismatch ...
#     - Missing: ['...camera1', '...camera2', '...camera3']
#     - Extra:   ['...front', '...wrist']
#
# lerobot_train.py:239 는 rename_map 을 넘기므로 학습에서는 안 터졌다.
# 이 차이가 학습은 되는데 롤아웃만 죽는 이유다.
#
# LeRobot 소스를 고치지 않는 것이 이 프로젝트의 원칙이므로, 카메라 키를
# camera1 / camera2 로 두어 rename 자체를 불필요하게 만든다. 검증은
# {camera1, camera2} ⊂ {camera1, camera2, camera3} 로 통과한다
# (policies/utils.py:245 — 어느 한쪽이 부분집합이면 합격).
#
#   camera1 = 탑뷰(주석 O)   ← 학습에서 front 가 매핑된 자리
#   camera2 = 손목(주석 X)   ← 학습에서 wrist 가 매핑된 자리
#
# 순서를 바꾸면 정책이 두 시점을 뒤바꿔 받는다. 절대 바꾸지 말 것.

# ── 시간 설정 ────────────────────────────────────────────────────
# episode_time_s 를 크게 두면 안 된다. 이전 eval_* 8개가 전부 0 에피소드인
# 이유가 이것이다 — 100000 으로 두고 Ctrl+C 로 죽여 아무것도 저장되지 않았다.
# 90초는 prototype_2 최장 에피소드(92초) 기준.
EPISODE_S=${EPISODE_S:-90}
RESET_S=${RESET_S:-20}               # 리셋: 적재함 비우기 / 3회마다 진열대 리필

DISPLAY_DATA=${DISPLAY_DATA:-true}   # rerun 실시간 확인. 부담되면 false

# ── 로봇 / 카메라 ────────────────────────────────────────────────
# 카메라는 udev 이름으로만 지정한다. /dev/videoN 은 USB 재열거로 뒤바뀐다
# (prototype_4 ep0~11 오염의 원인).
ROBOT_ARGS=(
  --robot.type=omx_follower
  --robot.port=/dev/omx_follower
  --robot.id=omx_follower_arm
)

# 탑뷰만 yolo_opencv. 손목은 원본 그대로 — 학습 데이터와 같은 구성이다.
# task 는 호출부에서 채운다 (타겟/목적지 굵게 그리기에 쓰인다).
cameras_arg() {
  local task="$1"
  cat <<EOF
{
  camera1: {type: yolo_opencv, index_or_path: /dev/omx_cam_top,
            width: 640, height: 480, fps: 30, fourcc: MJPG, warmup_s: 2,
            weights: $WEIGHTS,
            task: "$task"},
  camera2: {type: opencv, index_or_path: /dev/omx_cam_hand,
            width: 640, height: 480, fps: 30, fourcc: MJPG}
}
EOF
}

# ── 상품 표기 ────────────────────────────────────────────────────
# 유일한 출처는 annotate.PRODUCT_PHRASE 다. 여기에 값을 복사해 두면 언젠가
# 어긋나므로, 실행할 때마다 그 표에서 직접 읽어 검증한다.
#
# 왜 이렇게까지 하는가 (2026-08-19 실제 사고):
#   ./rollout.sh milk box1 5  →  "Pick up milk and place it in the box1"
# 학습에 쓰인 문장은 "Pick up milk carton and ..." 이다. 정책은 본 적 없는
# 문장을 받고 엉뚱한 물체(샌드위치)로 갔다. 5 에피소드를 통째로 버렸다.
# 클래스명(milk)과 지시문 표기(milk carton)가 다른 것은 milk 뿐이라 눈에
# 잘 띄지 않는다. 그래서 사람이 아니라 스크립트가 막아야 한다.
PRODUCTS=(sandwich "milk carton" icecream cake biscuit roll)

# 클래스명이든 지시문 표기든 받아서 정식 표기로 바꾼다. 목록에 없으면 실패.
normalize_product() {
  $PY - "$1" <<'PYEOF'
import sys
from omx_yolo.annotate import PRODUCT_PHRASE
q = sys.argv[1].strip().lower()
phrases = {v.lower(): v for v in PRODUCT_PHRASE.values()}
classes = {k.lower(): v for k, v in PRODUCT_PHRASE.items()}
if q in phrases:
    print(phrases[q])
elif q in classes:
    print(classes[q], file=sys.stdout)
    print(f"  ℹ 클래스명 '{q}' → 지시문 표기 '{classes[q]}' 로 보정했습니다.",
          file=sys.stderr)
else:
    print(f"알 수 없는 상품: {sys.argv[1]!r}", file=sys.stderr)
    print("  쓸 수 있는 값: " + ", ".join(sorted(set(
        list(PRODUCT_PHRASE) + list(PRODUCT_PHRASE.values())))), file=sys.stderr)
    sys.exit(2)
PYEOF
}

build_task() {   # build_task <상품표기> <box1|box2>
  echo "Pick up $1 and place it in the $2"
}

preflight() {
  local fail=0
  echo "── 사전 점검 ─────────────────────────────────"
  for dev in /dev/omx_follower /dev/omx_cam_top /dev/omx_cam_hand; do
    if [ -e "$dev" ]; then
      printf "  %-22s → %s\n" "$dev" "$(readlink -f $dev)"
    else
      printf "  %-22s 없음 ✗\n" "$dev"; fail=1
    fi
  done
  [ -f "$WEIGHTS" ] || { echo "  검출기 없음 ✗ $WEIGHTS"; fail=1; }
  if [ ! -d "$CKPT" ]; then
    echo "  체크포인트 없음 ✗ $CKPT"
    echo "    (학습이 끝나지 않았거나 경로가 다릅니다)"
    fail=1
  else
    echo "  체크포인트           → $CKPT"
  fi
  if [ $fail -eq 0 ]; then
    echo "  리그 정합 확인 중..."
    local rig
    rig=$($PY -m omx_yolo.checkrig 2>&1 | grep -viE "Corrupt JPEG|WARNING ⚠️|View Ultra|Update Sett")
    echo "$rig" | grep -E "평균 이동량|최대 이동량|매칭률|✅|⚠ |❌" | sed 's/^/  /'
    # checkrig 는 판정과 무관하게 항상 0 으로 끝난다. 출력으로 판정한다.
    if echo "$rig" | grep -q "❌"; then
      echo "  ✗ 리그가 기준과 어긋났습니다 — 롤아웃을 중단합니다."
      echo "    확인:  eog /home/newuser/il_ws/models/rig_check.png"
      fail=1
    fi
  fi
  echo "──────────────────────────────────────────────"
  return $fail
}
