# 롤아웃 기록 — 조건별 성공률 기준선

학습이 끝난 정책을 실제 팔에 물려 조건별로 돌리고, 그 기록을 채점해
**처음으로 비교 가능한 숫자**를 만든다. 지금은 기준선이 하나도 없다 —
기존 `eval_*` 데이터셋 8개가 전부 0 에피소드다.

```
rollout_env.sh    공통 설정. source 전용, 직접 실행하지 않는다
rollout.sh        조건 하나 기록
rollout_all.sh    12개 조건 순회
```

---

## 순서

```bash
cd ~/il_ws/scripts

# 1) 장치를 꽂고 사전 점검만 먼저
bash -c 'source ./rollout_env.sh; preflight'

# 2) 한 조건만 시험 (5회)
./rollout.sh sandwich box1 5

# 3) 결과 확인
PYTHONPATH=~/il_ws/src ~/venv/il/bin/python -m omx_yolo.evaluate \
    --repo-id kdy93/eval_v1_yolo --verbose

# 4) 문제 없으면 전 조건
./rollout_all.sh 5        # 조건당 5회 = 60 에피소드, 약 110분
```

조건 12개는 한 저장소 `kdy93/eval_v1_yolo` 에 이어 붙인다. `evaluate.py` 가
에피소드별 task 문자열로 조건을 갈라 채점하므로 저장소를 나눌 이유가 없다.
두 번째 실행부터 `--resume=true` 가 자동으로 붙는다.

**합격선** — 조건 평균 80% 이상, 어떤 조건도 50% 미만 없음.

---

## 이전에 실패한 이유를 반복하지 않기 위한 세 가지

**1. `episode_time_s` 를 크게 두지 않는다.**
기존 `eval_*` 8개가 전부 빈 이유가 이것이다. `100000` 으로 두고 `Ctrl+C` 로
죽이면 LeRobot 은 **아무것도 저장하지 않는다.** 여기서는 90초로 고정했다
(prototype_2 최장 에피소드 92초 기준).

다만 90초는 **안전 상한이지 목표가 아니다.** 정책은 종료 신호를 배우지 않아
한 번 넣고 홈으로 돌아온 뒤에도 빈 공간을 계속 집으려 든다. 그대로 두면 한
에피소드에 픽업이 여러 번 들어가 채점이 깨진다.

| 키 | 동작 |
|---|---|
| **→** | 지금 에피소드를 끝내고 **저장**, 다음으로 |
| **←** | 지금 에피소드를 버리고 **재녹화** |
| **ESC** | 전체 기록 정상 종료 |
| ~~Ctrl+C~~ | 쓰지 말 것 — 저장되지 않는다 |

**팔이 물건을 넣고 홈 자세로 돌아온 순간 → 를 누른다.** 그래야
"에피소드 1개 = 픽업 시도 1회" 가 되어 `evaluate.py` 의 판정이 성립한다.

**2. 카메라는 udev 이름으로만 지정한다.**
`/dev/video6` 같은 원시 인덱스는 USB 재열거로 뒤바뀐다. prototype_4 ep0~11
12개 에피소드가 이것 때문에 오염됐다. 스크립트는 `/dev/omx_cam_top`,
`/dev/omx_cam_hand` 만 쓴다.

**3. 카메라 키는 `camera1` / `camera2` 다 (rename_map 을 쓰지 않는다).**
학습은 `--rename_map` 으로 `front→camera1`, `wrist→camera2` 로 돌렸다. 그런데
`lerobot_record.py:505` 는 `make_policy()` 에 rename_map 을 넘기지 않아
`factory.py:525` 의 검증이 **rename 이전 이름**으로 돌고 죽는다:

```
ValueError: Feature mismatch ...
- Missing: ['...camera1', '...camera2', '...camera3']
- Extra:   ['...front', '...wrist']
```

`lerobot_train.py:239` 는 넘기므로 학습에서는 안 터진다. 이 차이가 **학습은
되는데 롤아웃만 죽는** 이유다. LeRobot 을 고치지 않기 위해 카메라를 처음부터
`camera1`/`camera2` 로 지어 rename 자체를 없앴다.

> **camera1 = 탑뷰(주석 O), camera2 = 손목(주석 X)** — 학습 매핑과 같은 순서다.
> 바꾸면 정책이 두 시점을 뒤바꿔 받는다.

---

## `lerobot-record` 대신 `python -m omx_yolo.record` 를 쓰는 이유

`lerobot-record` 는 entry point 스크립트라 `omx_yolo` 를 import 하지 않는다.
그러면 `CameraConfig` 레지스트리에 `yolo_opencv` 가 없어 draccus 가 이렇게 죽는다:

```
ValueError: Unknown choice 'yolo_opencv' for CameraConfig
```

수집(8/18)은 전부 `type: opencv` 로 했고 주석은 `convert.py` 가 오프라인으로
그렸기 때문에 이 경로를 한 번도 타지 않았다. **실시간 주석이 필요한 롤아웃에서
처음 쓰인다.** `omx_yolo/record.py` 가 import 후 같은 `main()` 을 부르는
얇은 래퍼다. 플래그는 `lerobot-record` 와 완전히 같다.

등록 확인:

```bash
PYTHONPATH=~/il_ws/src ~/venv/il/bin/python -c \
  "import omx_yolo; from lerobot.cameras.configs import CameraConfig; \
   print(sorted(CameraConfig._registry))"
# → ['opencv', 'yolo_opencv']
```

---

## 카메라 `task:` 필드

`lerobot-record` 는 카메라에 `set_task()` 를 호출해 주지 않는다. 그런데 학습
데이터에는 **타겟 상품과 목적지 적재함이 굵게(4px)** 그려져 있다. 추론에서 그
강조가 빠지면 학습/추론 불일치가 된다.

그래서 카메라 설정에 지시문을 직접 넣는다. `rollout.sh` 가
`--dataset.single_task` 와 **같은 문자열**을 자동으로 채우므로 손댈 일은 없다.
직접 명령을 짤 때는 두 곳을 반드시 일치시킬 것.

---

## 진열대 리필 주기

한 칸에 상품 3개가 놓인다. 리셋 20초 동안:

1. 적재함에 들어간 물건을 빼낸다 (매 회)
2. 진열대는 **3회마다** 채운다

잔여 수량 3 → 2 → 1 을 고루 겪게 하려는 것이다. 매번 즉시 채우면 잔여 3인
상황만 측정하게 된다. 수집 때와 마찬가지로 **로봇과 가장 가까운 것부터**
집는 선입선출이다.

---

## 조건을 쪼개서 보는 이유

전체 성공률 70% 는 "모든 조건 70%" 일 수도 있고 "box2 95% / box1 45%" 일 수도
있다. 후자라면 box1 데이터만 더 모으면 된다. prototype_4 실측에서 box1 이 box2
보다 픽업 1회당 평균 35% 느렸다(42초 대 31초) — 조건 간 난이도 차이는 이미
관측된 사실이다.

---

## 판정 기준 (관절 전용, 카메라 미사용)

| 항목 | 기준 |
|---|---|
| 동작 종료 | 홈 자세 복귀 후 K프레임 유지 (`success.HomeDetector`) |
| 파지 성공 | 그리퍼 실제 최소값 ≥ `GRASP_MIN` 49.4 |
| 목적지 | 놓는 순간 `shoulder_pan` 이 경계 어느 쪽인가 |

시각 판정은 폐기했다. 검출기가 진열대 18개 중 16개만 잡고, ROI 는 리그가
조금만 밀려도 무효가 되기 때문이다(하루 사이 43px 이동으로 판정이 깨진 적 있다).

`GRASP_MIN` 여유는 0.5 로 빠듯하다(헛집기 48.99, 성공 최소 49.9). 롤아웃 중
헛집기가 나오면 그 에피소드 번호를 적어 두면 경계를 더 정확히 잡을 수 있다.

---

## 중간 체크포인트로 먼저 보고 싶다면

```bash
CKPT=~/il_ws/src/lerobot/outputs/train/v1_yolo/checkpoints/030000/pretrained_model \
EVAL_REPO=kdy93/eval_v1_yolo_30k \
./rollout.sh sandwich box1 5
```

저장소를 나눠야 최종 체크포인트 결과와 섞이지 않는다.


---

## 카메라 읽기 경로 세 개 (2026-08-19 사고 기록)

첫 롤아웃에서 `camera1` 에 박스가 하나도 그려지지 않았다. `Annotator` 도
`task` 파싱도 정상이었는데 화면은 원본이었다.

원인: `omx_follower.get_observation()` 은 `read()` 도 `async_read()` 도 아닌
**`read_latest()`** 를 부른다 (`robots/omx_follower/omx_follower.py:179`).
`camera.py` 는 앞의 둘만 감싸고 있어서 정책이 주석 없는 원본을 받았다.

학습 데이터에는 박스가 그려져 있으므로 입력 분포가 어긋난다. 그날 관측된
"물체보다 앞을 집으려 함" 은 이것으로 설명된다.

기반 클래스 `lerobot/cameras/camera.py` 의 공개 읽기 메서드는 현재
**read / async_read / read_latest** 세 개다. 셋 다 감싸야 한다.
LeRobot 을 올릴 때 이 목록이 늘었는지 확인할 것:

```bash
python -c "from lerobot.cameras.camera import Camera; \
  print([m for m in dir(Camera) if 'read' in m.lower()])"
```

검증 (팔레트 픽셀이 0 이 아니어야 한다):

```bash
PYTHONPATH=~/il_ws/src python -c "
import numpy as np
from omx_yolo import YoloOpenCVCameraConfig, YoloOpenCVCamera
from omx_yolo.annotate import RGB, BOX_RGB
P=np.array(list(RGB.values())+list(BOX_RGB.values()),dtype=np.int16)
c=YoloOpenCVCamera(YoloOpenCVCameraConfig(index_or_path='/dev/omx_cam_top',
  width=640,height=480,fps=30,fourcc='MJPG',warmup_s=2,
  task='Pick up sandwich and place it in the box1')); c.connect()
for _ in range(20): c.read()
for m in ('read','async_read','read_latest'):
    f=getattr(c,m)(); a=f.reshape(-1,3).astype(np.int16)
    print(m, round(float((np.abs(a[:,None,:]-P[None,:,:]).sum(2).min(1)<40).mean()*100),2),'%')
c.disconnect()"
```

---

## 검출기 한계 실측 (2026-08-19 21:40, 현재 리그)

첫 정상 롤아웃에서 "샌드위치·케잌 박스가 제대로 안 그려진다"는 관찰이 나와
측정했다. 진열대에 6종 × 3개 = 18개가 놓인 프레임 기준.

**설정을 어떻게 바꿔도 sandwich 와 icecream 은 0개다.**

| conf | iou | 총검출 | sandwich | icecream | biscuit | cake | milk | roll |
|---|---|---|---|---|---|---|---|---|
| 0.20 | 0.70 | 17 | 0 | 0 | 1 | 2 | 11 | 3 |
| 0.25 | 0.90 | 14 | 0 | 0 | 1 | 2 | 8 | 3 |
| **0.35** | **0.90** | **12** | **0** | **0** | **0** | **2** | **7** | **3** |
| 0.40 | 0.70 | 11 | 0 | 0 | 0 | 2 | 6 | 3 |
| 0.35 | 0.95 | 19 | 0 | 0 | 0 | 3 | 10 | 6 |

conf 를 낮추면 `milk` 오검출만 늘고(최대 17개), iou 0.95 는 `roll` 을 중복
검출한다. 현재 설정(오차합 14)과 최선(conf 0.40 / iou 0.70, 오차합 13)의
차이는 무의미하다. **설정 탐색으로는 해결되지 않는다.**

### 학습 데이터도 똑같았다 — 이것이 핵심이다

원본 `smart_market_v1` 에서 5개 에피소드를 표본 추출해 같은 검출기를 돌렸다:

| | 밝기 | 프레임당 검출 | sandwich | icecream | biscuit |
|---|---|---|---|---|---|
| 학습 데이터 (5프레임 합) | 132.3 | 8.8개 | 1 | 0 | 0 |
| 현재 리그 | 130.8 | 13개 | 0 | 0 | 0 |

밝기가 같으므로 조명 변화가 아니다. **정책은 처음부터 이 상태의 주석으로
학습했다.** 즉 학습/추론 불일치는 없고, 지금 보이는 것이 원래의 품질 상한이다.
현재 리그가 오히려 학습 프레임보다 검출이 많다.

### 이것이 뜻하는 것

6종 중 **sandwich · icecream · biscuit 세 종은 YOLO 신호를 사실상 못 받는다.**
그 조건들에서 정책은 실질적으로 순수 행동복제로 동작하고, 화면에는 `milk`
오검출 박스만 어지럽게 남는다. `roll` 만 3/3 으로 안정적이다.

### 그래서 지금 무엇을 하는가

**설정을 바꾸지 않고 기준선을 먼저 잰다.** 지금 conf/iou 를 건드리면 학습
데이터와 어긋나 그나마 있던 일관성마저 깨진다(고치려면 재변환 84분 +
재학습 7.5시간).

기준선이 나오면 판단 근거가 생긴다:

- `roll` 조건은 잘 되고 `sandwich`/`icecream`/`biscuit` 이 무너진다
  → 검출기 파인튜닝의 효과가 입증된 것. 이 리그 프레임 200~300장 라벨링 →
  재변환 → 재학습이 정당화된다.
- 조건 간 차이가 없다 → YOLO 주석이 성능에 별 기여를 못 하고 있다는 뜻.
  파인튜닝보다 데이터 추가가 우선이다.

어느 쪽이든 **먼저 숫자를 만들어야 결정할 수 있다.**

---

## 진열 상태는 학습에 존재하는 형태로만 둘 것 (2026-08-19)

수집 때 사람은 항상 로봇과 가까운 것부터 집었다(FIFO). 그래서 학습 데이터에
존재하는 진열 상태는 세 가지뿐이다:

```
[1][2][3]        [ ][2][3]        [ ][ ][3]
 3개              2개              1개
```

**`[1][ ][ ]` 는 학습 데이터에 없다.** 그런데 정책이 순서를 어겨 두 번째나 세
번째를 집으면 이런 상태가 만들어진다. 그 상태로 다음 회차를 찍으면 정책은
"물체가 하나 남았다 → 뒤쪽 칸이다" 라는 학습된 연관을 따라 **빈 칸을 헛집는다.**

실제 사례 — `milk → box2`:

| ep | 진열 상태 | 정책 행동 | 결과 |
|---|---|---|---|
| 43 | `[1][2][3]` | 두 번째를 집음 (순서 위반) | 성공 |
| 44 | `[1][ ][3]` | 세 번째를 집음 | 성공 |
| 45 | `[1][ ][ ]` | **빈 세 번째 칸**을 헛집음 | 실패 |
| 46 | `[1][ ][ ]` | **빈 세 번째 칸**을 헛집음 | 실패 |

ep45·46 은 정책의 파지 능력 문제가 아니라 **학습 분포 밖 상태를 측정한 것**이다.
기준선 숫자로 쓰면 성능을 과소평가하게 된다.

**운용 규칙** — 매 회차 시작 전에 남은 물체를 뒤쪽으로 밀어 위 세 형태 중
하나로 맞춘다. 정책이 순서를 어긴 회차는 그 사실을 적어 두고, 다음 회차 전에
정렬한다.

**설계 시사점** — 관제 서버는 정책이 FIFO 를 지킨다고 가정하면 안 된다.
`server.py` 가 픽업 후 진열 상태를 확인하거나, 매 픽업 전 진열을 정규 상태로
되돌리는 절차가 필요하다.
