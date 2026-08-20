# omx_yolo

OMX-AI 픽업 정책(SmolVLA)의 시각 입력을 사전학습 YOLO 검출 박스로 보강하는 주석 계층.

**LeRobot 소스는 수정하지 않는다.** `CameraConfig` 레지스트리에 `yolo_opencv` 타입을
추가해 끼워 넣는 방식이다.

---

## 구성

| 파일 | 역할 |
|---|---|
| `annotate.py` | **주석 구현 본체.** 학습 변환과 실시간 추론이 이 `Annotator` 하나를 공유한다 |
| `camera.py` | LeRobot 에 `yolo_opencv` 카메라 타입 등록. `read()` 마다 주석 통과 |
| `geometry.py` | 진열대/적재함 고정 좌표. **일부 미검증 — 아래 참조** |
| `measure.py` | 실제 리그에서 ROI 를 실측하는 도구 |
| `convert.py` | 기존 데이터셋 → 주석 데이터셋 오프라인 변환 |
| `verify.py` | 변환 결과 검증 |

---

## 데이터셋 변환

```bash
export PYTHONPATH=/home/newuser/il_ws/src:$PYTHONPATH
export YOLO_AUTOINSTALL=false

# 먼저 계획만 확인 (아무것도 쓰지 않는다)
python -m omx_yolo.convert --src kdy93/smart_market_prototype_3 \
                           --dst kdy93/smart_market_prototype_3_yolo --dry-run

# 실행
python -m omx_yolo.convert --src kdy93/smart_market_prototype_3 \
                           --dst kdy93/smart_market_prototype_3_yolo

# 검증
python -m omx_yolo.verify --repo-id kdy93/smart_market_prototype_3_yolo \
                          --src kdy93/smart_market_prototype_3
```

원본은 절대 수정하지 않는다. 변환이 하는 일 네 가지:

**1. 탑뷰 스트림에만 주석을 그린다.** `annotate.Annotator` 를 그대로 쓰므로
추론 경로(`camera.py`)와 완전히 같은 코드다. 핸디캠은 원본 그대로 통과한다.

**2. 스트림 이름을 정규화한다** (`--canonical`, 기본 켜짐). 출력의
`observation.images.front` 는 항상 탑뷰, `wrist` 는 항상 핸디캠이 된다.
`--topview auto` (기본)가 에피소드마다 어느 쪽이 고정 탑뷰인지 판별하므로,
`prototype_4` ep0~11 처럼 뒤바뀐 구간도 자동 교정된다.

**3. task 문자열을 prototype_4 형식으로 정규화한다** (`--task-policy`).

```
"Pick up 1 sandwich and place it in the box1"       → "Pick up sandwich and place it in the box1"
"Pick up 2nd milk carton and place it in the box2"  → "Pick up milk and place it in the box2"
"...place them in the box2~"                        → "~" 제거
"ick up 2nd milk carton..."                         → "Pick up ..."
```

> ⚠️ **수량 2 이상 에피소드는 단일 픽업 형식으로 바꿀 수 없다.** 한 에피소드에
> 픽업이 2~3회 들어 있어서, 단일 픽업 프롬프트로 라벨을 바꾸면 정책이 "한 번
> 집어라"는 지시에 세 번 집도록 학습된다. 기본값(`normalize`)은 그런 에피소드를
> **원본 라벨 그대로** 남긴다. 버리려면 `--task-policy single-only`.
>
> 참고 — 단일 픽업 에피소드 비율: `prototype_1`·`prototype_3` 은 6개 task 중 2개
> (1/3), `prototype_2` 는 30개 task 중 18개(`1st`/`2nd`/`3rd` 형식, 3/5).
> `prototype_2` 가 단일 픽업 데이터를 가장 많이 갖고 있다(180 에피소드).

**4. 비정상 에피소드를 걸러낸다** (`--min-frames`, `--skip-episodes`).
300프레임(10초) 미만은 픽업을 완주할 수 없으므로 항상 경고한다.
실측 결과 `prototype_1` 의 **ep150 이 60프레임(2초)인데 task 는 "3 sandwiches"** 다.
`--min-frames 300` 으로 제외할 것.

### 처리 속도와 용량

실측 **31.6 fps**, 프레임당 약 25 KB(두 스트림 합).

| 데이터셋 | 프레임 | 용량 | 소요 시간 |
|---|---|---|---|
| prototype_3 | 116,807 | 약 2.9 GB | 약 1.0 시간 |
| prototype_1 | 234,095 | 약 5.9 GB | 약 2.1 시간 |
| prototype_2 | 378,133 | 약 9.5 GB | 약 3.3 시간 |
| **합계** | **729,035** | **약 18 GB** | **약 6.4 시간** |

병목 분해(프레임당 31.6 ms): 영상 디코딩 두 스트림 약 18 ms, YOLO `track()`
5.6 ms, 중간 PNG 쓰기 약 8 ms.

디코딩은 `decode_video_frames` 로 250프레임씩 배치 처리한다. `ds[i]` 개별
접근은 매 프레임 AV1 디코딩을 다시 시작해 26 fps 인데, 배치는 110 fps 다
(4.2배). 두 방식의 출력은 비트 단위로 동일함을 확인했다.

`--limit N` 으로 앞 N개 에피소드만 변환해 먼저 확인하는 것이 안전하다.

### 검증에서 반드시 볼 것

`verify.py` 의 세 열이 핵심이다.

- **고정도** — `front` 가 0.6 이상이어야 탑뷰다. 낮으면 스트림 정규화가 실패했다.
- **팔레트픽셀** — `front` 가 1% 이상이어야 주석이 살아 있다. 0에 가까우면
  인코딩이 선을 지웠거나 주석이 안 그려졌다.
- **상태·액션 최대 차이** — 0 이어야 한다. 에피소드를 제외했다면 인덱스가
  어긋나 커지는 것이 정상이다.

---

## 빠른 시작

```bash
export PYTHONPATH=/home/newuser/il_ws/src:$PYTHONPATH
export YOLO_AUTOINSTALL=false      # 아래 "런타임 자동 설치" 참조

# ROI 실측 (팔이 적재함을 가리지 않은 상태에서)
python -m omx_yolo.measure --camera /dev/omx_cam_top

# 출력된 값을 geometry.py 에 옮겨 적은 뒤 확인
python -m omx_yolo.measure --image frame.png --verify
```

추론/수집에서 쓸 때:

```bash
lerobot-record \
  --robot.type=omx_follower --robot.port=/dev/omx_follower \
  --robot.cameras="{
      front: {type: yolo_opencv, index_or_path: /dev/omx_cam_top,
              width: 640, height: 480, fps: 30, fourcc: MJPG, warmup_s: 2,
              weights: /home/newuser/il_ws/models/omx_goods_yolo11n.pt},
      wrist: {type: opencv, index_or_path: /dev/omx_cam_hand,
              width: 640, height: 480, fps: 30, fourcc: MJPG}
   }" \
  --dataset.episode_time_s=90 --dataset.push_to_hub=false
```

진입 스크립트에서 `import omx_yolo` 가 먼저 실행되어야 타입이 등록된다.

### `fourcc: MJPG` 를 반드시 지정할 것

생략하면 탑뷰 카메라가 **YUYV 로 열려 22 fps** 가 되고, LeRobot 이 다음과 같이
연결을 거부한다:

```
RuntimeError: YoloOpenCVCamera(/dev/omx_cam_top) failed to set fps=30 (actual_fps=22.0)
```

이 카메라는 `640x480` 에서 MJPG 는 30 fps, YUYV 는 22 fps 만 지원한다
(`v4l2-ctl -d /dev/omx_cam_top --list-formats-ext`).

### 워밍업

탑뷰 카메라는 **첫 프레임의 화이트밸런스가 크게 녹색으로 치우친다.** 약
30프레임(1초) 후 안정되고 그 뒤로는 완전히 일정하다(`B=124.2 G=138.1 R=145.6`
로 고정). `warmup_s: 2` 를 주면 충분하다.

### 실측 지연

`read()` 한 번에 **33.9 ms** — 30 fps 프레임 간격(33.3 ms)과 거의 같다. 즉
카메라 캡처 대기가 지배적이고 YOLO 는 사실상 공짜다(`track()` 단독 5.6 ms).

---

## ⚠️ 카메라 배정 불일치 — 이 패키지를 쓰기 전에 해결할 문제

기존 데이터셋을 조사하다 발견한 사실이다. **`smart_market_prototype_4` 의
에피소드 0~11 에서 두 카메라 스트림이 서로 뒤바뀌어 있다.**

판별 지표는 **중위 프레임 안정 픽셀 비율** — 한 파일에서 프레임 10장을 뽑아
픽셀별 중위 이미지를 만들고, 각 프레임이 그 중위값 근처(±18)에 머무는 픽셀의
비율을 센다. 고정 카메라는 팔이 지나는 영역만 변하므로 높게, 손목 카메라는
전 화면이 변하므로 낮게 나온다. `prototype_4` 4개 파일에 대해 육안 확인
결과와 100% 일치했다.

전체 87개 비디오 파일 측정 결과:

| 데이터셋 | 파일 수 | `front` 점수 | `wrist` 점수 | 탑뷰 스트림 |
|---|---|---|---|---|
| prototype_1 | 18 + 13 | 0.89 ~ 0.97 | 0.26 ~ 0.38 | `front` (전 구간) |
| prototype_2 | 43 + 33 | 0.79 ~ 0.96 | 0.25 ~ 0.42 | `front` (전 구간) |
| prototype_3 | 13 + 12 | 0.87 ~ 0.97 | 0.28 ~ 0.36 | `front` (전 구간) |
| **prototype_4** | 5 + 5 | file-000/001 **0.30** · file-002~004 **0.91~0.95** | file-000/001 **0.95~0.97** · file-002~004 **0.27~0.32** | **ep0~11 `wrist` / ep12~19 `front`** |

즉 **정상 구성은 `front` = 탑뷰이고, 뒤바뀐 것은 `prototype_4` 의 ep0~11
12개 에피소드뿐이다.** 전체 약 557 에피소드 중 12개다.

두 군집(0.79~0.97 대 0.25~0.42)은 명확히 갈리므로 판정에 모호함은 없다.
(임계값을 0.85 로 잡으면 prototype_2 의 일부 파일이 오분류된다. 0.6 을 쓸 것.)

영향: 그 12개 에피소드는 같은 입력 채널에 다른 시점을 넣으므로 학습을
오염시킨다. `prototype_4` 는 20 에피소드짜리 소규모 데이터셋이므로 폐기하거나
ep12~19 만 쓰는 것이 간단하다. **주력인 prototype_1~3 은 영향 없다.**

### 뒷받침하는 정황

1. **이미지 통계 불연속** — 에피소드별 `stats/observation.images.wrist/mean` 이
   ep 0~11 은 0.57~0.59, ep 12~19 는 0.51~0.52 로 급변한다.
2. **놓는 위치 라벨 오염** — 그리퍼가 열리는 순간의 `shoulder_pan`:

   | task | shoulder_pan |
   |---|---|
   | box1 (ep 0~9) | −27.81 ± 0.50 |
   | box2 (**ep 10, 11**) | **−27.91, −26.20** ← box1 값 |
   | box2 (ep 12~19) | −19.46 ± 0.34 |

   `ep10`, `ep11` 은 task 가 `box2` 인데 팔은 box1 위치에 놓았다.
   ep12~19 를 제외하면 두 위치는 약 20σ 로 완전히 분리되므로, 이 두
   에피소드는 오염된 것이 거의 확실하다.

가장 그럴듯한 원인은 **USB 재열거로 `/dev/video*` 번호가 재배치**되어
`index_or_path: 6` / `4` 같은 원시 인덱스가 서로 다른 물리 카메라를 가리키게
된 것이다. `ep12` 시점에 재배치가 일어났고, `ep10~11` 은 그 직전 구간이다.

### 해야 할 일

1. **udev 이름으로 전환** — `index_or_path: 6` 대신 `/dev/omx_cam_top`.
   통합 규칙은 `~/il_ws/udev/99-omx.rules` 에 있고 `apply.sh` 로 설치한다.
   구 `99-omx-cameras.rules` 는 `MODE="0666o"` 오타와 포트 경로 불일치로
   절반이 작동하지 않았다.
2. **`prototype_4` 처리 결정** — ep0~11 을 버리고 ep12~19 만 쓰거나,
   데이터셋 전체를 폐기한다 (20 에피소드뿐이라 손실이 작다).
3. **`ep10`, `ep11` 은 별도 문제** — 카메라 뒤바뀜과 무관하게 놓는 위치가
   task 라벨과 어긋난다. ep0~11 을 버리면 함께 해결된다.

### 앞으로 재발을 막는 방법

카메라를 `index_or_path: /dev/omx_cam_top` 처럼 **udev 이름으로 지정**하면
USB 재열거로 `/dev/video*` 번호가 바뀌어도 같은 물리 카메라를 가리킨다.
통합 규칙은 포트 경로가 아니라 `vendor:product` 로 키를 잡으므로 카메라를
다른 포트에 옮겨 꽂아도 이름이 따라온다.

수집 시작 전 확인 습관:

```bash
ls -l /dev/omx_cam_*        # 두 링크가 다른 videoN 을 가리키는지
python -m omx_yolo.measure --camera /dev/omx_cam_top --verify
```

---

## geometry.py — 전 상수 검증 완료 (2026-08-17)

| 상수 | 값 |
|---|---|
| `CARDBOARD_UNION` | `(0, 82, 285, 480)` |
| `SHELF_ROI` | `(285, 0, 640, 480)` |
| `BOX1_ROI` | `(0, 82, 285, 309)` — 상단, 로봇 팔과 가까운 쪽 |
| `BOX2_ROI` | `(0, 309, 285, 480)` — 하단, 로봇 팔과 먼 쪽 |

`/dev/omx_cam_top` 에서 직접 캡처한 프레임으로 측정했다. 두 적재함이 비어 있고
진열대는 6구역 × 3개로 완전히 채워진 상태였다. 미리보기로 육안 확인함.

`box1` 이 상단이라는 근거 두 가지:
1. 사용자 확인 — "로봇 팔과 가까운 쪽이 box1". 팔이 프레임 우측 상단에 있으므로
   상단 박스가 가깝다.
2. 독립 교차 검증 — `prototype_4` 의 ep09(task = box1) 종료 프레임에서 샌드위치가
   상단 박스 `(139, 223)` 에서 검출되었다.

카메라나 적재함·진열대를 움직이면 전부 무효다. 재측정:

```bash
python -m omx_yolo.measure --camera /dev/omx_cam_top --box1 upper
```

---

## 실제 리그에서의 검출 정확도

진열대가 6종 × 3개 = 18개로 채워진 프레임에서 `conf × iou` 격자 탐색을 했다.

| conf | iou | 진열대 검출 | 오차합 | 적재함 오검출 |
|---|---|---|---|---|
| 0.30 | 0.70 | 19 | 3 (`milk` 5개 — 딸기케잌 오검출) | 1 |
| **0.35** | **0.90** | **16** | **2** (`cake` −1, `sandwich` −1) | 1 |
| 0.40 | 0.90 | 15 | 3 | 1 |
| 0.35 | 0.95 | 23 | 5 (`biscuit` 6개 — 중복 검출) | 1 |

`iou` 기본값 0.70 은 인접한 동종 물체를 병합해 버린다. 0.90 으로 올리면 병합이
줄고, 0.95 는 같은 물체를 중복 검출한다. `conf` 를 0.30 이하로 내리면 딸기케잌이
`milk` 로 오검출된다.

**한계를 분명히 해 둘 것: 최적 설정에서도 18개 중 16개다.** 이 검출기는 팀원의
리그에서 학습된 것이고(보고 mAP50 0.995), 이 리그의 조명·각도가 다르다. 과거
데이터셋 프레임에서는 잘 작동했지만 진열대가 꽉 찬 상태는 처음 보는 조건이다.

주석 품질이 정책 성능의 상한을 결정하므로, 개선이 필요하면 **이 리그에서 찍은
프레임 200~300장을 라벨링해 파인튜닝**하는 것이 확실한 방법이다. 그때
`box1`/`box2` 2클래스를 함께 추가하면 고정 좌표에 의존하지 않아도 된다.

---

## 규약 (변경하면 기존 주석 데이터셋 전부 무효)

| 항목 | 값 | 근거 |
|---|---|---|
| 선 두께 | 타겟 4px / 비타겟 2px | SmolVLA 가 512×512 로 패딩 리사이즈(스케일 0.8) → 3.2px / 1.6px |
| 안티앨리어싱 | 끔 (`LINE_8`) | 반투명 경계 픽셀이 영상 압축에서 먼저 소실 |
| 채우기 / 텍스트 | 금지 | 물체 픽셀 가림 / 512 리사이즈 후 판독 불가 |
| `conf` | 0.35 | 32프레임 실측 최적점 (`KEEP_CLASSES` 적용 전제) |
| `classes` | `[0,1,3,4,5,6]` | `coke`/`yogurt` 제외 → 오검출 0 |
| `imgsz` | 640 | `best.pt` 학습값과 일치 |
| 타겟 강조 범위 | `SHELF_ROI` 내부만 | 적재함에 이미 담긴 동종 상품이 타겟으로 표시되는 것 방지 |

### 검출기

`best.pt` (팀원 제공, 2026-08-04) — YOLO11n, 2.59M 파라미터, 8클래스.
보고 지표 mAP50 0.995 / mAP50-95 0.886.

탑뷰 프레임 32장 실측: 검출 0개 프레임 0/32, 프레임당 11~19개.
`conf=0.35` + `classes` 필터에서 오검출 0. 지연 `track()` 포함 5.6 ms (RTX 5080).

**`box1`/`box2` 클래스는 없다.** 그래서 적재함은 검출이 아니라 `geometry.py` 의
고정 좌표로 그린다.

### 클래스 매핑

| 모델 클래스 | 상품 | 색 (RGB) |
|---|---|---|
| `sandwich` | 샌드위치 | 220, 84, 32 |
| `milk` | 초코우유 | 246, 176, 28 |
| `icecream` | 아이스크림 | 40, 96, 188 |
| `cake` | 케잌 | 150, 62, 196 |
| `biscuit` | 비스킷 | 54, 168, 96 |
| `roll` | 롤케잌 | 23, 176, 184 |
| `coke`, `yogurt` | — | 제외 |

`cake` 와 `roll` 은 부분문자열 관계가 아니므로 SmolVLA task 문자열에서도
이 이름을 그대로 쓰면 토큰이 분리된다.

---

## 런타임 자동 설치를 반드시 끌 것

`track()` 을 처음 호출할 때 ultralytics 가 ByteTrack 의존성 `lap` 이 없음을
감지하고 **런타임에 `pip install` 을 실행한다.** 로봇 제어 루프 중에 이런 일이
일어나면 치명적이다.

`lap` 은 이미 설치되어 있고, 추가로 `YOLO_AUTOINSTALL=false` 를 설정하라
(`ultralytics/utils/__init__.py:70` 이 이 변수를 읽는다).

---

## 환경

`venv/il` 에 `--no-deps` 로 설치되어 있다. `opencv-python-headless 4.12.0.88`
을 유지하기 위한 것이다 — `ultralytics` 는 `opencv-python` 을 요구하지만
LeRobot 은 `opencv-python-headless<4.13.0` 으로 제약하므로, 그냥 설치하면
두 배포판이 중복되어 `cv2` 가 섀도잉된다.

```
lerobot 0.4.4 (editable)   torch 2.10.0+cu128   numpy 2.2.6
opencv-python-headless 4.12.0.88   ultralytics 8.4.120   lap 0.5.13
```

`pip` 가 `ultralytics requires opencv-python ... which is not installed` 경고를
내는 것은 정상이다. 메타데이터 상 배포판 이름만 다르고 동일한 `cv2` 모듈이 제공된다.
