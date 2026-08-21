# OMX 로봇팔 — 픽업 · 포장 정책

> **관제서버 담당자에게** — 픽업과 포장은 **별개 서버**입니다. 포트가 다릅니다.
>
> | | 규격 | 적용 방법 |
> |---|---|---|
> | **픽업** `:8080` | [`API.md`](API.md) | [`scripts/controller_patch/`](scripts/controller_patch/) |
> | **포장** `:8081` | [`API_PACK.md`](API_PACK.md) | [`scripts/controller_patch_pack/`](scripts/controller_patch_pack/) |
>
> 각 `controller_patch*/` 에 붙여넣을 어댑터 코드와 `orders.py` 수정 위치가
> 들어 있습니다.
>
> **두 팔은 별개 하드웨어라 동시에 돌 수 있습니다** — 같은 락으로 묶지
> 마십시오. 실행 환경도 다릅니다(픽업 lerobot 0.4.4 / 포장 0.6.1). 두
> 프로세스를 각각 띄워야 합니다.


무인 마트에서 관제서버의 요청을 받아 진열대의 상품을 집어 카트 적재함에 담는다.
정책은 SmolVLA 행동복제이고, 시각 입력에 YOLO 검출 박스를 그려 넣는 것이 이
구현의 핵심이다. **학습 데이터와 실시간 추론이 같은 주석 코드를 공유한다.**

```
관제서버 ──HTTP──> OMX 서버 ──> 정책(SmolVLA) ──> 로봇팔
 :4100              :8080          ▲
                                   └── 탑뷰 카메라 + YOLO 주석
```

담당 범위는 **픽업**이다. 패킹은 별도 담당이다.

---

## 실행

```bash
export PYTHONPATH=/home/newuser/il_ws/src
export YOLO_AUTOINSTALL=false          # 제어 루프 중 pip install 방지

python -m omx_yolo.server \
    --policy <체크포인트>/pretrained_model \
    --port 8080
```

브라우저에서 `http://localhost:8080/view` 를 열면 탑뷰(주석 포함)와 손목 화면,
진행 상태가 함께 보인다.

> 정책 체크포인트(1.3 GB), 학습 데이터(3.4 GB), 검출기 가중치(5.3 MB)는
> 저장소에 없다. 별도로 받아야 한다.

---

## API

관제서버가 부르는 HTTP API의 **전체 규격은 [`API.md`](API.md) 에 있다.**
요약만 적으면:

| | |
|---|---|
| `POST /pick` | `{orderId, deviceCode, slug, quantity}` → `202`, 즉시 반환 |
| `GET /pick/state` | 진행 상태 폴링. `done` 이 지금까지 담은 개수 |
| `POST /pick/stop` | `{"mode":"afterCurrent"｜"immediate"}` |
| `GET /health` | 장치·리그 점검. 관제의 `is_reachable()` 이 쓴다 |
| `GET /products` | 집을 수 있는 상품 slug 목록 |
| `POST /home` | 홈 자세 복귀 (복구용) |
| `GET /view` `/stream` `/frame.jpg` | 화면 |

관제서버 담당자는 **[`API.md`](API.md)** 와
**[`scripts/controller_patch/`](scripts/controller_patch/)** 두 곳만 보면 된다.

## 관제서버가 알아야 할 제약 세 가지

**① 한 번에 3개까지.** 진열대 한 칸에 상품이 3개뿐이다. 초과하면 `400` 이다.

**② 한 개라도 실패하면 작업 전체를 중단한다.** 정책이 가까운 것부터 집는
순서를 어기면 진열 상태가 학습에 없는 형태가 되고, 그 뒤 픽업은 빈 칸을
헛집는다. 실패를 안고 계속 가면 남은 시도까지 버린다. `status:"FAILED"` 와
함께 그때까지의 `done` 을 돌려준다.

**③ 팔은 하나다.** 두 번째 요청은 `409` 다. 카트 두 대가 같은 매대에 몰리면
관제가 순서를 조율해야 한다.

---

## 상품 어휘

관제 `products.slug` ↔ 내부 클래스 ↔ 지시문 표기가 서로 다르다.
`omx_yolo/annotate.py` 의 `CONTROLLER_SLUG` 가 **유일한 출처**다.

| 관제 slug | 웨이포인트 | 내부 클래스 | 지시문 |
|---|---|---|---|
| `milk` | W3 | `milk` | **`milk carton`** |
| `biscuit` | W4 | `biscuit` | `biscuit` |
| `ice-cream` | W5 | `icecream` | `icecream` |
| `roll-cake` | W2 | `roll` | `roll` |
| `cake` | W1 | `cake` | `cake` |
| `sandwich` | W6 | `sandwich` | `sandwich` |

`cola` 는 지원하지 않는다 — 검출기가 오검출 때문에 `coke` 클래스를 제외했고
정책도 학습한 적이 없다. 카탈로그에서 `cola` 를 빼고 `biscuit` 을 넣기로
합의했다. 그래도 요청이 오면 `400` 으로 거절한다.

카트 매핑: `cart-1 → box1`(로봇과 가까운 적재함), `cart-2 → box2`.

---

## 성능 (2026-08-20 기준선)

12개 조건 63 에피소드, 전체 **81.0%** (95% 신뢰구간 69.6–88.8).
픽업 1회 평균 26.8초.

| 조건 | 성공률 | | 조건 | 성공률 |
|---|---|---|---|---|
| sandwich → box1/box2 | 100% / 100% | | roll → box1/box2 | 66.7% / 100% |
| biscuit → box1/box2 | 100% / 80% | | icecream → box1/box2 | 57.1% / 100% |
| cake → box1/box2 | 100% / 60% | | milk → box1/box2 | 80% / 40% |

**조건당 표본이 5개라 조건 사이의 차이는 통계적으로 구분되지 않는다.**
5/5(100%)의 신뢰구간은 [56.6%, 100%], 2/5(40%)는 [11.8%, 76.9%] 로 겹친다.
"대체로 80% 수준" 이상으로 읽으면 안 된다.

실패 12건은 **전부 파지에서** 났다. 동작 완주 63/63, 목적지 정확도 62/63 이다.

---

## 구성

| 파일 | 역할 |
|---|---|
| `omx_yolo/server.py` | **HTTP 서버 본체.** 작업 모델·인터럽트·화면 송출 |
| `omx_yolo/annotate.py` | 주석 구현 + 상품 어휘 매핑. 학습과 추론이 공유한다 |
| `omx_yolo/camera.py` | LeRobot 에 `yolo_opencv` 카메라 타입 등록 |
| `omx_yolo/kinematic.py` | 성공/실패 판정 (관절 전용, 카메라 미사용) |
| `omx_yolo/success.py` | 홈 복귀 감지 — 정책이 종료 신호를 안 내므로 필요하다 |
| `omx_yolo/geometry.py` | 진열대·적재함 고정 좌표 (탑뷰 픽셀) |
| `omx_yolo/checkrig.py` | 리그가 기준 배치에서 틀어졌는지 확인 |
| `omx_yolo/convert.py` | 원본 데이터셋 → 주석 데이터셋 오프라인 변환 |
| `omx_yolo/record.py` | 롤아웃 기록 진입점 (`yolo_opencv` 등록 래퍼) |
| `omx_yolo/evaluate.py` | 조건별 성공률 채점 |
| `scripts/` | 롤아웃 도구와 절차 |
| `scripts/controller_patch/` | **관제서버 쪽에 넣을 어댑터와 적용 방법** |

관제서버 담당자는 [`API.md`](API.md)(규격)와
[`scripts/controller_patch/`](scripts/controller_patch/)(적용 방법) 두 곳만 보면 된다.

---

## 환경

```
lerobot 0.4.4 (editable)   torch 2.10.0+cu128   numpy 2.2.6
opencv-python-headless 4.12.0.88   ultralytics 8.4.120   lap 0.5.13
```

`ultralytics` 는 `opencv-python` 을 요구하지만 LeRobot 은 headless 를 제약하므로
`--no-deps` 로 설치해 중복을 피했다. `pip` 경고는 정상이다.
