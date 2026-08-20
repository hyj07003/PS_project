# OMX 픽업 서버 API

관제서버(controller-server)가 로봇팔에 픽업을 시키기 위해 부르는 HTTP API.
**이 문서가 규격의 유일한 출처다.**

```
관제서버 :4100  ──HTTP(LAN)──>  OMX 서버 :8080  ──>  로봇팔
         (관제 PC)                  (OMX/로봇팔 PC)
```

**OMX 서버는 로봇팔이 연결된 별도 PC에서 구동**한다. 관제 PC의 `.env`에
그 PC의 LAN IP를 넣는다 (예: `http://192.168.129.50:8080`).
같은 PC에서 테스트할 때만 `http://127.0.0.1:8080` 을 쓴다.
OMX 서버는 기본 `0.0.0.0:8080` 으로 bind 하므로 LAN 접속을 받는다.
인증은 없다. 방화벽에서 **OMX PC의 TCP 8080** 을 관제 PC IP(또는 서브넷)에 허용할 것.
필드는 **camelCase**, 응답은 `{"success", "status", "message"}` 형태로,
pinky 로봇 서버(`PinkyHttpCartAdapter`)와 같은 규약을 따른다.

바로 쓸 수 있는 클라이언트 구현이 [`scripts/controller_patch/omx_adapter.py`](scripts/controller_patch/omx_adapter.py) 에 있다.

---

## 흐름

픽업은 **비동기**다. `POST /pick` 은 작업을 시작하고 즉시 돌아오며,
진행 상황은 `GET /pick/state` 를 폴링해서 본다.

```
관제                                   OMX
 │                                      │
 ├── POST /pick {quantity:2} ─────────> │  작업 시작
 │ <──────────── 202 RUNNING done:0 ────┤  (즉시 반환)
 │                                      │
 ├── GET /pick/state ─────────────────> │  ┐
 │ <──────────── RUNNING done:0 ────────┤  │ 0.5초 주기 폴링
 ├── GET /pick/state ─────────────────> │  │ (pinky /nav/state 와 같은 방식)
 │ <──────────── RUNNING done:1 ────────┤  │
 │            ⋮                         │  ┘
 │ <──────────── DONE    done:2 ────────┤  완료
```

블로킹으로 두지 않은 이유: 수량 3이면 최대 4.5분이고, 그동안 관제가
진행 개수도 못 보고 중단도 못 시킨다.

---

## `POST /pick`

수량만큼 집어 카트 적재함에 담는 작업을 시작한다.

### 요청

```json
{
  "orderId": 12,
  "deviceCode": "cart-1",
  "slug": "biscuit",
  "quantity": 2,
  "timeoutSec": 90
}
```

| 필드 | 필수 | 설명 |
|---|---|---|
| `orderId` | | 관제의 주문 번호. 로그·응답에 그대로 실려 돌아온다 |
| `deviceCode` | ✔ | `cart-1` 또는 `cart-2`. `box1`/`box2` 도 받는다 |
| `slug` | ✔ | 관제 DB 의 `products.slug` 를 그대로 (아래 표 참조) |
| `quantity` | ✔ | `order_items.quantity`. **1~3** |
| `timeoutSec` | | 픽업 1회당 상한. 기본 90 |

### 응답 — `202 Accepted`

```json
{
  "success": true, "status": "RUNNING", "jobId": "j1",
  "orderId": 12, "deviceCode": "cart-1", "slug": "biscuit",
  "box": "box1", "total": 2, "done": 0, "currentIndex": 1,
  "elapsedSec": 0.0, "results": [], "message": ""
}
```

### 오류

| 코드 | 상황 |
|---|---|
| `400` | 모르는 `slug`(예: `cola`), 모르는 `deviceCode`, `quantity` 가 1 미만이거나 3 초과 |
| `409` | 이미 다른 픽업이 진행 중 (팔이 하나뿐이다) |
| `500` | 로봇·정책 오류 |

```json
400 {"success": false, "status": "FAILED",
     "message": "지원하지 않는 상품입니다: 'cola' (가능: [...])"}
```

---

## `GET /pick/state`

진행 상태. **`done` 이 지금까지 적재함에 담은 개수다.**

```json
{
  "success": true, "status": "RUNNING", "busy": true,
  "jobId": "j1", "orderId": 12, "deviceCode": "cart-1",
  "slug": "biscuit", "box": "box1",
  "total": 2, "done": 1, "currentIndex": 2, "elapsedSec": 24.3,
  "results": [
    {"index": 1, "success": true, "grasped": true, "dest_ok": true,
     "seconds": 23.1, "grip_min": 52.8, "release_pan": -27.3, "reason": ""}
  ],
  "message": ""
}
```

| `status` | 뜻 |
|---|---|
| `IDLE` | 아직 아무 작업도 없음 |
| `RUNNING` | 진행 중 |
| `DONE` | `total` 개 전부 성공 |
| `FAILED` | 픽업 실패로 중단. `done` 까지는 담겼다 |
| `ABORTED` | 정지 요청으로 중단 |

`results[]` 는 픽업 1회마다 하나씩 쌓인다. `grip_min` 은 그리퍼가 닫힌
최소값으로, **51.0 이상이면 제대로 물린 것**이다(48.99=허공, 49.2~49.6=미끄러짐).

폴링 주기는 **0.5초면 충분하다.** 응답이 300바이트 남짓이라 부하는 무시할
수준이지만, 100Hz 같은 과도한 폴링은 30fps 제어 루프를 방해할 수 있다.

---

## `POST /pick/stop`

```json
{"mode": "afterCurrent"}   // 지금 집는 것만 마치고 정지 (권장)
{"mode": "immediate"}      // 즉시 정지 후 홈 자세로 복귀
```

```json
200 {"success": true, "status": "STOPPING", "mode": "afterCurrent",
     "message": "현재 픽업을 끝내고 정지합니다"}
```

진행 중인 작업이 없으면 `status: "IDLE"` 로 돌아온다(오류 아님).

`afterCurrent` 를 기본으로 쓰는 것이 좋다. `immediate` 는 팔이 물체를 든 채
멈출 수 있어 회수가 번거롭다(홈 복귀 시 그리퍼가 열리며 떨어뜨린다).

정지 후 `GET /pick/state` 는 `status: "ABORTED"` 와 그때까지의 `done` 을 준다.

---

## `GET /health`

관제의 `is_reachable()` 이 이것으로 판단한다. **어떤 경우에도 예외를 내지 않는다.**

```json
{"success": true, "status": "OK", "message": "",
 "robotConnected": true, "busy": false,
 "rig": {"meanShiftPx": 4.3, "maxShiftPx": 8.2, "matchRatio": 1.0, "ok": true}}
```

`rig` 는 진열대·카메라 배치가 기준에서 얼마나 틀어졌는지다. 어긋나면
`status: "DEGRADED"`, `success: false` 가 되고 픽업 정확도를 보장할 수 없다.

---

## `GET /products`

집을 수 있는 상품 목록. 관제 카탈로그와 대조할 때 쓴다.

```json
{"success": true, "status": "OK",
 "slugs": ["biscuit","cake","ice-cream","milk","roll-cake","sandwich"],
 "devices": ["cart-1","cart-2"], "shelfCapacity": 3, "message": ""}
```

## `POST /home`

팔을 홈 자세로 되돌린다(복구용). 작업 중이면 `409` — 먼저 `/pick/stop` 할 것.

## 화면

| | |
|---|---|
| `GET /view` | 탑뷰·손목 화면과 진행 상태 (브라우저용 페이지) |
| `GET /stream?cam=camera1&fps=10` | MJPEG. `<img src="...">` 로 바로 쓴다 |
| `GET /frame.jpg?cam=camera2` | 정지 화면 1장 |

`camera1` 이 탑뷰(YOLO 박스가 그려진, 정책이 실제로 보는 화면),
`camera2` 가 손목이다. 관리자 화면에 그대로 끼울 수 있다.

---

## 상품 어휘

관제 `slug` 를 그대로 보내면 된다. 내부 변환은 서버가 한다.

| 관제 `slug` | 웨이포인트 | 내부 클래스 | 정책 지시문 |
|---|---|---|---|
| `milk` | W3 | `milk` | **`milk carton`** |
| `biscuit` | W4 | `biscuit` | `biscuit` |
| `ice-cream` | W5 | `icecream` | `icecream` |
| `roll-cake` | W2 | `roll` | `roll` |
| `cake` | W1 | `cake` | `cake` |
| `sandwich` | W6 | `sandwich` | `sandwich` |

`cola` 는 **지원하지 않는다.** 검출기가 오검출 때문에 `coke` 클래스를
제외했고 정책도 학습한 적이 없다. 카탈로그에서 `cola` 를 빼고 `biscuit` 을
넣기로 합의했다(2026-08-20). 그래도 요청이 오면 `400` 이다.

카트 매핑: `cart-1 → box1`(로봇과 가까운 적재함), `cart-2 → box2`.

---

## 반드시 알아야 할 제약 세 가지

**① 한 번에 3개까지.** 진열대 한 칸에 상품이 3개뿐이다. 주문 수량이 4 이상이면
관제가 나눠 보내거나, 사이에 사람이 리필해야 한다.

**② 한 개라도 실패하면 작업 전체를 중단한다.** 정책이 로봇과 가까운 것부터
집는 순서를 어기면 진열 상태가 학습 데이터에 없는 형태가 되고, 그 뒤 픽업은
빈 칸을 헛집는다. 실패를 안고 계속 가면 남은 시도까지 버린다.
`status:"FAILED"` 와 함께 그때까지의 `done` 을 돌려주므로, 관제는 몇 개가
담겼는지 알 수 있다.

**③ 팔은 하나다.** 두 번째 요청은 `409`. 카트 두 대가 같은 매대에 몰리면
관제가 순서를 조율해야 한다.

---

## curl 로 확인하기

```bash
# 도달 확인
curl -s localhost:8080/health | python3 -m json.tool

# 픽업 시작
curl -s -X POST localhost:8080/pick -H 'Content-Type: application/json' \
     -d '{"orderId":1,"deviceCode":"cart-1","slug":"biscuit","quantity":2}'

# 진행 상태
watch -n0.5 'curl -s localhost:8080/pick/state | python3 -m json.tool'

# 정지
curl -s -X POST localhost:8080/pick/stop -H 'Content-Type: application/json' \
     -d '{"mode":"afterCurrent"}'
```

서버 없이 클라이언트만 시험하려면 `scripts/` 의 테스트가 가짜 팔로
전 경로를 돌린다.

---

## 성능 참고

12개 조건 63 에피소드 기준선(2026-08-20): 전체 성공률 **81.0%**
(95% 신뢰구간 69.6–88.8), 픽업 1회 평균 **26.8초**.

조건당 표본이 5개라 조건 사이의 차이는 통계적으로 구분되지 않는다.
타임아웃을 잡을 때는 **픽업 1회당 90초**를 상한으로 보면 된다.
