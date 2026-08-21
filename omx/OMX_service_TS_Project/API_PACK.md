# OMX 포장 서버 API

포장 스테이션(OMX 2번 팔, ACT 정책)이 관제서버에 제공하는 HTTP 규격이다.
픽업 서버([`API.md`](API.md), :8080)와 **같은 규약**이고 포트와 경로만 다르다.

```
관제 :4100 ──┬─> 픽업 :8080   POST /pick   무엇을 몇 개 집을지 지정
             └─> 포장 :8081   POST /pack   적재함을 비운다
```

규약은 pinky 어댑터(`PinkyHttpCartAdapter`)에서 그대로 가져왔다.

- POST 는 JSON 본문, 필드는 **camelCase**
- 응답은 `{"success": bool, "status": ..., "message": str}`
- `GET /health` 로 도달 가능 여부를 판단한다 (`is_reachable`)
- 진행 상태는 **폴링**으로 읽는다
- 인증 없음

바로 붙일 수 있는 어댑터 구현이 [`scripts/controller_patch_pack/`](scripts/controller_patch_pack/)
에 있다. 이 문서는 규격, 그쪽은 적용 방법이다.

---

## 픽업과 다른 점 — 먼저 읽을 것

**① 무엇을 담을지 지정할 수 없다.**
포장 정책(ACT)에는 언어 조건 입력이 없다. 픽업의 `slug` 에 해당하는 것이
없고, 적재함에 들어 있는 것을 담을 뿐이다.

**② 개수를 지정하지 않는다.**
작업의 단위는 **"이 적재함을 비워라"** 다. `quantity` 가 없다. 완료 판정은
서버가 탑뷰 카메라로 적재함을 보고 한다.

**③ 완료 여부는 `boxEmpty` 하나로 본다.**
`done`/`total` 같은 개수 필드가 없다. 관제가 볼 것은 `boxEmpty` 다.

**④ 팔이 다르다.**
픽업 팔(:8080)과 포장 팔(:8081)은 별개 하드웨어다. 두 작업은 동시에 돌 수
있으므로 같은 락으로 묶지 말 것.

---

## 흐름

```
관제                                    포장 서버(:8081)
 │
 ├─ POST /pack {orderId,deviceCode} ──▶  202 즉시 반환, 워커 스레드 시작
 │                                       │
 │◀─ GET /pack/state (0.5초 주기) ─────┤  status=RUNNING, boxEmpty=null
 │                                       │  에피소드 실행 (최대 timeoutSec)
 │                                       │  ↓ 끝나면 탑뷰로 적재함 확인
 │                                       │  비었으면 DONE
 │                                       │  남았으면 다시 시도 (maxAttempts 까지)
 │◀─ GET /pack/state ──────────────────┤  status=DONE, boxEmpty=true
 │
 └─ (필요시) POST /pack/stop ──────────▶  afterCurrent | immediate
```

블로킹으로 두지 않은 이유는 픽업과 같다. 한 시도가 최대 90초이고 재시도가
3회면 4분이 넘는다. 그동안 관제가 상태도 못 보고 인터럽트도 못 걸면 곤란하다.

---

## `POST /pack`

적재함을 비우는 작업을 시작하고 **즉시 반환**한다.

### 요청

```json
{
  "orderId": 12,
  "deviceCode": "cart-1",
  "maxAttempts": 3,
  "timeoutSec": 90
}
```

| 필드 | 필수 | 기본 | 설명 |
|---|---|---|---|
| `deviceCode` | **예** | — | `cart-1` 또는 `cart-2`. 어느 적재함/바구니인지 정한다 |
| `orderId` | 아니오 | `0` | 로그와 응답에 그대로 실려 돌아온다 |
| `maxAttempts` | 아니오 | `3` | 적재함이 안 비었을 때 다시 시도할 횟수 |
| `timeoutSec` | 아니오 | `90` | **한 시도**의 상한 |

`quantity` 를 보내도 거절하지 않는다 — 재시도 횟수로 읽고 로그를 남긴다.
옛 클라이언트가 깨지지 않게 한 것이지 권장하는 사용법은 아니다.

### 응답 — `202 Accepted`

```json
{
  "success": true, "status": "RUNNING", "busy": true,
  "jobId": "p1", "orderId": 12,
  "deviceCode": "cart-1", "basket": "yellow",
  "boxEmpty": null, "attempt": 1, "maxAttempts": 3,
  "elapsedSec": 0.0, "results": [], "message": ""
}
```

### 오류

| 코드 | 언제 | `message` 예 |
|---|---|---|
| `400` | 모르는 `deviceCode` | `모르는 deviceCode 입니다: 'cart-9' (아는 것: cart-1, cart-2)` |
| `400` | `maxAttempts` < 1 | `maxAttempts 는 1 이상이어야 합니다: 0` |
| `400` | JSON 파싱 실패 | `JSON 파싱 실패: ...` |
| `400` | 시작 자세가 학습 범위 밖 (`--strict-start` 일 때만) | `시작 자세가 학습 범위 밖입니다: gripper 63.03 는 ...` |
| `409` | 이미 작업 중 | `이미 포장 작업을 처리 중입니다. 팔이 하나뿐입니다.` |
| `409` | 서버가 올린 것과 다른 바구니 | `이 서버는 yellow 바구니 모델을 올려 두었습니다. cart-2(mint) 요청은 처리할 수 없습니다.` |
| `500` | 그 밖의 오류 | 예외 메시지 |

`409` 응답에는 `jobId` 또는 `loaded`/`requested` 가 함께 실린다.

> **바구니 불일치를 왜 막는가** — 서버는 기동할 때 바구니 하나의 체크포인트만
> 올린다(`--basket`). 다른 바구니 요청을 조용히 처리하면 팔이 엉뚱한 자리로
> 간다. 예외도 경고도 없이 잘못 움직이는 것이 가장 나쁘다.

---

## `GET /pack/state`

관제가 폴링하는 진행 상태. `/status` 도 같은 응답을 준다.

```json
{
  "success": true, "status": "DONE", "busy": false,
  "jobId": "p1", "orderId": 12,
  "deviceCode": "cart-1", "basket": "yellow",
  "boxEmpty": true,
  "attempt": 1, "maxAttempts": 3,
  "elapsedSec": 63.9,
  "results": [
    {
      "index": 1, "basket": "yellow",
      "success": true, "finished": true, "judged": false,
      "seconds": 63.9, "aborted": false,
      "reason": "적재함이 비었습니다 (3/3 회 연속 비어 보임)",
      "boxEmpty": true, "boxLooks": 12,
      "trace": "/home/newuser/il_ws/traces/.../ep01.npz",
      "boxView": "/home/newuser/il_ws/traces/.../ep01_boxview.jpg"
    }
  ],
  "message": "적재함을 비웠습니다"
}
```

작업 전에는 `{"success": true, "status": "IDLE", "busy": false}` 만 온다.

| 필드 | 의미 |
|---|---|
| `status` | `IDLE` · `RUNNING` · `DONE` · `FAILED` · `ABORTED` |
| **`boxEmpty`** | **`true` 비움 · `false` 남음 · `null` 확인하지 못함** |
| `attempt` / `maxAttempts` | 몇 번째 시도인지 |
| `results[]` | 시도별 기록. `reason` 에 왜 끝났는지 |
| `boxLooks` | 적재함을 실제로 들여다본 횟수. `0` 이면 한 번도 못 봤다는 뜻 |
| `boxView` | 완료를 판정한 순간의 화면 (경로). 판정이 의심스러울 때 볼 것 |
| `stopRequested` | 정지 요청이 걸려 있으면 그 모드 |

### ⚠ `boxEmpty: null` 을 `false` 로 바꾸지 말 것

`null` 은 **"확인하지 못했다"** 이지 "안 비었다"가 아니다. 팔이 적재함을
가려서 볼 수 없었다는 뜻이다. 서버는 최대 6초 동안 가림이 걷히길 기다린 뒤
그래도 못 보면 `null` 을 준다.

둘은 관제가 다르게 다뤄야 한다 — `false` 는 다시 시도할 일이고, `null` 은
사람이 봐야 할 일이다.

---

## `POST /pack/stop`

```json
{"mode": "afterCurrent"}
```

| 모드 | 동작 |
|---|---|
| `afterCurrent` | 지금 시도를 끝내고 정지 |
| `immediate` | 즉시 정지 |

응답 `200`:

```json
{"success": true, "status": "STOPPING", "mode": "afterCurrent",
 "message": "현재 담기를 끝내고 정지합니다"}
```

진행 중인 작업이 없으면 `{"success": true, "status": "IDLE"}` 이다.
모르는 모드는 `400`.

정지된 작업은 `status: "ABORTED"` 로 끝난다. **`FAILED` 와 구분된다** —
운영자가 멈춘 것과 작업이 실패한 것은 다른 일이다.

---

## `GET /health`

관제의 `is_reachable()` 이 이것으로 판단한다. **어떤 경우에도 예외를 밖으로
내지 않는다.**

```json
{
  "success": true, "status": "OK", "message": "",
  "robotConnected": true, "busy": false,
  "basket": "yellow", "finishMode": "box-empty", "observeOnly": false,
  "startPose": {
    "grade": "OK", "out": [], "edge": [],
    "messages": [],
    "state": [-5.01, -65.76, 54.29, 51.99, -0.46, 53.24]
  },
  "rig": {"error": "포장 리그 기준값이 아직 없습니다"}
}
```

`startPose.grade` 가 `OUT` 이면 `status` 가 `DEGRADED` 가 되고 `success` 가
`false` 다 — 지금 팔 자세가 정책이 학습한 적 없는 상태라는 뜻이다.

---

## `GET /baskets`

```json
{
  "success": true, "status": "OK",
  "devices": {"cart-1": "yellow", "cart-2": "mint"},
  "baskets": ["mint", "yellow"],
  "boxCapacity": 3,
  "loaded": "yellow",
  "message": ""
}
```

`loaded` 가 지금 올려둔 바구니 모델이다. 관제는 이걸로 어느 요청을 보낼 수
있는지 미리 알 수 있다.

---

## `POST /home`

홈 자세 복귀 (복구용). 본문 없음.

**현재 `501` 을 반환한다.** 포장 팔의 홈 자세가 아직 측정되지 않았다.
픽업 값을 그대로 쓰면 다른 팔·다른 자리라 엉뚱한 데로 간다. 작업 중이면
`409` 다.

---

## 화면

| 경로 | 내용 |
|---|---|
| `GET /view` | 탑뷰·손목 두 화면과 진행 상태 (브라우저용) |
| `GET /stream?cam=front&fps=12` | MJPEG 스트림. `cam` 은 `front` 또는 `wrist` |
| `GET /frame.jpg?cam=front` | 한 장. 프레임을 못 얻으면 `503` |

`front` 가 탑뷰(적재함·바구니), `wrist` 가 손목이다. **픽업과 달리 YOLO 주석을
그리지 않는다** — ACT 는 원본 프레임으로 학습했다.

---

## 반드시 알아야 할 제약

**① 팔은 하나다.** 두 번째 요청은 `409` 다. 카트 두 대가 포장 스테이션에
몰리면 관제가 순서를 조율해야 한다.

**② 한 번에 다 못 옮길 수 있다.** 2026-08-21 실측에서 물건 3개씩 3회를 돌려
3 / 1 / 1 개를 옮겼다(5/9). 성능이 **물건 위치에 민감하다.** `maxAttempts`
재시도가 이것을 흡수하는 장치다.

**③ 정책은 스스로 멈추지 않는다.** 적재함을 다 비워도 빈 공간과 상자
테두리를 계속 집으려 든다. 그래서 서버가 탑뷰로 보고 끊는다. 이 판정이
없으면 `timeoutSec` 까지 돈다.

**④ 서버는 바구니 하나만 올린다.** `cart-1`(노랑)과 `cart-2`(민트)를 모두
받으려면 서버를 두 개 띄우거나(포트 분리), 요청 사이에 재기동해야 한다.
체크포인트가 각각 0.28 GB 라 동시 상주 자체는 가능하지만 아직 구현되지
않았다.

---

## curl 로 확인하기

```bash
# 도달 확인
curl -s localhost:8081/health   | python3 -m json.tool
curl -s localhost:8081/baskets  | python3 -m json.tool

# 포장 시작
curl -s -X POST localhost:8081/pack -H 'Content-Type: application/json' \
     -d '{"orderId":1,"deviceCode":"cart-1","maxAttempts":3}'

# 진행 상태
watch -n0.5 "curl -s localhost:8081/pack/state | python3 -m json.tool"

# 정지
curl -s -X POST localhost:8081/pack/stop -H 'Content-Type: application/json' \
     -d '{"mode":"afterCurrent"}'
```

**하드웨어 없이 시험**하려면 서버를 `--mock` 으로 띄우면 된다. HTTP·작업
모델·인터럽트가 전부 동작하고 화면에는 `MOCK - NO CAMERA` 가 나온다.

```bash
PYTHONPATH=~/il_ws/src ~/venv/pack/bin/python -m omx_pack.server \
    --mock --mock-episode-sec 4 --port 8081
```

실기 기동:

```bash
PYTHONPATH=~/il_ws/src ~/venv/pack/bin/python -m omx_pack.server \
    --basket yellow --strict-start \
    --robot-port /dev/omx_pack_follower \
    --front /dev/omx_cam_pack_top --wrist /dev/omx_cam_pack_hand \
    --finish box-empty --box cart-1 --port 8081
```

---

## 성능 참고 (2026-08-21)

| | |
|---|---|
| 제어 주기 | 30 fps 유지 (1,800프레임 중 1회 놓침) |
| 한 시도 | 60초 안팎. 적재함이 비면 그 전에 끊는다 |
| 빈 적재함 판정 | 7.6초 (최소 대기 5초 포함) |
| 옮김 성공률 | **5/9 (56%)** — 물건 위치에 민감 |

픽업(81%)보다 불안정하다. 데이터 보강 전까지는 `maxAttempts` 를 넉넉히
두고, `FAILED` 가 오면 사람이 확인하는 편이 안전하다.
