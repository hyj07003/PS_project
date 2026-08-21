# 관제 서버 — 포장(OMX 2번 팔) 연동

픽업 연동(`../controller_patch/`)과 짝을 이루는 문서다. 포장 팔을 관제에
붙이기 위해 `PS_project/server/apps/controller-server` 에 넣을 것들이다.

> **API 규격은 `omx_pack/server.py` 의 모듈 문서에 있다.** 이 문서는
> **적용 방법**만 다룬다.

```
omx_pack_adapter.py   → app/adapters.py 맨 아래에 추가
```

---

## 픽업과 무엇이 다른가

관제 입장에서 어댑터가 하나 느는 것뿐이지만, **부르는 방식이 다르다.**

| | 픽업 (:8080) | 포장 (:8081) |
|---|---|---|
| 무엇을 | `slug` 로 지정 | **지정 불가** — 적재함에 있는 것을 담는다 |
| 몇 개 | `quantity` | **없음** — "적재함을 비워라" 가 단위 |
| 완료 판정 | 관제가 `done` 개수로 | **OMX 가 탑뷰로 적재함을 본다** (`boxEmpty`) |
| 트리거 | 카트가 매대 도킹 (`_omx_pick_at_shelf`) | 카트가 웨이포인트 `P` 도킹 |
| 팔 | 1번 팔 | **2번 팔 (별개 하드웨어)** |

**정책에 언어 조건이 없다.** 포장 정책(ACT)은 지시문 입력 자체가 없어서
"비스킷을 담아라" 를 전달할 방법이 없다. 어느 바구니에 담을지는
`deviceCode` 가 정한다(`cart-1`→노랑, `cart-2`→민트).

**팔이 다르므로 `_omx_arm_lock` 을 공유하면 안 된다.** 픽업과 포장은
동시에 돌 수 있다. 락을 공유하면 서로를 불필요하게 막는다.

---

## 1. 어댑터 등록

`app/adapters.py` 맨 아래에 `omx_pack_adapter.py` 내용을 붙인다.
`OmxHttpStationAdapter` 가 쓰는 것과 같은 모듈 전역(`os`, `json`, `time`,
`urllib`, `logger`, `Callable`, `Literal`, `Any`)을 그대로 쓰므로 추가
import 는 필요 없다.

`.env` 에 추가:

```
PACK_URL=http://192.168.129.50:8081
PACK_POLL_SEC=0.5
PACK_ATTEMPT_TIMEOUT_SEC=90
PACK_MAX_ATTEMPTS=3
PACK_CONNECT_TIMEOUT_SEC=5
```

> 포장 팔은 픽업 팔과 **같은 PC** 에 붙어 있다(둘 다 OMX PC). 포트만
> 다르다 — 픽업 8080, 포장 8081. `.env.example` 의 `OMX_URL` 과 호스트가
> 같고 포트만 바뀐다고 보면 된다.

---

## 2. `app/services/orders.py` — 4곳

### 수정 ①  import (파일 상단)

```python
from ..adapters import (
    ORDER_FLOW,
    MockAiAdapter,
    MockCartAdapter,
    MockStationAdapter,
    OmxHttpStationAdapter,
    OmxPackStationAdapter,      # ← 추가
    PinkyHttpCartAdapter,
    parse_omx_url,
    parse_omx_pack_url,         # ← 추가
    parse_pinky_robot_urls,
)
```

### 수정 ②  어댑터 선택 (`__init__`, 105행 부근)

픽업 어댑터를 고르는 바로 아래에 같은 모양으로 붙인다.

```python
        # 기존 (그대로 둔다)
        omx_url = parse_omx_url()
        if omx_url and os.environ.get("ADAPTER_MODE") != "mock":
            self.station_port = OmxHttpStationAdapter(omx_url)
        else:
            self.station_port = MockStationAdapter()

        # 추가 — 포장은 별개 팔이므로 포트를 따로 둔다
        pack_url = parse_omx_pack_url()
        if pack_url and os.environ.get("ADAPTER_MODE") != "mock":
            self.pack_port = OmxPackStationAdapter(pack_url)
        else:
            self.pack_port = None       # 포장 없이도 관제는 그대로 돈다
```

`self._omx_arm_lock` 옆(117행 부근)에 포장용 락을 하나 더 만든다:

```python
        self._omx_arm_lock = threading.Lock()
        self._omx_pack_lock = threading.Lock()      # ← 추가 (별개 팔)
```

### 수정 ③  포장 실행 메서드 추가

`_omx_pick_at_shelf` 옆에 둔다. 그 메서드와 같은 모양이다.

```python
    def _acquire_omx_pack(self, mission_id: int) -> None:
        """포장 팔을 기다린다. 픽업 팔과 별개다."""
        waited = False
        while True:
            self._ensure_not_aborted(mission_id)
            if self._omx_pack_lock.acquire(timeout=0.5):
                if waited:
                    self._mission_note(mission_id, "omx pack arm acquired")
                return
            if not waited:
                self._mission_note(mission_id, "omx pack arm wait")
                waited = True

    def _omx_pack_at_station(
        self,
        device_code: str,
        order_id: int,
        mission_id: int,
        waypoint_id: str,
    ) -> None:
        """포장 스테이션에서 적재함을 비운다. 포장 팔이 없으면 dwell 로 지나간다."""
        if self.pack_port is None:
            self._dwell_at(device_code, mission_id, waypoint_id)
            return

        self._acquire_omx_pack(mission_id)
        try:
            self._ensure_not_aborted(mission_id)
            self._set_waypoint(mission_id, waypoint_id, label_suffix="포장 중")
            self._mission_note(mission_id, f"omx pack start {device_code}")

            def _progress(attempt: int, total: int, box_empty) -> None:
                # box_empty 는 True/False/None 이고 None 은 "확인하지 못했다" 다.
                # 비지 않았다는 뜻이 아니므로 구분해서 남긴다.
                mark = ("비움" if box_empty is True
                        else "남음" if box_empty is False else "확인전")
                self._set_waypoint(
                    mission_id, waypoint_id,
                    label_suffix=f"포장 {attempt}/{total} {mark}")
                self._mission_note(
                    mission_id, f"omx pack {attempt}/{total} box={mark}")

            result = self.pack_port.pack(
                device_code=device_code,
                order_id=order_id,
                should_abort=lambda: self._is_aborted(mission_id),
                on_progress=_progress,
            )

            state = self.pack_port.last_state or {}
            box_empty = state.get("boxEmpty")
            if result == "DONE":
                self._mission_note(mission_id, "omx pack done (적재함 비움)")
                return

            # 실패·중단은 예외로 올린다. 바깥 try/except 가 FAILED 처리와
            # 복구를 이미 담당한다 — 픽업과 같은 방식이다.
            reason = self.pack_port.last_error or result
            if box_empty is None:
                # "모른다" 를 "안 비었다" 로 바꾸지 않는다. 사람이 봐야 하는
                # 상황이므로 메시지에 그대로 드러낸다.
                reason = f"적재함 상태를 확인하지 못했습니다 ({reason})"
            self._mission_note(mission_id, f"omx pack {result}: {reason}")
            raise RuntimeError(f"OMX 포장 {result}: {reason}")
        finally:
            self._omx_pack_lock.release()
```

### 수정 ④  트리거 연결 (`_leave_checkout_then_pack`, 1425행)

```python
            # 기존
            self._dock_dwell_undock(device_code, "P", mission_id)
```

```python
            # 변경 — 도킹 → 포장 → 후진
            travel = self._aruco_dock_or_fail(device_code, "P", mission_id)
            self._omx_pack_at_station(device_code, order_id, mission_id, "P")
            self._undock_after_shelf_aruco(device_code, "P", travel, mission_id)
```

> **`_dock_dwell_undock` 자체를 고치면 안 된다.** 그 함수는 `P`(포장)와
> `C`(계산대) 양쪽에서 쓰인다. `C` 는 3초 dwell 그대로여야 한다.

`_leave_checkout_then_pack` 의 시그니처에 `order_id` 가 없다면 호출부
(1642행 부근)에서 함께 넘겨야 한다.

---

## 알아둘 제약 세 가지

**① 팔은 하나다.** 두 번째 `/pack` 요청은 `409` 다. 카트 두 대가 포장
스테이션에 몰리면 관제가 순서를 조율해야 한다. `_acquire_omx_pack` 락이
그 역할을 한다.

**② 한 번에 다 못 옮길 수 있다.** 2026-08-21 실측에서 물건 3개씩 3회를
돌렸더니 3 / 1 / 1 개만 옮겼다(5/9). 그래서 `maxAttempts`(기본 3) 만큼
다시 시도하고, 그래도 안 비면 `FAILED` 로 알린다. 성능은 물건 위치에
민감하다.

**③ `boxEmpty: null` 은 실패가 아니다.** 팔이 적재함을 가려 확인하지
못했다는 뜻이다. OMX 는 최대 6초 동안 가림이 걷히길 기다린 뒤 그래도
못 보면 `null` 을 돌려준다. **"모른다" 를 "안 비었다" 로 바꾸지 말 것** —
사람이 봐야 하는 상황이다.

---

## 확인 방법

포장 팔 없이 (기존과 동일해야 한다):

```bash
# .env 에서 PACK_URL 을 비우거나 ADAPTER_MODE=mock
# → P 에서 3초 dwell 하고 지나간다
```

포장 팔 연결 후:

```bash
# OMX PC 에서
PYTHONPATH=~/il_ws/src ~/venv/pack/bin/python -m omx_pack.server \
    --basket yellow --strict-start \
    --robot-port /dev/omx_pack_follower \
    --front /dev/omx_cam_pack_top --wrist /dev/omx_cam_pack_hand \
    --finish box-empty --box cart-1 --port 8081

# 관제 PC 에서 도달 확인
curl -s <PACK_URL>/health   | python3 -m json.tool
curl -s <PACK_URL>/baskets  | python3 -m json.tool

# 주문 하나 넣고 미션 이벤트를 본다
sqlite3 data/smartshop.db \
  "SELECT note, created_at FROM mission_events ORDER BY id DESC LIMIT 20;"
# omx pack start cart-1
# omx pack 1/3 box=확인전
# omx pack 1/3 box=비움
# omx pack done (적재함 비움)
```

어댑터만 따로 시험하려면 관제 없이도 된다 — 포장 서버를 `--mock` 으로
띄우고 어댑터를 직접 부르면 된다(2026-08-21 이 방식으로 검증했다).
