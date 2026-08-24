# 관제 서버(controller-server) 쪽 변경분

OMX 로봇팔을 픽업 스테이션으로 붙이기 위해 `PS_project/server` 에 넣어야 할
것들이다. 여기 있는 파일은 원본이고, 실제 적용은 controller-server 쪽에서 한다.

> **📖 API 규격은 [`../../API.md`](../../API.md) 에 있다.**
> 어떤 요청을 어떤 형식으로 보내야 하는지, 응답이 어떻게 생겼는지,
> 오류 코드와 제약이 전부 거기 있다. 이 문서는 **적용 방법**만 다룬다.

```
omx_adapter.py   → app/adapters.py 맨 아래에 추가
```

---

## 1. 상품 카탈로그 — `cola` 제거, `biscuit` 추가

**왜** — OMX 검출기의 `KEEP_CLASSES` 에서 `coke` 를 오검출 때문에 의도적으로
제외했고, 정책도 콜라를 학습한 적이 없다. 반대로 `biscuit` 은 OMX 가 가장
잘 집는 상품인데(롤아웃 10회 중 9회 성공) 카탈로그에 없었다. 그래서
`app/seed.py` 의 `PRODUCTS` 에서 `cola` 를 빼고 `biscuit` 을 넣는다.

**어느 상품이 어느 웨이포인트에 있는지는 OMX 소관이 아니다.** 그 표는
카트가 어느 매대로 갈지를 정하는 것이고 관제/카트 담당이 정한다. OMX 는
화면에서 상품을 찾아 집으므로 매대 번호와 무관하다. 아래 값은 2026-08-21
기준 관제의 현재 매핑을 옮겨 적은 것이니, 바뀌었으면 관제 쪽을 따를 것.

- W1=cake, W2=roll-cake, W3=milk, W4=biscuit, W5=ice-cream, W6=sandwich

```python
# 빼기
{
    "code": "beverage",
    "name": "콜라",
    "slug": "cola",
    "description": "매장 데모 — 콜라 매대(W6).",
    "price": 2000,
    "featured": 1,
},

# 넣기
{
    "code": "snack",
    "name": "비스킷",
    "slug": "biscuit",
    "description": "매장 데모 — 비스킷 매대(W6).",
    "price": 3000,
    "featured": 1,
},
```

`app/waypoints.py` 의 `SLUG_TO_WAYPOINT` 도 같이 고친다:

```python
SLUG_TO_WAYPOINT: dict[str, str] = {
    "cake": "W1",
    "roll-cake": "W2",
    "milk": "W3",
    "biscuit": "W4",
    "ice-cream": "W5",
    "sandwich": "W6",
}
```

> 기존 DB 가 있으면 `ensure_demo_catalog()` 가 `slug NOT IN (...)` 로
> 비활성화하므로 자동 정리된다.

OMX 가 실제로 받는 slug 목록은 `GET /products` 로 확인할 수 있다.
관제에만 있고 OMX 에 없는 상품이 오면 `400` 으로 거절된다.

---

## 2. 어댑터 등록

`app/adapters.py` 맨 아래에 `omx_adapter.py` 내용을 붙인 뒤,
`app/services/orders.py` 에서 교체한다.

```python
# 기존
self.station_port = MockStationAdapter()

# 변경
from ..adapters import OmxHttpStationAdapter, parse_omx_url
self.station_port = (
    OmxHttpStationAdapter()
    if (os.environ.get("ADAPTER_MODE") != "mock" and parse_omx_url())
    else MockStationAdapter()
)
```

`.env` 에 추가:

```
OMX_URL=http://192.168.129.50:8080   # OMX(로봇팔) PC 의 LAN IP
OMX_POLL_SEC=0.5
OMX_CONNECT_TIMEOUT_SEC=5
```

**OMX 는 관제와 다른 PC 에서 돈다.** 로봇팔이 붙어 있는 PC 의 LAN IP 를
적는다. 그 PC 에서 `scripts/start_server.sh` 로 서버를 띄우면 기동 로그에
`관제 PC server/.env 예: OMX_URL=http://<IP>:8080` 이 찍히므로 그 값을
그대로 옮기면 된다. 방화벽에서 TCP 8080 을 열어야 한다.

한 PC 에서 다 돌리는 경우에는 `http://127.0.0.1:8080` 이다. 관제 4100,
pinky 4200, OMX 8080 이라 포트는 충돌하지 않는다.

---

## 3. 픽업 투어에 연결 — `app/services/orders.py`

가장 손이 가는 부분이다. 값 교체가 아니라 구조를 손대야 한다.

### 왜 단순 치환이 안 되나

지금 픽업 투어는 **웨이포인트 단위**로 돈다. `waypoint_ids_for_slugs()` 가
slug 들을 웨이포인트 id 로 접어버리기 때문에, 매대에 도착한 시점에는
**무엇을 몇 개 집어야 하는지가 사라져 있다.**

```python
slugs = [r["slug"] for r in rows if r["slug"]]      # ["sandwich", "milk"]
shelf_ids = waypoint_ids_for_slugs(slugs)           # ["W3", "W5"]  ← 정보 소실
tour = nearest_neighbor_order(start, shelf_ids)
```

`pick(device_code, slug, quantity, ...)` 를 부르려면 `slug` 와 `quantity` 가
필요하므로, **웨이포인트 → slug 역매핑**과 **slug → 수량** 표를 함께 들고 가야
한다. 웨이포인트 순회 순서는 그대로 두고 정보만 덧붙이는 방식이다.

---

### 수정 ①  import 추가 (파일 상단)

```python
from ..adapters import (
    ORDER_FLOW,
    MockAiAdapter,
    MockCartAdapter,
    MockStationAdapter,
    OmxHttpStationAdapter,      # ← 추가
    PinkyHttpCartAdapter,
    parse_omx_url,              # ← 추가
    parse_pinky_robot_urls,
)
from ..waypoints import (
    WAYPOINTS,
    SLUG_TO_WAYPOINT,           # ← 추가
    aruco_marker_id_for_waypoint,
    # (나머지는 그대로 둔다)
)
```

> `omx_adapter.py` 를 `app/omx_adapter.py` 로 따로 뒀다면
> `from ..omx_adapter import OmxHttpStationAdapter, parse_omx_url` 로 쓴다.

### 수정 ②  어댑터 선택 (`__init__`, 55행 부근)

```python
# 기존
self.station_port = MockStationAdapter()

# 변경 — pinky 어댑터를 고르는 방식과 같은 모양으로
omx_url = parse_omx_url()
if omx_url and os.environ.get("ADAPTER_MODE") != "mock":
    self.station_port = OmxHttpStationAdapter(omx_url)
else:
    self.station_port = MockStationAdapter()
```

`OMX_URL` 이 없거나 `ADAPTER_MODE=mock` 이면 지금처럼 Mock 으로 돈다.
**OMX 없이도 관제를 그대로 돌릴 수 있어야 개발이 편하다.**

### 수정 ③  쿼리에 수량 추가 (940행 부근)

```python
# 기존
rows = self.conn.execute(
    """
    SELECT oi.product_id, p.slug
    FROM order_items oi
    JOIN products p ON p.id = oi.product_id
    WHERE oi.order_id = ?
    """,
    (order_id,),
).fetchall()
slugs = [r["slug"] for r in rows if r["slug"]]
shelf_ids = waypoint_ids_for_slugs(slugs)
tour = nearest_neighbor_order(start, shelf_ids)
```

```python
# 변경 — quantity 를 함께 뽑고, 웨이포인트에서 slug 를 되찾을 표를 만든다
rows = self.conn.execute(
    """
    SELECT oi.product_id, p.slug, oi.quantity
    FROM order_items oi
    JOIN products p ON p.id = oi.product_id
    WHERE oi.order_id = ?
    """,
    (order_id,),
).fetchall()
slugs = [r["slug"] for r in rows if r["slug"]]
shelf_ids = waypoint_ids_for_slugs(slugs)
tour = nearest_neighbor_order(start, shelf_ids)

# 같은 매대에 여러 품목이 오면 수량을 더한다 (지금 카탈로그는 매대당 1품목이라
# 실제로는 일어나지 않지만, 나중에 늘어나도 깨지지 않게 해 둔다)
qty_by_slug: dict[str, int] = {}
wp_to_slugs: dict[str, list[str]] = {}
for r in rows:
    slug = r["slug"]
    if not slug:
        continue
    qty_by_slug[slug] = qty_by_slug.get(slug, 0) + int(r["quantity"])
    wid = SLUG_TO_WAYPOINT.get(slug)
    if wid:
        wp_to_slugs.setdefault(wid, [])
        if slug not in wp_to_slugs[wid]:
            wp_to_slugs[wid].append(slug)
```

### 수정 ④  투어 루프에서 픽업 호출 (952행 부근)

```python
# 기존
for wp in tour:
    self._ensure_not_aborted(mission_id)
    self._set_waypoint(mission_id, wp.id)
    self._nav_or_fail(device_code, wp.x, wp.y, wp.yaw, wp.id, mission_id=mission_id)
    self._aruco_dock_or_fail(device_code, wp.id, mission_id)
    self._dwell_at(device_code, mission_id, wp.id)
```

```python
# 변경
for wp in tour:
    self._ensure_not_aborted(mission_id)
    self._set_waypoint(mission_id, wp.id)
    self._nav_or_fail(device_code, wp.x, wp.y, wp.yaw, wp.id, mission_id=mission_id)
    self._aruco_dock_or_fail(device_code, wp.id, mission_id)
    self._pick_at(device_code, order_id, mission_id, wp.id, wp_to_slugs, qty_by_slug)
```

그리고 `_dwell_at` 옆에 이 메서드를 추가한다:

```python
    def _pick_at(
        self,
        device_code: str,
        order_id: int,
        mission_id: int,
        waypoint_id: str,
        wp_to_slugs: dict[str, list[str]],
        qty_by_slug: dict[str, int],
    ) -> None:
        """매대에 도착한 뒤 OMX 로봇팔에 픽업을 시킨다.

        OMX 가 붙어 있지 않으면(Mock) 기존처럼 dwell 만 하고 지나간다.
        그래야 로봇팔 없이도 관제 전체 흐름을 시험할 수 있다.
        """
        slugs = wp_to_slugs.get(waypoint_id) or []
        if not slugs or not isinstance(self.station_port, OmxHttpStationAdapter):
            self._dwell_at(device_code, mission_id, waypoint_id)
            return

        for slug in slugs:
            qty = qty_by_slug.get(slug, 1)
            self._ensure_not_aborted(mission_id)
            self._set_waypoint(
                mission_id, waypoint_id, label_suffix=f"{slug} 0/{qty}"
            )
            self._mission_note(mission_id, f"omx pick start {slug} x{qty}")

            result = self.station_port.pick(
                device_code,
                slug,
                qty,
                order_id,            # OMX 로그·응답에 그대로 실려 돌아온다
                # 운영자 정지를 OMX 까지 그대로 전파한다.
                # 어댑터가 알아서 POST /pick/stop {"mode":"afterCurrent"} 를 보낸다.
                should_abort=lambda: self._is_aborted(mission_id),
                # 진행 개수를 미션 이벤트와 웨이포인트 라벨에 흘린다.
                # 관리자 화면에서 "sandwich 1/2" 처럼 보인다.
                on_progress=lambda done, total: (
                    self._set_waypoint(
                        mission_id, waypoint_id, label_suffix=f"{slug} {done}/{total}"
                    ),
                    self._mission_note(mission_id, f"omx pick {slug} {done}/{total}"),
                ),
            )

            done = int(self.station_port.last_state.get("done") or 0)
            if result == "DONE":
                self._mission_note(mission_id, f"omx pick done {slug} {done}/{qty}")
                continue

            # 실패·중단은 여기서 예외로 올린다. 바깥 try/except 가
            # FAILED 처리와 홈 복귀를 이미 담당하고 있다.
            reason = self.station_port.last_error or result
            self._mission_note(
                mission_id, f"omx pick {result} {slug} {done}/{qty}: {reason}"
            )
            raise RuntimeError(f"OMX 픽업 {result} ({slug} x{qty}, {done}개 담김): {reason}")
```

---

### 이렇게 짠 이유

**`_dwell_at` 을 지우지 않았다.** OMX 가 없으면 그대로 쓴다. `ADAPTER_MODE=mock`
이나 `OMX_URL` 미설정 상태에서 관제만 돌려 볼 수 있어야 한다.

**예외로 올려서 바깥에 맡긴다.** `_run_mission` 의 `try/except` 가 이미
`FAILED` 상태 기록·홈 복귀·디바이스 반납을 처리한다. `_pick_at` 안에서
따로 처리하면 그 흐름과 어긋난다.

**`should_abort` 에 기존 판단을 그대로 넘긴다.** 운영자가 관제에서 정지를
누르면 `_is_aborted(mission_id)` 가 True 가 되고, 어댑터가 그것을 보고
OMX 에 정지를 요청한다. **별도 배선이 필요 없다.**

**`on_progress` 로 진행 개수를 남긴다.** 웨이포인트 라벨(`sandwich 1/2`)과
미션 이벤트 양쪽에 흘리면, 관리자 화면에서 몇 개까지 담겼는지 실시간으로
보인다. 실패했을 때 `done` 이 응답에 들어 있으므로 "2개 중 1개는 담겼다" 를
사용자에게 알릴 수 있다.

**같은 매대의 여러 품목을 리스트로 다뤘다.** 지금 카탈로그는 매대당 1품목이라
불필요해 보이지만, `SLUG_TO_WAYPOINT` 에 품목이 추가되면 조용히 하나만
집고 넘어가는 버그가 된다.

---

### 확인 방법

OMX 없이 (Mock 유지):

```bash
# .env 에서 OMX_URL 을 비우거나 ADAPTER_MODE=mock
# → 기존과 똑같이 3초 dwell 하고 지나가야 한다
```

OMX 연결 후:

```bash
# 주문 하나 넣고 미션 이벤트를 본다
sqlite3 data/smartshop.db \
  "SELECT note, created_at FROM mission_events ORDER BY id DESC LIMIT 20;"
# omx pick start biscuit x2
# omx pick biscuit 1/2
# omx pick done biscuit 2/2
```

## 알아둘 제약 세 가지

**① 한 번에 3개까지.** 진열대 한 칸에 상품이 3개뿐이다. `quantity > 3` 은
`400` 으로 거절된다. 주문에 4개 이상이 들어올 수 있으면 사람이 리필하거나
주문 단계에서 막아야 한다.

**② 한 개라도 실패하면 즉시 중단한다.** 정책이 가까운 것부터 집는
순서(FIFO)를 어기면 진열 상태가 학습 데이터에 없는 형태가 되고, 그 뒤 픽업은
빈 칸을 헛집는다(2026-08-19 실측). 실패를 안고 계속 가면 남은 시도까지 버린다.
그래서 `status:"FAILED"` 와 함께 그때까지의 `done` 을 돌려준다.

**③ 팔은 하나다.** 두 번째 `/pick` 요청은 `409` 로 거절된다. 카트 두 대가
동시에 매대에 도착하면 관제가 순서를 조율해야 한다.

---

## 시험 순서

```bash
# 1) OMX 서버 띄우기 (OMX PC)
cd ~/il_ws/src
PYTHONPATH=$PWD YOLO_AUTOINSTALL=false ~/venv/il/bin/python -m omx_yolo.server \
    --policy ~/il_ws/src/lerobot/outputs/train/v1_yolo/checkpoints/last/pretrained_model \
    --port 8080

# 2) 도달 확인
curl -s localhost:8080/health | python3 -m json.tool
curl -s localhost:8080/products | python3 -m json.tool

# 3) 픽업 한 번
curl -s -X POST localhost:8080/pick -H 'Content-Type: application/json' \
     -d '{"orderId":1,"deviceCode":"cart-1","slug":"biscuit","quantity":1}'

# 4) 진행 상태 폴링
watch -n0.5 "curl -s localhost:8080/pick/state | python3 -m json.tool"

# 5) 정지
curl -s -X POST localhost:8080/pick/stop -H 'Content-Type: application/json' \
     -d '{"mode":"afterCurrent"}'
```

하드웨어 없이 HTTP·작업·인터럽트 로직만 검증하려면:

```bash
PYTHONPATH=~/il_ws/src python <스크래치패드>/test_server.py
```
