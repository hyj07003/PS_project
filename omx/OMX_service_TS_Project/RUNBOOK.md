# 통합 시험 절차

관제 + 카트 + 픽업 팔 + 포장 팔을 한 번에 돌릴 때의 순서다. OMX(팔 2대)
쪽에서 준비할 것만 다룬다. 관제·카트는 각 담당 절차를 따른다.

---

## 0. 사전 점검 — 먼저 돌린다

```bash
python3 ~/il_ws/scripts/preflight.py
```

장치 이름·카메라·모터 버스·환경·체크포인트·서버·자원을 한 번에 본다.
**종료 코드 0 이면 통과, 1 이면 경고, 2 이면 실패다.**

팔 자세까지 보려면 `--pose` 를 준다. 이 점검만 팔에 연결하므로 **토크가
켜진다** — 주변을 비우고 실행할 것. 나머지는 전부 읽기라 팔이 움직이지 않는다.

무엇을 왜 보는지는 이런 것들이다.

| 항목 | 놓치면 생기는 일 |
|---|---|
| 카메라 4대가 **서로 다른 장치**인가 | 같은 모델이 2대씩이라 이름이 뒤바뀔 수 있다. 픽업 서버가 포장 화면으로 추론해도 예외가 안 난다 |
| 모터 버스 응답 | ID 11 이 가끔 통째로 무응답이 된다. 서버 기동이 실패하거나 롤아웃 중에 끊긴다 |
| 시작 자세 | 학습 범위 밖에서 출발하면 정책이 본 적 없는 상태에서 자신 있게 움직인다 |
| venv/체크포인트 | 픽업 0.4.4, 포장 0.6.1. 섞이면 임포트부터 깨진다 |

---

## 1. 서버 두 개를 띄운다

**픽업** (터미널 1)

```bash
cd ~/il_ws/src
export PYTHONPATH=~/il_ws/src
export YOLO_AUTOINSTALL=false          # 제어 루프 중 pip install 방지
~/venv/il/bin/python -m omx_yolo.server \
    --policy ~/il_ws/src/lerobot/outputs/train/v1_yolo/checkpoints/060000/pretrained_model \
    --retries 2 --port 8080
```

`--retries` 는 **헛집었을 때만** 다시 시도한다(집었다 놓친 경우는 진열 상태를
알 수 없어 재시도하지 않는다). 한 번에 30초쯤 걸리므로 관제의
`OMX_PICK_TIMEOUT_SEC` 도 함께 늘려야 한다. 기본은 0 = 첫 실패에 중단.

또는 `POLICY=<체크포인트> ~/il_ws/scripts/start_server.sh`

**포장** (터미널 2)

```bash
PYTHONPATH=~/il_ws/src ~/venv/pack/bin/python -m omx_pack.server \
    --strict-start --home-after \
    --robot-port /dev/omx_pack_follower \
    --front /dev/omx_cam_pack_top --wrist /dev/omx_cam_pack_hand \
    --trace-dir ~/il_ws/traces/$(date +%m%d) \
    --finish box-empty --port 8081
```

바구니는 요청의 `deviceCode` 가 정한다 — 노랑·민트 모델을 둘 다 올리므로
`cart-1`·`cart-2` 를 한 서버로 처리한다.

`--home-after` 는 작업이 끝나면 팔을 대기 자세로 되돌린다. 홈 값이 없으면
경고만 남고 작업 결과는 그대로다. 처음 한 번은 기록해 두어야 한다 —
**서버를 내린 상태에서** 팔을 원하는 대기 자세에 두고:

```bash
PYTHONPATH=~/il_ws/src ~/venv/pack/bin/python -m omx_pack.home --capture
```

기동 순서는 상관없다. 관제가 먼저 떠 있어도 요청할 때마다 `/health` 를 본다.
다만 **정책 로드와 카메라 워밍업에 몇 초 걸리므로** 아래 줄이 찍힌 뒤에
주문을 넣는다.

```
INFO 로봇 연결 완료. 준비됨 (yellow).
INFO 서버 시작 http://0.0.0.0:8081  (바구니 yellow · 종료판정 box-empty · ...)
```

---

## 2. 관제에서 도달 확인

```bash
curl -s http://<OMX PC>:8080/health | python3 -m json.tool
curl -s http://<OMX PC>:8081/health | python3 -m json.tool
```

**`robotConnected: true` 여야 한다.** false 면 관제가 조용히 Mock 으로
빠진다 — 에러가 안 나므로 "로봇이 왜 안 움직이지" 가 된다.

관제 `.env`:

```
OMX_URL=http://<OMX PC LAN IP>:8080     # 픽업
PACK_URL=http://<OMX PC LAN IP>:8081    # 포장
```

방화벽에서 TCP 8080·8081 을 열어야 한다.

---

## 3. 시험 중에 볼 것

브라우저로 두 화면을 띄워 둔다.

```
http://<OMX PC>:8080/view    픽업 (탑뷰에 YOLO 박스 + 손목)
http://<OMX PC>:8081/view    포장 (탑뷰 + 손목, 시도 횟수와 적재함 상태)
```

로그에서 이런 줄이 나오면 정상이다.

```
INFO 적재함 확인 12.3초: 물건 3개 보임 (가림 12%)
INFO 적재함 확인 31.8초: 3/3 회 연속 비어 보임
INFO 궤적 기록: /home/newuser/il_ws/traces/0821/...npz
INFO 판정 근거 화면: /home/newuser/il_ws/traces/0821/..._boxview.jpg
```

---

## 4. 잘 안 될 때

| 증상 | 원인과 조치 |
|---|---|
| 서버 기동이 `Missing motor IDs: 11` 로 실패 | 모터 버스 간헐 무응답. 서버가 3회 재시도하지만 그래도 실패하면 팔 전원을 껐다 켠다. 반복되면 베이스 관절 커넥터를 다시 꽂는다 |
| `POST /pack` 이 400 · "시작 자세가 학습 범위 밖" | `--strict-start` 가 막은 것이다. 팔을 학습 범위 안 자세로 두거나, 데모 중이면 `--strict-start` 없이 띄우고 기동 로그의 자세 표만 확인한다 |
| `POST /pack` 이 409 · "yellow 바구니 모델을 올려 두었습니다" | 서버는 바구니 하나만 올린다. `cart-2` 를 쓰려면 `--basket mint --box cart-2` 로 다시 띄운다 |
| 작업 후 팔이 멈춘 자리에 그대로 섬 | 정책은 스스로 홈으로 가지 않는다. `--home-after` 를 주고, 홈 값이 없으면 `omx_pack.home --capture` 로 기록한다 |
| 서버가 `Port is in use` 로 죽음 | 다른 프로세스가 같은 팔을 잡고 있다. `preflight.py`·`omx_pack.home`·`omx_pack.dist` 는 서버를 내린 뒤에 쓸 것 |
| 포장이 안 끝나고 90초까지 감 | 적재함을 못 본 것이다. 로그에 `N초째 적재함을 보지 못했습니다` 가 있는지 본다. 팔이 계속 가리고 있으면 ROI 나 가림 기준을 다시 잡아야 한다 |
| `boxEmpty: null` 로 끝남 | 확인 실패이지 실패가 아니다. `_boxview.jpg` 를 열어 무엇을 보고 그랬는지 확인한다 |
| 픽업이 매대에서 3초만 대기하고 지나감 | 관제가 Mock 으로 빠진 것이다. `OMX_URL` 과 `/health` 를 확인한다 |
| 화면이 안 나옴 | 다른 프로세스가 카메라를 잡고 있다. 서버가 떠 있으면 `preflight.py --skip cameras` 로 돌린다 |

---

## 4-1. 관제 없이 단위 시험

`try.sh` 로 서버만 따로 돌려볼 수 있다. 요청을 보내고 끝날 때까지 폴링하며
진행이 바뀔 때만 한 줄씩 찍는다.

```bash
~/il_ws/scripts/try.sh health                # 두 서버 상태
~/il_ws/scripts/try.sh pick sandwich cart-1  # 픽업 1개
~/il_ws/scripts/try.sh pick biscuit cart-1 1 2   # 개수 1 · 재시도 2
~/il_ws/scripts/try.sh pack cart-1           # 포장 (적재함 비우기)
~/il_ws/scripts/try.sh pack cart-2 3         # 민트 바구니 · 재시도 3
~/il_ws/scripts/try.sh stop pick             # 정지
```

관제 PC 에서 돌릴 때는 주소를 넘긴다:

```bash
PICK_URL=http://<OMX PC>:8080 PACK_URL=http://<OMX PC>:8081 \
  ~/il_ws/scripts/try.sh health
```

## 5. 시험 뒤

궤적이 `--trace-dir` 에 쌓인다. 분석은:

```bash
# 종료 판정·자세
PYTHONPATH=~/il_ws/src ~/venv/pack/bin/python -m omx_pack.trace ~/il_ws/traces/0821

# 파지 시도와 타이밍 (담긴 개수는 세지 못한다 — 문서 참조)
PYTHONPATH=~/il_ws/src ~/venv/pack/bin/python -m omx_pack.trace ~/il_ws/traces/0821 --grasp
```

**바구니별로 디렉터리를 나눌 것.** 섞이면 분석기가 경고하지만 애초에
나누는 편이 낫다.

---

## 알아둘 제약

- **팔은 각각 하나씩이다.** 픽업·포장 모두 두 번째 요청은 `409` 다. 다만
  서로 다른 하드웨어라 **동시에 돌 수 있다** — 관제에서 같은 락으로 묶지 말 것.
- **포장 서버는 바구니 하나만 올린다.** `cart-2`(민트)를 쓰려면
  `--basket mint --box cart-2` 로 **다시 띄워야 한다.** 서버를 두 개 띄우는
  방법은 쓸 수 없다 — 팔이 하나뿐이라 같은 시리얼 포트를 두 프로세스가
  열 수 없다.
- **포장 성공률이 5/9(56%)이고 물건 위치에 민감하다.** `maxAttempts` 재시도가
  이를 흡수하지만, 데모에서는 적재함에 물건을 2~3개만 두는 편이 안전하다.
- **`POST /home` 은 포장에서 `501` 이다.** 포장 팔의 홈 자세를 아직 측정하지
  않았다.
