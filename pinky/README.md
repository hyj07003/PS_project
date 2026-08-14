# Pinky Robot Server

pinky_pro(ROS2 실기 배포본)의 배터리·라이다·IMU·초음파·LED·LCD 인터페이스를
래핑한 모듈과, SmartShop Controller(`:4100`)와 HTTP로 통신하는 Flask 서버입니다.

> 개발 PC에서는 기본 `PINKY_BACKEND=mock` 으로 동작합니다.  
> 로봇에서는 ROS2 Jazzy를 source 한 뒤 `PINKY_BACKEND=ros2` 로 실행하세요.

---

## 구조

```text
pinky/
├── modules/                 # 센서·액추에이터 모듈
│   ├── robot.py             # PinkyRobot 파사드
│   ├── battery.py / lidar.py / imu.py / ultrasonic.py
│   ├── led.py / lcd.py
│   └── backends/            # mock | ros2 (+ ros2_runtime)
├── controllers/             # ROS2 센서 publisher (run.py와 동시 기동)
│   ├── hardware.py          # pinkylib / I2C(ADC·BNO055) 읽기
│   └── sensor_publisher.py  # battery·imu·ultrasonic 토픽 발행
├── server/                  # Flask API
├── requirements.txt
├── pinky.env.example        # 로봇 업로드용 설정 템플릿 (→ pinky.env)
├── .env.example             # 로컬 개발용 (동일 내용)
└── run.py
```

> **로봇 배포 팁:** `.env`는 숨김 파일이라 SCP/파일관리자에서 업로드가 안 되는 경우가 많습니다.  
> `pinky.env.example` → `pinky.env`로 복사해 로봇 `pinky/` 폴더에 넣으면 됩니다. (`run.py`가 자동 로드)



## pinky_pro 매핑


| 모듈         | ROS2                                   | 비고                          |
| ---------- | -------------------------------------- | --------------------------- |
| Battery    | `/battery/percent`, `/battery/voltage` | bringup `battery_publisher` |
| Lidar      | `/scan`                                | `rplidarc1` (RPLidar C1 `/dev/ttyAMA0`) 발행 |
| IMU        | `/imu_raw`                             | BNO055 (별도 노드)              |
| Ultrasonic | `/us_sensor/range`, `/ir_sensor/range` | ADC 노드                      |
| LED        | `/set_led`, `/set_brightness`          | `pinky_led`                 |
| LCD        | `/set_emotion`                         | `pinky_emotion`             |
| Drive      | `/cmd_vel`                             | Dynamixel bringup           |


---



## 실행 (Linux)

```bash
cd ~/PS_project/pinky
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
# 로컬: .env 또는 로봇 전송용 pinky.env
cp pinky.env.example pinky.env   # 권장 (업로드 가능)
# cp .env.example .env           # 로컬 전용도 가능
.venv/bin/python run.py
```

기본 포트: **4200** → [http://127.0.0.1:4200/health](http://127.0.0.1:4200/health)

로봇(실기)에서 ROS 연동:

```bash
# ROS2 환경 source 후
export PINKY_BACKEND=ros2
export PINKY_SENSOR_PUBLISHER=auto   # run.py가 센서 publisher도 같이 기동
export CONTROLLER_URL=http://<PC_IP>:4100
.venv/bin/pip install -r requirements.txt   # smbus2 포함
.venv/bin/python run.py
```

`run.py` 기동 시 (`PINKY_BACKEND=ros2`):

1. `controllers/sensor_publisher.py` — 하드웨어에서 읽어 ROS 토픽 발행  
   (`battery/*`, `imu_raw`, `us_sensor/range`, `ir_sensor/range`, **`scan`**)
2. Flask + `Ros2Backend` — 위 토픽 구독 후 HTTP로 제공

라이다는 `controllers/lidar.py`가 **rplidarc1**로 `/dev/ttyAMA0`을 읽고,
`PINKY_DEFER_LIDAR` 시 pinky_pro `sllidar`의 `/scan`을 구독합니다.
`/scan`이 비어 있으면 기동 시 `lidar_recovery`가 sllidar를 정리하고 LidarReader로 폴백합니다.
로그: `~/pinky_logs/pinky_pro_bringup.log`
`output_queue` 원시 샘플을 **1회전 단위로 모아** 고밀도 포인트를 만듭니다.
(실패 시 pyserial → `sllidar_ros2` launch 순으로 폴백. 기존 sllidar와 동시 사용 시 포트 충돌 주의.)

맵 밀도 관련 env: `PINKY_LIDAR_BINS`(기본 2160), `PINKY_LIDAR_RAW_MAX`(4320), `PINKY_LIDAR_API_POINTS`(2880)

---



## Flask API



### 센서 / 액추에이터


| Method | Path                      | 설명                        |
| ------ | ------------------------- | ------------------------- |
| GET    | `/health`                 | 헬스                        |
| GET    | `/sensors`                | 전체 스냅샷                    |
| GET    | `/sensors/battery`        | 배터리                       |
| GET    | `/sensors/lidar`          | 라이다 (샘플 포함)               |
| GET    | `/sensors/imu`            | IMU                       |
| GET    | `/sensors/ultrasonic`     | 초음파·IR                    |
| POST   | `/actuators/led`          | `{command,r,g,b,pixels?}` |
| POST   | `/actuators/lcd`          | `{emotion}`               |
| GET    | `/actuators/lcd/emotions` | 표정 목록                     |
| POST   | `/cmd/drive`              | `{linearX, angularZ}`     |
| POST   | `/cmd/assign`             | Controller → 할당           |
| POST   | `/cmd/navigate`           | Controller → 웨이포인트        |
| GET    | `/map/meta`               | Occupancy 맵 메타 (yaml)      |
| GET    | `/map/image`              | 맵 PNG (`PINKY_MAP`)        |
| GET    | `/nav/state`              | pose · navigating · mapId · amclActive · localizationMode |
| GET    | `/nav/plan`               | Nav2 `/plan` 글로벌 경로 `{poses:[{x,y,yaw},...]}` |
| GET    | `/nav/path`               | `/nav/plan` 호환 래퍼 `{success,path:{frameId,count,poses}}` |
| POST   | `/nav/plan`               | ComputePathToPose — 이동 없이 경로만 `{x,y,yaw?,timeoutSec?,plannerId?}` |
| POST   | `/nav/initialpose`        | `{x,y,yaw}` map 좌표 → AMCL |
| POST   | `/nav/goal`               | `{x,y,yaw?}` → Nav2 goal (비동기) |
| POST   | `/nav/goal_wait`          | 동일 + 도착/실패/타임아웃까지 대기     |
| POST   | `/nav/stop`               | 주행 취소                       |
| POST   | `/nav/aruco_dock`         | ArUco 정밀 도킹 `{markerId, standoffM?, timeoutSec?}` |


### Controller 연동


| Method | Path                       | 설명                  |
| ------ | -------------------------- | ------------------- |
| GET    | `/controller/health`       | Controller 연결 확인    |
| GET    | `/controller/devices`      | 디바이스 목록             |
| GET    | `/controller/orders/:id`   | 주문 조회               |
| GET    | `/controller/missions`     | 미션 목록               |
| PATCH  | `/controller/missions/:id` | 미션 상태 보고            |
| POST   | `/telemetry/push`          | 센서 스냅샷 → Controller |
| POST   | `/heartbeat`               | 디바이스 status 갱신      |


Controller 쪽 대응 API (`server/apps/controller-server`):

- `GET/PATCH /missions`, `GET/PATCH /missions/:id`
- `PATCH /devices/:code`
- `POST/GET /robot/telemetry`

Controller에서 Pinky로 명령을내려면 `.env`에 `PINKY_URL=http://127.0.0.1:4200` 을 설정합니다.
(주문 Mock 파이프라인의 cart assign/navigate가 Pinky HTTP를 호출합니다.)

---

## 맵 · Nav2 네비게이션

### `run.py` 한 번으로 (권장)

`PINKY_BACKEND=ros2` + `PINKY_AUTO_LAUNCH=auto`(기본) 이면 `run.py`가 서브프로세스로:

1. `ros2 launch pinky_bringup bringup_robot.launch.xml` — 모터·오돔·**sllidar**·battery  
2. `ros2 launch pinky_navigation bringup_launch.xml map:=<PINKY_MAP.yaml>` — AMCL·Nav2  

를 띄웁니다. 종료(`Ctrl+C` / atexit) 시 자식 launch도 종료합니다.

**겹치는 센서는 pinky 쪽에서 끔** (pinky_pro 유지):

| 센서 | 담당 |
|------|------|
| 라이다 `/scan` | pinky_pro sllidar (`PINKY_DEFER_LIDAR`) |
| 배터리 | pinky_pro `battery_publisher` (`PINKY_DEFER_BATTERY`) |
| IMU·초음파 | pinky `sensor_publisher` (계속 발행) |

사전: ROS2 + pinky_pro 워크스페이스 source, `ros2` CLI 사용 가능.

끄려면: `PINKY_AUTO_LAUNCH=0` (수동으로 위 launch 실행).

**중요: Nav2는 한 세트만.** `PINKY_AUTO_LAUNCH`와 수동 `ros2 launch pinky_navigation`을 동시에 켜면 `/amcl` `/bt_navigator` 등이 **여러 개**로 보여 goal이 `REJECTED`/`NO_TF` 됩니다. `run.py` auto-launch는 이미 Nav2가 있으면 추가 기동하지 않습니다. 중복이 감지되면 기본(`PINKY_NAV2_REPLACE_DUPLICATES=1`)으로 Nav2만 정리한 뒤 **한 세트**를 다시 띄웁니다. 끄려면 `PINKY_NAV2_REPLACE_DUPLICATES=0`. 수동 정리:

```bash
pkill -f 'pinky_navigation|pinky_bringup|nav2_container|component_container_isolated|run.py'
# 한 번만
cd ~/pinky && python3 run.py
# 확인: ros2 node list | grep amcl   → /amcl 한 줄만
```

투어/작업 중 `ensure localization`은 S1/S2 **홈 fallback을 하지 않습니다**. TF·마지막 주행 pose·도킹 hold만 사용하고, 없으면 goal을 거부합니다 (대기 점프 방지).
### 대기 중 AMCL lifecycle

대기(idle)에서는 `/amcl`을 **deactivate**해 라이다로 pose가 점프하지 않게 합니다. NavigateToPose·수동 initialpose 직전에만 activate + `initialpose` 후 주행하고, 정지·도착 시 다시 deactivate합니다.

| 변수 | 기본 | 설명 |
|------|------|------|
| `PINKY_AMCL_IDLE_FREEZE` | `1` | idle 시 AMCL deactivate |
| `PINKY_AMCL_NODE` | `amcl` | lifecycle 노드명 |
| `PINKY_LOCALIZE_SETTLE_SEC` | `1.0` | activate 후 settle |
| `PINKY_AMCL_IDLE_FREEZE_DELAY_SEC` | `45.0` | 주행 종료 후 deactivate 유예 (투어 중 유지) |

투어: **첫 goal(또는 AMCL off/대기 freeze)만** `initialpose`. 이후 구간은 AMCL을 켠 채 goal만 전송.

`/initialpose` 발행은 **VOLATILE** (래치 없음). 예전 TRANSIENT_LOCAL 은 부트 홈 pose가 남아 AMCL 재활성 시 대기장소로 점프하는 원인이 됨.

작업/투어 중에는 S1/S2 **홈 initialpose·모니터 시드가 거부**됩니다 (`home seed locked out`). 대기장소 idle freeze 시에만 잠금이 풀립니다.

`nav2_params.yaml` 의 `amcl.set_initial_pose` 는 **false** 로 둔다. true 이면 AMCL activate 시마다 yaml 홈(S1)으로 점프한다.

`GET /nav/state`에 `amclActive`, `localizationMode` (`idle`|`active`)가 포함됩니다.

### HTTP 네비 API

`run.py`는 Nav2 **클라이언트 브리지**도 제공합니다 (`/nav/*`, `/map/*`).

좌표는 **map 프레임 미터**. 관리자 UI: 좌드래그 pose · 우클릭 goal.

맵 파일: `PINKY_MAP=map_test1` → `map_test1.yaml` + `.pgm`

### ArUco 정밀 도킹

Nav2로 W*/C/P 근처 도착 후 **SEARCH → FACE(정면) → SHIFT(짧은 횡스텝) → FACE 재검출 반복 → APPROACH(거리)** 순으로 도킹합니다. 횡이동은 한 번에 크게 밀지 않고 `SHIFT_STEP_M`만큼만 옮긴 뒤 마커를 다시 잡아 정밀 조정합니다. 도킹 중 AMCL을 freeze 해 모니터 pose 점프를 막습니다.

| 변수 | 기본 | 설명 |
|------|------|------|
| `PINKY_CAMERA_DEVICE` | `/dev/video0` | V4L 장치 (USB) |
| `PINKY_CAMERA_BACKEND` | `auto` | `picamera2` (Pi CSI 권장) / `v4l2` / `gstreamer` |
| `PINKY_CAMERA_FLIP` | `hv` | `hv`/`180`=180°(권장), `v`=수직만(미러→ArUco 깨짐), `h`, `none` |
| `PINKY_CAMERA_WIDTH` / `HEIGHT` | `640` / `480` | 캡처 해상도 (캘리브와 맞출 것) |
| `PINKY_CAMERA_CALIB_PATH` | `camera_calibration.npz` | `camera_matrix`, `distortion_coefficients` |
| `PINKY_ARUCO_DICT` | `DICT_5X5_50` | 마커 딕셔너리 |
| `PINKY_CAMERA_INTRINSICS` | `auto` | `auto`: 깨진 calib(과대 fx·왜곡)이면 FOV 핀홀 사용. `fov` 강제 / `calib` 파일 그대로 |
| `PINKY_CAMERA_HFOV_DEG` | `62` | FOV 모드 수평 화각 (Pi 캠 대략값) |
| `PINKY_ARUCO_MARKER_LENGTH_M` | `0.037` | **검은 사각형 한 변 실측(m)** (3.7cm). 거리·횡이동 스케일 |
| `PINKY_ARUCO_DOCK_STANDOFF_M` | `0.12` | 도착 거리(~12cm). **정자세(중앙·횡·정면) 우선** 후 이 거리 이내만 ARRIVED |
| `PINKY_ARUCO_CENTER_TOL_PX` | `15` | ALIGN 완료 판정 (가로 px) |
| `PINKY_ARUCO_CENTER_Y_TOL_PX` | `25` | ALIGN 완료 판정 (세로 px) |
| `PINKY_ARUCO_ALIGN_SETTLE_FRAMES` | `8` | 중앙 유지 연속 프레임 후 APPROACH |
| `PINKY_ARUCO_SHIFT_STEP_M` | `0.04` | 횡이동 1회 최대 거리 (m) — 이후 FACE 재검출 |
| `PINKY_ARUCO_SHIFT_STEP_GAIN` | `0.55` | 이번 스텝에서 보정할 tx 비율 |
| `PINKY_ARUCO_SHIFT_MAX_ITERS` | `12` | micro-SHIFT 최대 반복 |
| `PINKY_ARUCO_SHIFT_SETTLE_SEC` | `0.25` | 스텝 후 정지·재검출 대기 |
| `PINKY_ARUCO_APPROACH_V` | `0.04` | 최대 접근 속도 (m/s) |
| `PINKY_ARUCO_APPROACH_GAIN` | `0.35` | 속도 = gain × (현재거리 − standoff) |
| `PINKY_ARUCO_SEARCH_W` | `0.15` | SEARCH 회전 속도 (rad/s) |
| `PINKY_ARUCO_SEARCH_AMPLITUDE_DEG` | `70` | 한쪽 끝 각도. T=amp/w 후 2T로 반대쪽까지 왕복 (양쪽 FOV) |
| `PINKY_ARUCO_IDS` | `W1:1,…,C:10,P:11` | 웨이포인트→마커 ID |

캘리브 파일: 저장소의 `pinky/camera_calibration.npz`.

수동 검증:

```bash
# mock
curl -s -X POST http://127.0.0.1:4200/nav/aruco_dock \
  -H 'Content-Type: application/json' \
  -d '{"markerId":1,"standoffM":0.03,"timeoutSec":45}'

# 실기: 마커를 카메라 앞에 두고 동일 호출 → ALIGN→APPROACH→~3cm
# 주문 투어: mission note 에 `aruco dock W3 ok distance=0.031` 확인
# 마커 없음: status NO_MARKER / TIMEOUT → 미션 실패
```

Controller는 Nav2 성공 후 W*/C/P에 `POST /nav/aruco_dock`을 호출합니다 (`ARUCO_MARKER_BY_WAYPOINT`로 ID 덮어쓰기 가능).

---

## 모듈 사용 예

```python
from modules import PinkyRobot

robot = PinkyRobot(backend="mock", device_code="cart-1")
robot.start()

print(robot.battery.read().to_dict())
print(robot.lidar.read().to_dict())
print(robot.navigation.map_info())
robot.navigation.set_initial_pose(0.0, 0.0, 0.0)
robot.navigation.go_to(1.0, 0.5, 0.0)
robot.led.fill(255, 0, 0)
robot.lcd.set_emotion("happy")
robot.stop()
```

