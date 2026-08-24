"""OMX 픽업 팔을 HTTP 서버로 감싼다.

중앙 관제 서버가 "샌드위치를 1번 카트에" 같은 요청을 보내면, 그에 맞는
자연어 지시문을 만들어 정책을 돌리고, 완료·성공 여부를 돌려준다.

    python -m omx_yolo.server --policy <체크포인트경로> --port 8080

────────────────────────────────────────────────────────────────────────
설계
────────────────────────────────────────────────────────────────────────
정책은 종료 신호를 내지 않는다. 그래서 서버가 대신 판단한다.

    1. 요청 수신          {"product": "sandwich", "box": "box1"}
    2. 지시문 생성        "Pick up sandwich and place it in the box1"
    3. 추론 루프 (30fps)  관측 → 정책 → 로봇
    4. 종료 판정          홈 자세 복귀 (kinematic.HomeDetector)
    5. 성공 판정          그리퍼 구간 + 놓는 순간 shoulder_pan
    6. 응답 반환          {"success": true, "reason": "", ...}

한 번에 한 요청만 처리한다. 팔이 하나뿐이므로 동시 실행은 의미가 없고
위험하다. 처리 중 들어온 요청은 409 로 거절한다.

의존성을 늘리지 않으려고 표준 라이브러리 http.server 를 쓴다. FastAPI 를
넣으면 torch·opencv 조합이 깨질 위험이 있다.

────────────────────────────────────────────────────────────────────────
API
────────────────────────────────────────────────────────────────────────
GET  /health
    장치 연결과 리그 정합 상태. 수집·운용 전 점검용.

GET  /status
    {"busy": false, "last": {...}}

POST /pick
    요청  {"product": "sandwich", "box": "box1", "timeout_s": 90}
    응답  {"success": true, "finished": true, "grasped": true,
           "dest_ok": true, "seconds": 31.0, "grip_min": 52.7,
           "release_pan": -27.5, "reason": "", "task": "Pick up ..."}

POST /home
    팔을 홈 자세로 되돌린다. 실패 후 복구용.
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

from lerobot.utils.constants import OBS_STR

from .annotate import (CONTROLLER_DEVICE_BOX, CONTROLLER_SLUG, PRODUCT_PHRASE,
                       build_task, resolve_device, resolve_slug)
from .kinematic import KinematicJudge

logger = logging.getLogger("omx_yolo.server")

PRODUCTS = tuple(PRODUCT_PHRASE)      # sandwich milk icecream cake biscuit roll
BOXES = ("box1", "box2")

# 진열대 한 칸에 놓이는 최대 개수. 리필 없이 연속 픽업의 물리적 상한이다.
SHELF_CAPACITY = 3


def _cors_allow_origin() -> str:
    """LAN/원격 관제·브라우저에서 접근할 때 CORS. 기본 * (인증 없음)."""
    return (os.environ.get("OMX_ALLOW_ORIGINS") or "*").strip() or "*"


def _list_lan_urls(port: int) -> list[str]:
    """관제 .env 의 OMX_URL 설정용 LAN 주소 목록."""
    seen: set[str] = set()
    urls: list[str] = []
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            if ip and ip not in seen:
                seen.add(ip)
                urls.append(f"http://{ip}:{port}")
    except OSError:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if ip.startswith("127."):
                continue
            if ip not in seen:
                seen.add(ip)
                urls.append(f"http://{ip}:{port}")
    except OSError:
        pass
    return urls


class ArmController:
    """로봇·정책을 한 번만 올려두고 픽업 요청을 순차 처리한다."""

    def __init__(self, policy_path: str, robot_port: str, robot_id: str,
                 top_device: str, hand_device: str, weights: str,
                 annotate: bool, fps: int = 30, retries: int = 0):
        self.fps = fps
        # 헛집었을 때 다시 시도할 횟수. 기본 0 = 예전처럼 첫 실패에서 중단.
        # 재시도는 성공률을 올리는 것이 아니라 **기회를 더 주는 것**이다.
        # 한 번에 28초쯤 걸리므로 무한정 늘릴 수는 없다.
        self.retries = int(retries)
        self.lock = threading.Lock()
        self.busy = False
        self.last: dict | None = None
        self.job: dict | None = None        # 진행 중/최근 작업
        # 화면 송출용 최근 프레임 캐시. 제어 루프가 이미 받아 온 프레임을
        # 그대로 재사용한다 — 스트림 때문에 YOLO 를 다시 돌리지 않기 위해서다.
        self._frames: dict[str, object] = {}
        self._frame_t = 0.0
        self._frame_lock = threading.Lock()
        self._stop: str | None = None       # None | 'afterCurrent' | 'immediate'
        self._seq = 0
        self.judge = KinematicJudge()
        self._annotate = annotate

        from lerobot.cameras.opencv import OpenCVCameraConfig
        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.policies.factory import make_pre_post_processors
        from lerobot.robots.omx_follower import OmxFollowerConfig
        from lerobot.robots.utils import make_robot_from_config

        cam_cls = OpenCVCameraConfig
        cam_kwargs: dict = {}
        if annotate:
            from .camera import YoloOpenCVCameraConfig  # 등록도 함께 발동

            cam_cls = YoloOpenCVCameraConfig
            cam_kwargs = {"weights": weights}

        # 카메라 키는 반드시 camera1 / camera2 다. 정책이 아는 이름이기 때문이다.
        #
        # 학습은 --rename_map 으로 front→camera1, wrist→camera2 로 돌렸으므로
        # 체크포인트의 input_features 는 observation.images.camera1/2(/3) 이다.
        # 여기서 front/wrist 로 두면 predict_action 이 넘기는 관측 키가
        # policy 가 찾는 키와 어긋나 이미지 입력이 통째로 비게 된다
        # (rename_map={} 이라 중간에 이름을 바꿔 주는 단계가 없다).
        #
        #   camera1 = 탑뷰(주석 O)   camera2 = 손목(주석 X)
        # 순서를 바꾸면 정책이 두 시점을 뒤바꿔 받는다.
        cams = {
            "camera1": cam_cls(index_or_path=top_device, width=640, height=480,
                               fps=fps, fourcc="MJPG", warmup_s=2, **cam_kwargs),
            "camera2": OpenCVCameraConfig(index_or_path=hand_device, width=640,
                                          height=480, fps=fps, fourcc="MJPG"),
        }
        self.robot = make_robot_from_config(
            OmxFollowerConfig(port=robot_port, id=robot_id, cameras=cams))

        logger.info("정책 로드 중: %s", policy_path)
        cfg = PreTrainedConfig.from_pretrained(policy_path)
        cfg.pretrained_path = policy_path
        self.policy = self._load_policy(cfg, policy_path)
        self.pre, self.post = make_pre_post_processors(
            policy_cfg=cfg, pretrained_path=policy_path,
            preprocessor_overrides={"device_processor": {"device": cfg.device}})
        # predict_action 은 torch.device 를 기대한다(device.type 을 읽는다).
        # cfg.device 는 "cuda" 문자열이라 그대로 넘기면 죽는다:
        #     AttributeError: 'str' object has no attribute 'type'
        # lerobot_record.py:361 도 get_safe_torch_device 를 거쳐서 넘긴다.
        from lerobot.utils.utils import get_safe_torch_device

        self.device = get_safe_torch_device(cfg.device)
        self.cfg = cfg

        self.robot.connect()

        # 관측·액션 처리기 — lerobot-record 와 같은 경로를 쓴다.
        #
        # predict_action 에 robot.get_observation() 결과를 그대로 넘기면 안 된다.
        # 그 결과는 {"shoulder_pan.pos": 12.3, ..., "camera1": ndarray} 처럼
        # 관절이 스칼라 float 이고, 정책은 observation.state 배열을 기대한다:
        #     TypeError: expected np.ndarray (got float)
        #
        # lerobot_record.py:351 이 하는 일을 그대로 한다:
        #     obs → robot_observation_processor → build_dataset_frame
        # build_dataset_frame 이 관절들을 observation.state 로 묶고 카메라를
        # observation.images.<키> 로 옮긴다.
        from lerobot.datasets.pipeline_features import (
            aggregate_pipeline_dataset_features, create_initial_features)
        from lerobot.datasets.utils import build_dataset_frame, combine_feature_dicts
        from lerobot.policies.utils import make_robot_action
        from lerobot.processor.factory import make_default_processors

        (self._teleop_proc, self._act_proc,
         self._obs_proc) = make_default_processors()
        self._features = combine_feature_dicts(
            aggregate_pipeline_dataset_features(
                pipeline=self._teleop_proc,
                initial_features=create_initial_features(action=self.robot.action_features),
                use_videos=True),
            aggregate_pipeline_dataset_features(
                pipeline=self._obs_proc,
                initial_features=create_initial_features(
                    observation=self.robot.observation_features),
                use_videos=True),
        )
        self._build_frame = build_dataset_frame
        self._make_robot_action = make_robot_action
        logger.info("로봇 연결 완료. 준비됨.")

    @staticmethod
    def _load_policy(cfg, path):
        from lerobot.policies.factory import get_policy_class

        return get_policy_class(cfg.type).from_pretrained(path, config=cfg).eval()

    # ────────────────────────────────────────────────────────────────
    def _set_camera_task(self, task: str) -> None:
        """주석 카메라에 이번 타겟·목적지를 알린다."""
        if not self._annotate:
            return
        from .annotate import parse_task

        cam = self.robot.cameras.get("camera1")
        if cam is not None and hasattr(cam, "set_task"):
            cam.set_task(*parse_task(task))
            if hasattr(cam, "reset_tracker"):
                cam.reset_tracker()

    def pick(self, product: str, box: str, timeout_s: float = 90.0) -> dict:
        from lerobot.utils.control_utils import predict_action

        task = build_task(product, box)
        self._set_camera_task(task)
        self.policy.reset()

        from .success import HomeDetector

        states: list[np.ndarray] = []
        # 종료 감지는 매 프레임 한 번씩만 갱신한다. 프레임마다 전체 시퀀스를
        # 다시 판정하면 O(n²) 이 되어 30fps 를 못 지킨다.
        det = HomeDetector()
        det.reset()
        t0 = time.perf_counter()
        finished = False
        period = 1.0 / self.fps
        slow = 0

        aborted = False
        while time.perf_counter() - t0 < timeout_s:
            if self._stop == "immediate":
                aborted = True
                break
            tick = time.perf_counter()
            obs = self.robot.get_observation()
            s = np.array(
                [obs[f"{m}.pos"] for m in
                 ("shoulder_pan", "shoulder_lift", "elbow_flex",
                  "wrist_flex", "wrist_roll", "gripper")], dtype=np.float32)
            states.append(s)

            self._cache_frames(obs)
            obs_processed = self._obs_proc(obs)
            frame = self._build_frame(self._features, obs_processed, prefix=OBS_STR)
            action_values = predict_action(
                observation=frame, policy=self.policy, device=self.device,
                preprocessor=self.pre, postprocessor=self.post,
                use_amp=getattr(self.cfg, "use_amp", False),
                task=task, robot_type=self.robot.robot_type)
            act = self._make_robot_action(action_values, self._features)
            self.robot.send_action(self._act_proc((act, obs)))

            if det.update(s):
                finished = True
                break

            left = period - (time.perf_counter() - tick)
            if left < 0:
                slow += 1
            time.sleep(max(0.0, left))

        if slow:
            logger.warning("제어 주기를 %d/%d 프레임에서 놓쳤습니다 (%.0f%%). "
                           "추론이 %dfps 를 못 따라갑니다.",
                           slow, len(states), 100 * slow / max(len(states), 1), self.fps)

        seq = np.stack(states) if states else np.zeros((1, 6), np.float32)
        verdict = self.judge(seq, box)
        out = {
            "task": task, "product": product, "box": box,
            "success": bool(verdict.success and finished and not aborted),
            "finished": bool(finished),
            "grasped": bool(verdict.grasped),
            "dest_ok": verdict.dest_ok,
            "dest_pred": verdict.dest_pred,
            "seconds": round(len(seq) / self.fps, 2),
            "grip_min": round(float(verdict.grip_min), 2),
            "release_pan": (None if np.isnan(verdict.release_pan)
                            else round(float(verdict.release_pan), 2)),
            "aborted": bool(aborted),
            "reason": ("운영자 즉시 정지" if aborted else
                       verdict.reason if finished else "시간 초과 — 홈으로 복귀하지 않음"),
        }
        self.last = out
        return out

    # ────────────────────────────────────────────────────────────────
    #  작업(job) — 수량 N 개를 순차 픽업하고, 도중에 끊을 수 있다
    # ────────────────────────────────────────────────────────────────
    def start_job(self, slug: str, device_code: str, quantity: int,
                  order_id: int = 0, timeout_s: float = 90.0,
                  retries: int | None = None) -> dict:
        """픽업 작업을 시작하고 즉시 반환한다. 실제 동작은 워커 스레드가 한다.

        블로킹으로 두면 안 되는 이유: N=3 이면 최대 4.5분이고, 그동안 관제가
        진행 개수도 못 보고 인터럽트도 못 건다. 관제는 이미 /nav/state 를
        0.35초 주기로 폴링하는 구조이므로 같은 방식에 맞춘다.
        """
        product = resolve_slug(slug)          # 모르면 ValueError
        box = resolve_device(device_code)     # 모르면 ValueError
        n = int(quantity)
        if n < 1:
            raise ValueError(f"quantity 는 1 이상이어야 합니다: {quantity}")
        if n > SHELF_CAPACITY:
            raise ValueError(
                f"quantity 가 진열대 재고 상한({SHELF_CAPACITY})을 넘습니다: {n}. "
                f"리필 없이는 한 번에 {SHELF_CAPACITY} 개까지만 집을 수 있습니다.")

        if retries is not None:
            self.retries = max(0, int(retries))
        self._seq += 1
        self._stop = None
        self.job = {
            "jobId": f"j{self._seq}",
            "orderId": int(order_id),
            "deviceCode": device_code,
            "slug": slug,
            "product": product,
            "box": box,
            "total": n,
            "done": 0,
            "state": "RUNNING",
            "currentIndex": 1,
            "startedAt": time.time(),
            "results": [],
            "retries": 0,
            "message": "",
        }
        t = threading.Thread(target=self._run_job, args=(timeout_s,), daemon=True)
        t.start()
        return self.state()

    def _run_job(self, timeout_s: float) -> None:
        job = self.job
        assert job is not None
        try:
            self.busy = True
            tries = 0
            i = 0
            while job["done"] < job["total"]:
                i = job["done"] + 1
                if self._stop:
                    job["state"] = "ABORTED"
                    job["message"] = f"운영자 정지({self._stop})"
                    break
                job["currentIndex"] = i
                r = self.pick(job["product"], job["box"], timeout_s)
                r["index"] = i
                job["results"].append(r)

                if r.get("aborted"):
                    job["state"] = "ABORTED"
                    job["message"] = "운영자 즉시 정지"
                    break
                if not r["success"]:
                    # 실패에는 두 종류가 있고, 재시도해도 되는 것은 하나뿐이다.
                    #
                    #   grasped=False  아무것도 못 집었다. 진열 상태가 그대로
                    #                  이므로 학습 분포 안이고, 다시 해도 된다.
                    #                  (오늘 실패가 이것: grip 49.3 미끄러짐)
                    #
                    #   grasped=True   집었는데 놓치거나 엉뚱한 데 뒀다. 물건이
                    #                  어디 갔는지 모른다. 진열 상태가 학습에
                    #                  없는 형태가 됐을 수 있으므로 재시도하면
                    #                  빈 칸을 헛집는다(2026-08-19 실측).
                    #
                    # 그래서 헛집은 경우만 다시 한다. 그것도 정해진 횟수까지다 —
                    # 리그가 틀어졌거나 물건이 닿지 않는 자리에 있으면 몇 번을
                    # 해도 안 되고, 그때는 사람이 봐야 한다.
                    retryable = (not r.get("grasped")) and not r.get("aborted")
                    if retryable and tries < self.retries:
                        tries += 1
                        job["retries"] = tries
                        logger.warning(
                            "픽업 실패(%s) — 아무것도 집지 못했으므로 다시 "
                            "시도합니다 (%d/%d)",
                            r.get("reason"), tries, self.retries)
                        self._mission_retry_note(job, r, tries)
                        continue          # i 를 늘리지 않는다 — 같은 한 개다
                    job["state"] = "FAILED"
                    job["message"] = r.get("reason") or "픽업 실패"
                    if retryable and self.retries:
                        job["message"] += f" ({self.retries}회 재시도 후)"
                    break
                job["done"] += 1
                tries = 0                 # 하나 성공했으면 재시도 횟수를 되돌린다
            if job["state"] == "RUNNING" and job["done"] >= job["total"]:
                job["state"] = "DONE"
                job["message"] = ""

            if self._stop == "immediate":
                self.go_home()
        except Exception as exc:                     # noqa: BLE001
            logger.exception("작업 실패")
            job["state"] = "FAILED"
            job["message"] = str(exc)
        finally:
            job["finishedAt"] = time.time()
            self.busy = False
            self._stop = None

    @staticmethod
    def _mission_retry_note(job: dict, r: dict, tries: int) -> None:
        """재시도를 결과 목록에 남긴다. 관제가 몇 번 만에 됐는지 볼 수 있어야
        한다 — 조용히 다시 하면 성공률이 실제보다 좋아 보인다."""
        r["retried"] = tries

    def request_stop(self, mode: str) -> dict:
        """mode='afterCurrent' 현재 1개 끝내고 정지 · 'immediate' 즉시 정지."""
        if mode not in ("afterCurrent", "immediate"):
            raise ValueError("mode 는 afterCurrent 또는 immediate 여야 합니다")
        if not self.busy:
            return {"success": True, "status": "IDLE",
                    "message": "진행 중인 작업이 없습니다"}
        self._stop = mode
        return {"success": True, "status": "STOPPING", "mode": mode,
                "message": ("현재 픽업을 끝내고 정지합니다" if mode == "afterCurrent"
                            else "즉시 정지 후 홈으로 복귀합니다")}

    def state(self) -> dict:
        """관제가 폴링하는 진행 상태. done 이 '현재까지 담은 개수' 다."""
        if self.job is None:
            return {"success": True, "status": "IDLE", "busy": False}
        j = self.job
        out = {
            "success": j["state"] in ("RUNNING", "DONE"),
            "status": j["state"],
            "busy": bool(self.busy),
            "jobId": j["jobId"], "orderId": j["orderId"],
            "deviceCode": j["deviceCode"], "slug": j["slug"], "box": j["box"],
            "total": j["total"], "done": j["done"],
            "currentIndex": j["currentIndex"],
            "retries": j.get("retries", 0),
            "maxRetries": self.retries,
            "elapsedSec": round(time.time() - j["startedAt"], 1),
            "results": j["results"],
            "message": j["message"],
        }
        if self._stop:
            out["stopRequested"] = self._stop
        return out

    # ────────────────────────────────────────────────────────────────
    #  화면 송출 — 제어 루프가 본 것과 똑같은 프레임을 내보낸다
    # ────────────────────────────────────────────────────────────────
    def _cache_frames(self, obs: dict) -> None:
        """제어 루프가 받은 관측에서 카메라 프레임만 떼어 보관한다."""
        with self._frame_lock:
            for k in ("camera1", "camera2"):
                if k in obs:
                    self._frames[k] = obs[k]
            self._frame_t = time.time()

    def get_frame(self, cam: str = "camera1", max_age_s: float = 0.5):
        """송출용 프레임(RGB ndarray).

        픽업 중에는 제어 루프가 채워 둔 것을 그대로 쓴다. 그 프레임이야말로
        정책이 실제로 본 화면이므로, 화면과 동작이 어긋나지 않는다.

        멈춰 있을 때는 캐시가 낡으므로 카메라에서 새로 읽는다. 이때는
        주석 계층을 그대로 통과하므로 탑뷰에 박스가 그려진다(YOLO 5.6ms).
        """
        with self._frame_lock:
            fresh = (time.time() - self._frame_t) < max_age_s
            f = self._frames.get(cam)
        if fresh and f is not None:
            return f
        c = self.robot.cameras.get(cam)
        if c is None:
            return None
        try:
            f = c.read_latest()
            with self._frame_lock:
                self._frames[cam] = f
                self._frame_t = time.time()
            return f
        except Exception:                            # noqa: BLE001
            return f

    @staticmethod
    def encode_jpeg(frame, quality: int = 80) -> bytes | None:
        import cv2

        if frame is None:
            return None
        bgr = cv2.cvtColor(np.asarray(frame), cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode(".jpg", bgr,
                               [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
        return buf.tobytes() if ok else None

    def go_home(self, seconds: float = 3.0) -> dict:
        """홈 자세로 천천히 복귀. 실패 후 복구용."""
        from .success import HOME

        n = int(seconds * self.fps)
        obs = self.robot.get_observation()
        names = list(self.robot.action_features)
        cur = np.array([obs[f"{m}.pos"] for m in
                        ("shoulder_pan", "shoulder_lift", "elbow_flex",
                         "wrist_flex", "wrist_roll", "gripper")], np.float32)
        for i in range(n):
            a = cur + (HOME - cur) * ((i + 1) / n)
            self.robot.send_action({k: float(v) for k, v in zip(names, a)})
            time.sleep(1.0 / self.fps)
        return {"ok": True}

    def _raw_top_frame(self):
        """탑뷰 원본 프레임. 이미 열려 있는 로봇 카메라에서 가져온다.

        checkrig.grab() 은 장치를 새로 연다. 서버가 도는 동안 그 장치는
        로봇이 잡고 있으므로 열리지 않는다:
            [WARN] cap.cpp: backend is generally available but can't be
                   used to capture by name
        그래서 /health 가 통째로 실패했다(2026-08-19 실측).

        주석이 그려지지 않은 원본이 필요하므로 부모 클래스의 read_latest 를
        직접 부른다. 주석이 그려진 프레임은 흰 칸 검출을 방해할 수 있다.
        """
        from lerobot.cameras.opencv import OpenCVCamera

        cam = self.robot.cameras.get("camera1")
        if cam is None:
            raise RuntimeError("탑뷰 카메라(camera1)가 없습니다")
        frame = OpenCVCamera.read_latest(cam)          # 주석 계층 우회
        return frame

    def health(self) -> dict:
        """관제의 is_reachable() 이 이걸로 도달 가능 여부를 판단한다.

        어떤 경우에도 예외를 밖으로 내지 않는다. 여기서 죽으면 관제가
        OMX 를 '도달 불가' 가 아니라 '응답 깨짐' 으로 보게 된다.
        """
        import json as _json
        from pathlib import Path

        from .checkrig import REF_PATH, compare, find_cells

        out = {"success": True, "status": "OK", "message": "",
               "robotConnected": bool(self.robot.is_connected),
               "busy": bool(self.busy)}
        try:
            if Path(REF_PATH).exists():
                ref = [tuple(c) for c in _json.load(open(REF_PATH))["cells"]]
                cells = find_cells(self._raw_top_frame())
                m, mx, match = compare(cells, ref)
                out["rig"] = {"meanShiftPx": round(m, 1),
                              "maxShiftPx": round(mx, 1),
                              "matchRatio": round(match, 2),
                              "ok": bool(m < 8.0)}
                if not out["rig"]["ok"]:
                    out["success"] = False
                    out["status"] = "DEGRADED"
                    out["message"] = "리그가 기준과 어긋났습니다 — 픽업 정확도를 보장할 수 없습니다"
            else:
                out["rig"] = {"error": "기준이 없습니다. checkrig --save-reference 실행"}
        except Exception as e:                      # noqa: BLE001
            out["rig"] = {"error": str(e)}
        return out

    def close(self) -> None:
        try:
            self.robot.disconnect()
        except Exception:                            # noqa: BLE001
            pass


# ───────────────────────────────────────────────────────────────────────
VIEW_HTML = """<!doctype html><html lang="ko"><head><meta charset="utf-8">
<title>OMX 픽업 화면</title>
<style>
 :root{color-scheme:dark light}
 body{margin:0;background:#0d1116;color:#e3eaf0;
      font-family:ui-monospace,Menlo,Consolas,monospace}
 header{padding:14px 20px;border-bottom:1px solid #28323c;display:flex;
        gap:16px;align-items:baseline;flex-wrap:wrap}
 h1{font-size:16px;margin:0;letter-spacing:-.02em}
 #st{font-size:12px;color:#7a8895}
 #st b{color:#4fcb84}
 .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));
       gap:16px;padding:16px}
 figure{margin:0}
 img{width:100%;display:block;border:1px solid #3a4650;border-radius:4px;background:#1c242d}
 figcaption{font-size:12px;color:#7a8895;margin-top:6px}
 .tag{color:#6ba5f5}
</style></head><body>
<header>
  <h1>OMX 픽업 화면</h1>
  <span id="st">상태 읽는 중…</span>
</header>
<div class="grid">
  <figure><img src="/stream?cam=camera1&fps=12" alt="탑뷰">
    <figcaption><span class="tag">camera1</span> 탑뷰 — 정책이 보는 화면 (YOLO 박스 포함)</figcaption></figure>
  <figure><img src="/stream?cam=camera2&fps=12" alt="손목">
    <figcaption><span class="tag">camera2</span> 손목 — 원본</figcaption></figure>
</div>
<script>
async function tick(){
  try{
    const r = await fetch('/pick/state'); const j = await r.json();
    const el = document.getElementById('st');
    if(j.status === 'IDLE'){ el.innerHTML = '<b>IDLE</b> — 대기 중'; }
    else{
      el.innerHTML = `<b>${j.status}</b> · ${j.slug||''} → ${j.box||''} ·
        ${j.done}/${j.total} 개 · ${j.elapsedSec||0}초` +
        (j.message ? ` · ${j.message}` : '');
    }
  }catch(e){}
  setTimeout(tick, 500);
}
tick();
</script></body></html>"""


def make_handler(ctrl: ArmController):
    """관제 서버(controller-server) 규약에 맞춘 HTTP 인터페이스.

    규약은 pinky 어댑터(PinkyHttpCartAdapter)에서 그대로 가져왔다:
      · POST 는 JSON 본문, 필드는 camelCase (orderId, deviceCode, timeoutSec)
      · 응답은 {"success": bool, "status": ..., "message": str}
      · GET /health 로 도달 가능 여부를 판단한다 (is_reachable)
      · 진행 상태는 폴링으로 읽는다 (관제는 이미 /nav/state 를 0.35초 주기로 폴링)
      · 인증 없음

    엔드포인트
      POST /pick        {"orderId","deviceCode","slug","quantity"}  → 202, 즉시 반환
      GET  /pick/state  진행 상태 (done = 지금까지 담은 개수)
      POST /pick/stop   {"mode":"afterCurrent"|"immediate"}
      POST /home        홈 복귀 (복구용)
      GET  /health      장치·리그 점검
    """

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt, *args):           # 기본 stderr 로그 억제
            logger.info("%s - %s", self.address_string(), fmt % args)

        def _add_cors_headers(self) -> None:
            origin = _cors_allow_origin()
            self.send_header("Access-Control-Allow-Origin", origin)
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")

        def do_OPTIONS(self):                        # noqa: N802
            self.send_response(204)
            self._add_cors_headers()
            self.end_headers()

        def _send(self, code: int, payload: dict) -> None:
            body = json.dumps(payload, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self._add_cors_headers()
            self.end_headers()
            self.wfile.write(body)

        def _q(self, key: str, default: str) -> str:
            from urllib.parse import parse_qs, urlparse

            return parse_qs(urlparse(self.path).query).get(key, [default])[0]

        def _send_html(self, html: str) -> None:
            body = html.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self._add_cors_headers()
            self.end_headers()
            self.wfile.write(body)

        def _stream(self, cam: str, fps: float) -> None:
            """MJPEG(multipart/x-mixed-replace). 브라우저 <img> 로 바로 보인다.

            길이를 미리 알 수 없으므로 keep-alive 를 끊는다. ThreadingHTTPServer
            라 이 연결이 오래 열려 있어도 다른 요청을 막지 않는다.
            """
            period = 1.0 / max(1.0, min(float(fps), 30.0))
            bound = "omxframe"
            self.close_connection = True
            self.send_response(200)
            self.send_header("Content-Type",
                             f"multipart/x-mixed-replace; boundary={bound}")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            try:
                while True:
                    t0 = time.time()
                    jpg = ctrl.encode_jpeg(ctrl.get_frame(cam))
                    if jpg:
                        self.wfile.write(
                            f"--{bound}\r\nContent-Type: image/jpeg\r\n"
                            f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                    time.sleep(max(0.0, period - (time.time() - t0)))
            except (BrokenPipeError, ConnectionResetError):
                pass                                  # 브라우저가 창을 닫음

        def _fail(self, code: int, message: str, **extra) -> None:
            self._send(code, {"success": False, "status": "FAILED",
                              "message": message, **extra})

        # ── GET ────────────────────────────────────────────────────
        def do_GET(self):                            # noqa: N802
            path = self.path.split("?")[0].rstrip("/") or "/"
            if path == "/health":
                self._send(200, ctrl.health())
            elif path in ("/pick/state", "/status"):   # /status 는 구 이름 유지
                self._send(200, ctrl.state())
            elif path == "/view":
                self._send_html(VIEW_HTML)
            elif path in ("/frame", "/frame.jpg"):
                cam = self._q("cam", "camera1")
                jpg = ctrl.encode_jpeg(ctrl.get_frame(cam))
                if jpg is None:
                    self._fail(503, f"프레임을 가져오지 못했습니다: {cam}")
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpg)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(jpg)
            elif path == "/stream":
                self._stream(self._q("cam", "camera1"),
                             float(self._q("fps", "10")))
            elif path == "/products":
                self._send(200, {"success": True, "status": "OK",
                                 "slugs": sorted(CONTROLLER_SLUG),
                                 "devices": sorted(CONTROLLER_DEVICE_BOX),
                                 "shelfCapacity": SHELF_CAPACITY, "message": ""})
            else:
                self._fail(404, f"unknown path: {path}")

        # ── POST ───────────────────────────────────────────────────
        def do_POST(self):                           # noqa: N802
            path = self.path.split("?")[0].rstrip("/") or "/"
            n = int(self.headers.get("Content-Length") or 0)
            try:
                req = json.loads(self.rfile.read(n) or b"{}")
            except json.JSONDecodeError as e:
                self._fail(400, f"JSON 파싱 실패: {e}")
                return
            if not isinstance(req, dict):
                self._fail(400, "JSON 본문은 객체여야 합니다")
                return

            if path == "/pick/stop":
                try:
                    self._send(200, ctrl.request_stop(
                        str(req.get("mode", "afterCurrent"))))
                except ValueError as e:
                    self._fail(400, str(e))
                return

            if path == "/home":
                if ctrl.busy:
                    self._fail(409, "픽업 작업이 진행 중입니다. 먼저 /pick/stop 하십시오.",
                               status="RUNNING")
                    return
                if not ctrl.lock.acquire(blocking=False):
                    self._fail(409, "다른 요청을 처리 중입니다", status="RUNNING")
                    return
                try:
                    ctrl.go_home()
                    self._send(200, {"success": True, "status": "DONE",
                                     "message": "홈 자세로 복귀했습니다"})
                except Exception as e:               # noqa: BLE001
                    logger.exception("홈 복귀 실패")
                    self._fail(500, str(e))
                finally:
                    ctrl.lock.release()
                return

            if path != "/pick":
                self._fail(404, f"unknown path: {path}")
                return

            # ── POST /pick ──────────────────────────────────────────
            if ctrl.busy:
                self._fail(409, "이미 픽업 작업을 처리 중입니다. 팔이 하나뿐입니다.",
                           status="RUNNING", jobId=(ctrl.job or {}).get("jobId"))
                return
            if not ctrl.lock.acquire(blocking=False):
                self._fail(409, "다른 요청을 처리 중입니다", status="RUNNING")
                return
            try:
                # slug 는 관제 DB 의 products.slug. quantity 는 order_items.quantity.
                out = ctrl.start_job(
                    slug=str(req.get("slug", "")),
                    device_code=str(req.get("deviceCode", "")),
                    quantity=int(req.get("quantity", 1)),
                    order_id=int(req.get("orderId", 0)),
                    timeout_s=float(req.get("timeoutSec", 90.0)),
                    retries=(int(req["retries"]) if "retries" in req else None),
                )
                self._send(202, out)
            except ValueError as e:
                self._fail(400, str(e))
            except Exception as e:                   # noqa: BLE001
                logger.exception("작업 시작 실패")
                self._fail(500, str(e))
            finally:
                ctrl.lock.release()

    return Handler


def main() -> None:
    p = argparse.ArgumentParser(description="OMX 픽업 서버")
    p.add_argument("--policy", required=True, help="정책 체크포인트 절대 경로")
    p.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("OMX_PORT", "8080")),
    )
    p.add_argument(
        "--host",
        default=os.environ.get("OMX_HOST", "0.0.0.0"),
        help="bind 주소. LAN에서 관제 PC가 접속하려면 0.0.0.0 (기본)",
    )
    p.add_argument("--robot-port", default="/dev/omx_follower")
    p.add_argument("--robot-id", default="omx_follower_arm")
    p.add_argument("--top", default="/dev/omx_cam_top")
    p.add_argument("--hand", default="/dev/omx_cam_hand")
    p.add_argument("--weights", default="/home/newuser/il_ws/models/omx_goods_yolo11n.pt")
    p.add_argument("--retries", type=int, default=0,
                   help="헛집었을 때(아무것도 못 집었을 때) 다시 시도할 횟수. "
                        "집었다가 놓친 경우는 진열 상태를 알 수 없으므로 "
                        "재시도하지 않는다. 기본 0")
    p.add_argument("--no-annotate", action="store_true",
                   help="YOLO 주석 없이 원본 프레임 사용. "
                        "주석 없이 학습한 정책을 돌릴 때 반드시 지정할 것")
    a = p.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")
    ctrl = ArmController(a.policy, a.robot_port, a.robot_id, a.top, a.hand,
                         a.weights, annotate=not a.no_annotate,
                         retries=a.retries)
    srv = ThreadingHTTPServer((a.host, a.port), make_handler(ctrl))
    logger.info(
        "서버 시작 http://%s:%d  (주석 %s, CORS=%s)",
        a.host,
        a.port,
        "켬" if not a.no_annotate else "끔",
        _cors_allow_origin(),
    )
    lan_urls = _list_lan_urls(a.port)
    if lan_urls:
        logger.info("관제 PC .env 예: OMX_URL=%s", lan_urls[0])
        for url in lan_urls[1:]:
            logger.info("  (추가 LAN) %s", url)
    logger.info("방화벽: TCP %d 허용 필요 (관제 PC → 이 PC)", a.port)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        logger.info("종료 중...")
    finally:
        srv.server_close()
        ctrl.close()


if __name__ == "__main__":
    main()
