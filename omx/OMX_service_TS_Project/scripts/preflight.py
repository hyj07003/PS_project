#!/usr/bin/env python3
"""통합 시험 전 리그 점검 — 조용히 틀린 채로 시작하는 것을 막는다.

    python3 ~/il_ws/scripts/preflight.py
    python3 ~/il_ws/scripts/preflight.py --skip cameras   # 특정 항목 건너뛰기

왜 필요한가 — 2026-08-21 하루에 겪은 것만 해도 이렇다.

  · /dev/omx_cam_hand 가 픽업이 아니라 **포장 팔 손목**을 가리키고 있었다.
    같은 모델 카메라가 2대씩이라 vendor:product 로는 안 갈렸다. 픽업 서버를
    그대로 띄웠으면 포장 화면으로 추론했을 것이다 — 예외도 경고도 없이.
  · 포장 팔 모터 ID 11 이 간헐적으로 응답하지 않아 서버 기동이 실패했다.
    조회로는 6개가 다 보이는데 연결만 실패한다.
  · 그리퍼가 63.03 으로 학습 범위(최대 61.83) 밖에 있었다. 정책은 학습한 적
    없는 상태에서도 자신 있게 움직인다.

전부 **시작하기 전에 알 수 있는 것들**이다. 시험 중에 발견하면 원인을 되짚는
데 훨씬 오래 걸린다.

종료 코드: 0 모두 통과 · 1 경고 있음 · 2 실패 있음
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

IL = "/home/newuser/venv/il/bin/python"
PACK = "/home/newuser/venv/pack/bin/python"
SRC = "/home/newuser/il_ws/src"
OUT = "/home/newuser/il_ws/src/lerobot/outputs/train"

OK, WARN, FAIL = "OK", "WARN", "FAIL"
COLOR = {OK: "\033[32m", WARN: "\033[33m", FAIL: "\033[31m"}
RESET = "\033[0m"

results: list[tuple[str, str, str]] = []


def record(name: str, status: str, detail: str = "") -> None:
    results.append((name, status, detail))
    tint = COLOR[status] if sys.stdout.isatty() else ""
    end = RESET if tint else ""
    print(f"  {tint}{status:4s}{end}  {name:34s} {detail}")


def run(cmd: list[str], timeout: float = 60.0) -> tuple[int, str]:
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return p.returncode, (p.stdout + p.stderr).strip()
    except subprocess.TimeoutExpired:
        return 124, "시간 초과"
    except FileNotFoundError as e:
        return 127, str(e)


# ── 1. 장치 이름 ───────────────────────────────────────────────────────
def check_devices() -> None:
    print("\n■ 장치 이름 (udev)")
    expect = {
        "/dev/omx_follower": "픽업 팔",
        "/dev/omx_pack_follower": "포장 팔",
        "/dev/omx_cam_top": "픽업 탑뷰",
        "/dev/omx_cam_hand": "픽업 손목",
        "/dev/omx_cam_pack_top": "포장 탑뷰",
        "/dev/omx_cam_pack_hand": "포장 손목",
    }
    for path, label in expect.items():
        p = Path(path)
        if not p.exists():
            record(f"{label} ({path})", FAIL, "없음 — 케이블 또는 udev 확인")
            continue
        record(f"{label} ({path})", OK, f"→ {os.path.realpath(path).split('/')[-1]}")

    # 같은 모델이 2대씩이라 포트 경로로만 갈린다. 서로 다른 장치를 가리키는지
    # 확인한다 — 같은 것을 가리키면 규칙이 무너진 것이다.
    cams = ["/dev/omx_cam_top", "/dev/omx_cam_hand",
            "/dev/omx_cam_pack_top", "/dev/omx_cam_pack_hand"]
    real = [os.path.realpath(c) for c in cams if Path(c).exists()]
    if len(real) != len(set(real)):
        record("카메라 4대가 서로 다른 장치인가", FAIL,
               "두 이름이 같은 장치를 가리킵니다 — udev 규칙 확인")
    elif len(real) == 4:
        record("카메라 4대가 서로 다른 장치인가", OK, "")


# ── 2. 카메라 ──────────────────────────────────────────────────────────
CAM_SNIPPET = r"""
import cv2, sys, time, json
out = {}
for dev in sys.argv[1:]:
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    t0, f, n = time.time(), None, 0
    while time.time() - t0 < 4.0:      # 워밍업이 필요한 카메라가 있다
        ok, x = cap.read()
        if ok: f, n = x, n + 1
    cap.release()
    out[dev] = None if f is None else [f.shape[1], f.shape[0], n]
print(json.dumps(out))
"""


def _server_up(port: int) -> bool:
    import urllib.request

    try:
        urllib.request.urlopen(f"http://127.0.0.1:{port}/health", timeout=2)
        return True
    except Exception:                                 # noqa: BLE001
        return False


def check_cameras() -> None:
    print("\n■ 카메라 읽기 (각 4초 워밍업)")
    cams = ["/dev/omx_cam_top", "/dev/omx_cam_hand",
            "/dev/omx_cam_pack_top", "/dev/omx_cam_pack_hand"]
    live = [c for c in cams if Path(c).exists()]
    if not live:
        record("카메라", FAIL, "장치가 없어 건너뜀")
        return

    # 서버가 떠 있으면 카메라를 잡고 있어서 여기서는 못 읽는다. 그것을
    # 고장으로 보고하면 오탐이다 — 실제로 2026-08-21 에 그렇게 나왔다.
    # 서버가 떠 있는 경우에는 서버의 /health 가 카메라 상태를 대신 말해 준다.
    busy = [p for p in (8080, 8081) if _server_up(p)]
    if busy:
        ports = ", ".join(f":{p}" for p in busy)
        record("카메라", WARN,
               f"서버({ports})가 카메라를 잡고 있어 건너뜀 — ■ 서버 항목을 볼 것")
        return
    code, out = run([PACK, "-c", CAM_SNIPPET, *live], timeout=40)
    try:
        data = json.loads(out.splitlines()[-1])
    except Exception:
        record("카메라 읽기", FAIL, f"판독 실패: {out[:80]}")
        return
    for dev, info in data.items():
        name = dev.split("/")[-1]
        if info is None:
            record(name, FAIL, "프레임을 못 읽음 (다른 프로세스가 잡고 있을 수 있음)")
        else:
            w, h, n = info
            fps = n / 4.0
            status = OK if (w, h) == (640, 480) and fps >= 10 else WARN
            record(name, status, f"{w}x{h} · 약 {fps:.0f} fps")


# ── 3. 로봇 팔 ─────────────────────────────────────────────────────────
ARM_SNIPPET = r"""
import sys, json
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.dynamixel import DynamixelMotorsBus
J = ("shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll","gripper")
M = ("xl430-w250",)*3 + ("xl330-m288",)*3
out = {}
for port in sys.argv[1:]:
    motors = {j: Motor(11+i, M[i],
              MotorNormMode.RANGE_0_100 if j=="gripper" else MotorNormMode.DEGREES)
              for i, j in enumerate(J)}
    try:
        bus = DynamixelMotorsBus(port=port, motors=motors)
        bus.connect(handshake=False)
        res = {}
        for j, m in motors.items():
            res[j] = sum(1 for _ in range(20) if bus.ping(m.id) is not None)
        pos = {}
        for j in J:
            try: pos[j] = round(float(bus.read("Present_Position", j)), 2)
            except Exception: pos[j] = None
        err = {}
        for j in J:
            try: err[j] = int(bus.read("Hardware_Error_Status", j, normalize=False))
            except Exception: err[j] = None
        try: bus.disconnect()
        except Exception: pass
        out[port] = {"ping": res, "pos": pos, "err": err}
    except Exception as e:
        out[port] = {"error": str(e)[:120]}
print(json.dumps(out))
"""


def check_arms() -> None:
    print("\n■ 로봇 팔 모터 버스 (핑만 — 팔은 움직이지 않음)")
    arms = {"/dev/omx_follower": "픽업", "/dev/omx_pack_follower": "포장"}
    live = [p for p in arms if Path(p).exists()]
    if not live:
        record("로봇 팔", FAIL, "장치가 없어 건너뜀")
        return

    # 서버가 떠 있으면 팔을 건드리지 않는다.
    #
    # Dynamixel 포트 핸들러는 스레드/프로세스 안전하지 않다. 서버의 제어
    # 루프가 30Hz 로 쓰는 동안 여기서 읽으면 서버 쪽이 이렇게 죽는다:
    #     Failed to sync write 'Goal_Position' ... [TxRxResult] Port is in use!
    # 2026-08-21 통합 시험 중 실제로 작업 하나가 통째로 실패했다.
    # 점검하자고 시험을 깨뜨릴 수는 없다.
    busy = [pt for pt in (8080, 8081) if _server_up(pt)]
    if busy:
        ports = ", ".join(f":{p}" for p in busy)
        record("로봇 팔", WARN,
               f"서버({ports})가 떠 있어 건너뜀 — 팔을 건드리면 작업이 깨집니다")
        return
    # 모터 버스는 가끔 통째로 응답하지 않는 구간이 있다(2026-08-21 실측:
    # 20회 연속 무응답 후 곧바로 정상 복구). 한 번의 실패로 단정하지 않고
    # 다시 재 본다 — 다만 재시도가 필요했다는 사실은 남긴다.
    data = None
    retried = False
    for attempt in (1, 2):
        code, out = run([PACK, "-c", ARM_SNIPPET, *live], timeout=120)
        try:
            data = json.loads(out.splitlines()[-1])
        except Exception:
            record("모터 버스", FAIL, f"판독 실패: {out[:100]}")
            return
        flaky = any("error" in v or any(n < 20 for n in v.get("ping", {}).values())
                    for v in data.values())
        if not flaky:
            break
        if attempt == 1:
            retried = True
    if retried and data is not None:
        record("모터 버스 재시도", WARN,
               "1회차에 응답 불안정 — 배선(특히 베이스 관절)을 확인하십시오")
    for port, info in data.items():
        label = arms[port]
        if "error" in info:
            record(f"{label} 팔", FAIL, info["error"])
            continue
        bad = {j: n for j, n in info["ping"].items() if n < 20}
        if bad:
            worst = ", ".join(f"{j} {n}/20" for j, n in bad.items())
            record(f"{label} 팔 통신", FAIL if any(n == 0 for n in bad.values())
                   else WARN, f"응답 불안정: {worst}")
        else:
            record(f"{label} 팔 통신", OK, "6개 모터 20/20 응답")
        errs = {j: e for j, e in info["err"].items() if e}
        record(f"{label} 팔 하드웨어 에러", FAIL if errs else OK,
               str(errs) if errs else "래치된 에러 없음")


# ── 4. 시작 자세가 학습 분포 안인가 ────────────────────────────────────
def check_start_pose(enabled: bool = False) -> None:
    print("\n■ 포장 팔 시작 자세 (정책이 본 적 있는 상태인가)")
    if not enabled:
        # 이 점검만 팔에 연결한다 = 토크가 켜진다. 나머지 점검은 전부
        # 읽기뿐이라 팔이 움직이지 않는다. 놀라지 않도록 기본은 건너뛴다.
        record("시작 자세", WARN,
               "건너뜀 — 팔에 토크가 걸리므로 --pose 로 명시하십시오")
        return
    if not Path("/dev/omx_pack_follower").exists():
        record("시작 자세", WARN, "포장 팔이 없어 건너뜀")
        return
    print("        팔에 연결합니다 — 토크가 켜집니다. 주변을 비우십시오.")
    code, out = run([PACK, "-m", "omx_pack.dist", "--basket", "yellow",
                     "--robot-port", "/dev/omx_pack_follower"], timeout=60)
    if code != 0:
        record("시작 자세", WARN, f"확인 실패: {out.splitlines()[-1][:70] if out else ''}")
        return
    body = [l for l in out.splitlines() if l.strip()]
    outside = [l for l in body if "범위 밖" in l]
    if outside:
        record("시작 자세", WARN,
               "학습 범위 밖: " + "; ".join(l.split()[0] for l in outside))
        print("        (--strict-start 로 띄우면 /pack 이 400 으로 거절됩니다)")
    else:
        record("시작 자세", OK, "모든 관절이 학습 분포 안")


# ── 5. 실행 환경 ───────────────────────────────────────────────────────
def check_envs() -> None:
    print("\n■ 실행 환경")
    for py, name, want in ((IL, "픽업 venv (il)", "0.4.4"),
                           (PACK, "포장 venv (pack)", "0.6.1")):
        if not Path(py).exists():
            record(name, FAIL, f"없음: {py}")
            continue
        code, out = run([py, "-c",
                         "import lerobot,torch;print(lerobot.__version__,torch.__version__,"
                         "torch.cuda.is_available())"], timeout=90)
        if code != 0:
            record(name, FAIL, out.splitlines()[-1][:70] if out else "임포트 실패")
            continue
        ver, torch_v, cuda = out.split()[-3:]
        status = OK if ver == want and cuda == "True" else WARN
        record(name, status, f"lerobot {ver} · torch {torch_v} · CUDA {cuda}")


def check_checkpoints() -> None:
    print("\n■ 정책 체크포인트")
    ck = {
        "픽업 v1_yolo/060000": f"{OUT}/v1_yolo/checkpoints/060000/pretrained_model",
        "포장 YELLOW": f"{OUT}/my_act_20260819_125841-CART_YELLOW_MODEL"
                       f"/my_act_20260819_125841/checkpoints/last/pretrained_model",
        "포장 MINT": f"{OUT}/my_act_20260820_123027_CART_MINT_MODEL"
                     f"/my_act_20260820_123027/checkpoints/last/pretrained_model",
    }
    for name, path in ck.items():
        p = Path(path)
        if not (p / "model.safetensors").exists():
            record(name, FAIL, f"없음: {path}")
        else:
            mb = (p / "model.safetensors").stat().st_size / 1e6
            record(name, OK, f"{mb:.0f} MB")
    w = Path("/home/newuser/il_ws/models/omx_goods_yolo11n.pt")
    record("YOLO 검출기", OK if w.exists() else FAIL,
           f"{w.stat().st_size/1e6:.1f} MB" if w.exists() else "없음")


# ── 6. 서버 포트 ───────────────────────────────────────────────────────
def check_servers() -> None:
    print("\n■ 서버 (떠 있으면 health 확인)")
    import urllib.request

    for port, label in ((8080, "픽업 :8080"), (8081, "포장 :8081")):
        try:
            with urllib.request.urlopen(f"http://127.0.0.1:{port}/health",
                                        timeout=3) as r:
                h = json.loads(r.read().decode())
        except Exception:
            record(label, WARN, "아직 안 떠 있음 (시험 전에 띄울 것)")
            continue
        connected = bool(h.get("robotConnected"))
        detail = f"robotConnected={connected} · busy={h.get('busy')}"
        if h.get("basket"):
            detail += f" · 바구니 {h['basket']}"
        record(label, OK if connected else FAIL, detail)
        if h.get("status") == "DEGRADED":
            record(f"{label} 상태", WARN, str(h.get("message"))[:70])


# ── 7. 자원 ────────────────────────────────────────────────────────────
def check_resources() -> None:
    print("\n■ 자원")
    code, out = run(["nvidia-smi", "--query-gpu=name,memory.used,memory.total",
                     "--format=csv,noheader"], timeout=20)
    if code == 0 and out:
        record("GPU", OK, out.splitlines()[0])
    else:
        record("GPU", FAIL, "nvidia-smi 실패")
    total, used, free = shutil.disk_usage("/home")
    record("디스크 여유", OK if free > 20e9 else WARN, f"{free/1e9:.0f} GB")


CHECKS = {
    "devices": check_devices, "cameras": check_cameras, "arms": check_arms,
    "pose": check_start_pose, "envs": check_envs,
    "checkpoints": check_checkpoints, "servers": check_servers,
    "resources": check_resources,
}


def main() -> None:
    ap = argparse.ArgumentParser(description="통합 시험 전 리그 점검")
    ap.add_argument("--skip", nargs="*", default=[], choices=list(CHECKS))
    ap.add_argument("--only", nargs="*", default=[], choices=list(CHECKS))
    ap.add_argument("--pose", action="store_true",
                    help="포장 팔 시작 자세도 점검한다 (팔에 토크가 걸린다)")
    a = ap.parse_args()

    os.environ.setdefault("PYTHONPATH", SRC)
    os.environ.setdefault("YOLO_AUTOINSTALL", "false")

    print("═" * 62)
    print(" OMX 리그 사전 점검")
    print("═" * 62)
    for name, fn in CHECKS.items():
        if name in a.skip or (a.only and name not in a.only):
            continue
        try:
            fn(a.pose) if name == "pose" else fn()
        except Exception as exc:                      # noqa: BLE001
            record(name, FAIL, f"점검 자체가 실패: {exc}")

    n_fail = sum(1 for _, s, _ in results if s == FAIL)
    n_warn = sum(1 for _, s, _ in results if s == WARN)
    print("\n" + "═" * 62)
    if n_fail:
        print(f" 실패 {n_fail}건 · 경고 {n_warn}건 — 고치고 다시 돌리십시오.")
        sys.exit(2)
    if n_warn:
        print(f" 경고 {n_warn}건 — 내용을 보고 진행 여부를 판단하십시오.")
        sys.exit(1)
    print(f" 모두 통과 ({len(results)}항목). 시험을 시작해도 좋습니다.")


if __name__ == "__main__":
    main()
