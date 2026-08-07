from __future__ import annotations

import logging
import os
import signal
import subprocess
import time
from pathlib import Path
from typing import Any

logger = logging.getLogger("pinky.pro_launch")


def _truthy(name: str, default: str = "0") -> bool:
    return os.environ.get(name, default).lower().strip() in (
        "1",
        "true",
        "on",
        "yes",
    )


def should_auto_launch_pro() -> bool:
    """
    pinky_pro bringup + navigation 을 subprocess 로 기동할지.
    PINKY_AUTO_LAUNCH=auto → ros2 백엔드일 때 on
    """
    flag = os.environ.get("PINKY_AUTO_LAUNCH", "auto").lower().strip()
    if flag in ("0", "false", "off", "no"):
        return False
    if flag in ("1", "true", "on", "yes"):
        return True
    # auto
    return os.environ.get("PINKY_BACKEND", "mock").lower().strip() == "ros2"


def defer_lidar_to_pro() -> bool:
    """라이다는 pinky_pro sllidar 에 맡김 (기본: auto_launch 켜져 있으면 on)."""
    flag = os.environ.get("PINKY_DEFER_LIDAR", "auto").lower().strip()
    if flag in ("0", "false", "off", "no"):
        return False
    if flag in ("1", "true", "on", "yes"):
        return True
    return should_auto_launch_pro()


def defer_battery_to_pro() -> bool:
    """배터리는 pinky_pro battery_publisher 에 맡김."""
    flag = os.environ.get("PINKY_DEFER_BATTERY", "auto").lower().strip()
    if flag in ("0", "false", "off", "no"):
        return False
    if flag in ("1", "true", "on", "yes"):
        return True
    return should_auto_launch_pro()


def resolve_map_yaml() -> Path:
    mid = os.environ.get("PINKY_MAP", "map_test1").strip()
    root = Path(__file__).resolve().parents[1]
    for base in (Path.cwd(), root):
        y = base / f"{mid}.yaml" if not mid.endswith(".yaml") else base / mid
        if y.is_file():
            return y.resolve()
        y2 = Path(mid)
        if y2.is_file():
            return y2.resolve()
    # fallback path even if missing (launch will error clearly)
    return (root / f"{mid}.yaml").resolve()


def _pro_root() -> Path:
    env = os.environ.get("PINKY_PRO_ROOT", "").strip()
    if env:
        return Path(env).resolve()
    # PS_project/pinky → ../pinky_pro
    return Path(__file__).resolve().parents[2] / "pinky_pro"


class ProStackLauncher:
    """
    pinky_pro ROS2 launch 를 서브프로세스로 기동/종료.
      1) pinky_bringup bringup_robot.launch.xml  (모터·오돔·sllidar·battery)
      2) pinky_navigation bringup_launch.xml     (map_server·AMCL·Nav2)
    """

    def __init__(self) -> None:
        self._procs: list[subprocess.Popen] = []
        self._started = False

    @property
    def started(self) -> bool:
        return self._started

    def start(self) -> None:
        if self._started:
            return
        if not should_auto_launch_pro():
            logger.info("PINKY_AUTO_LAUNCH off — skip pinky_pro launches")
            return

        map_yaml = resolve_map_yaml()
        logger.info("auto-launch pinky_pro | map=%s", map_yaml)

        # 라이다/배터리 충돌 방지 기본값 (이미 설정된 env 는 유지)
        if "PINKY_DEFER_LIDAR" not in os.environ:
            os.environ["PINKY_DEFER_LIDAR"] = "1"
        if "PINKY_DEFER_BATTERY" not in os.environ:
            os.environ["PINKY_DEFER_BATTERY"] = "1"
        # pinky LidarReader 가 sllidar 를 또 띄우지 않도록
        os.environ["PINKY_LIDAR_SLLIDAR"] = "0"

        env = os.environ.copy()
        # Source overlay if present so `ros2 launch pinky_bringup` resolves
        pro = _pro_root()
        setup = pro / "install" / "setup.bash"
        log_dir = Path(
            os.environ.get("PINKY_PRO_LOG_DIR", str(Path.home() / "pinky_logs"))
        )
        try:
            log_dir.mkdir(parents=True, exist_ok=True)
        except Exception:
            log_dir = Path("/tmp")

        cmds: list[tuple[str, list[str]]] = [
            (
                "bringup",
                [
                    "ros2",
                    "launch",
                    "pinky_bringup",
                    "bringup_robot.launch.xml",
                ],
            ),
            (
                "nav",
                [
                    "ros2",
                    "launch",
                    "pinky_navigation",
                    "bringup_launch.xml",
                    f"map:={map_yaml}",
                ],
            ),
        ]

        for name, cmd in cmds:
            logger.info("spawn: %s", " ".join(cmd))
            log_path = log_dir / f"pinky_pro_{name}.log"
            try:
                log_f = open(log_path, "ab", buffering=0)
            except Exception:
                log_f = subprocess.DEVNULL

            # Prefer bash -lc with sourced workspace when install/ exists
            if setup.is_file():
                shell_cmd = (
                    f"set -e; source /opt/ros/${{ROS_DISTRO:-jazzy}}/setup.bash "
                    f"2>/dev/null || true; source '{setup}'; "
                    + " ".join(f"'{c}'" for c in cmd)
                )
                popen_cmd: list[str] = ["bash", "-lc", shell_cmd]
            else:
                popen_cmd = cmd

            try:
                proc = subprocess.Popen(
                    popen_cmd,
                    env=env,
                    stdout=log_f if log_f is not subprocess.DEVNULL else subprocess.DEVNULL,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                self._procs.append(proc)
                logger.info("spawned %s pid=%s log=%s", name, proc.pid, log_path)
            except FileNotFoundError:
                logger.error(
                    "ros2 not found — source ROS2 / workspace before run.py"
                )
                self.stop()
                return
            except Exception as exc:
                logger.exception("failed to spawn %s: %s", cmd, exc)
                self.stop()
                return

        # TF / scan / odom 준비 대기 (라이다 스핀업에 여유)
        wait_s = float(os.environ.get("PINKY_PRO_LAUNCH_WAIT", "8.0"))
        time.sleep(max(0.5, wait_s))

        for i, proc in enumerate(list(self._procs)):
            code = proc.poll()
            if code is not None:
                logger.error(
                    "pinky_pro launch exited early index=%s code=%s — see ~/pinky_logs/",
                    i,
                    code,
                )

        self._started = True
        logger.info(
            "pinky_pro launches started (%d procs, wait=%.1fs)",
            len(self._procs),
            wait_s,
        )

    def stop(self) -> None:
        if not self._procs:
            self._started = False
            return
        logger.info("stopping pinky_pro launch subprocesses...")
        for proc in self._procs:
            try:
                if proc.poll() is None:
                    os.killpg(proc.pid, signal.SIGTERM)
            except (ProcessLookupError, PermissionError, OSError):
                try:
                    proc.terminate()
                except Exception:
                    pass
        deadline = time.time() + 5.0
        for proc in self._procs:
            remaining = max(0.1, deadline - time.time())
            try:
                proc.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except (ProcessLookupError, PermissionError, OSError):
                    try:
                        proc.kill()
                    except Exception:
                        pass
        self._procs.clear()
        self._started = False
        logger.info("pinky_pro launches stopped")


_shared: ProStackLauncher | None = None


def get_pro_launcher() -> ProStackLauncher:
    global _shared
    if _shared is None:
        _shared = ProStackLauncher()
    return _shared
