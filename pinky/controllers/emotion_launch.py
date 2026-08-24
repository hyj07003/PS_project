"""LCD emotion_server subprocess launcher (pinky_emotion)."""

from __future__ import annotations

import logging
import os
import signal
import subprocess
import time
from pathlib import Path

logger = logging.getLogger("pinky.emotion_launch")


def _truthy(name: str, default: str = "0") -> bool:
    return os.environ.get(name, default).lower().strip() in (
        "1",
        "true",
        "on",
        "yes",
    )


def should_auto_launch_emotion() -> bool:
    """PINKY_AUTO_EMOTION=auto → ros2 백엔드일 때 on."""
    flag = os.environ.get("PINKY_AUTO_EMOTION", "auto").lower().strip()
    if flag in ("0", "false", "off", "no"):
        return False
    if flag in ("1", "true", "on", "yes"):
        return True
    return os.environ.get("PINKY_BACKEND", "mock").lower().strip() == "ros2"


def _pro_root() -> Path:
    env = os.environ.get("PINKY_PRO_ROOT", "").strip()
    if env:
        return Path(env).resolve()
    # PS_project/pinky → ../pinky_pro
    return Path(__file__).resolve().parents[2] / "pinky_pro"


def _pinky_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _ensure_emotion_dir_env() -> None:
    if os.environ.get("PINKY_EMOTION_DIR", "").strip():
        return
    emotion_dir = _pro_root() / "pinky_emotion" / "emotion"
    if emotion_dir.is_dir():
        os.environ["PINKY_EMOTION_DIR"] = str(emotion_dir)
        logger.info("PINKY_EMOTION_DIR=%s", emotion_dir)


def _emotion_service_ready(timeout_sec: float = 1.5) -> bool:
    """True if /set_emotion already advertised."""
    try:
        out = subprocess.check_output(
            ["ros2", "service", "list"],
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=max(1.0, timeout_sec),
        )
    except Exception:
        return False
    for line in out.splitlines():
        name = line.strip()
        if name.endswith("/set_emotion") or name == "/set_emotion" or name == "set_emotion":
            return True
    return False


def _install_setup() -> Path | None:
    pro = _pro_root()
    for name in ("setup.bash", "local_setup.bash"):
        p = pro / "install" / name
        if p.is_file():
            return p
    return None


class EmotionServerLauncher:
    """Spawn ``ros2 run pinky_emotion emotion_server`` for LCD GIFs."""

    def __init__(self) -> None:
        self._proc: subprocess.Popen | None = None
        self._log_f = None
        self.started = False
        self.skipped_reason: str | None = None

    def start(self) -> None:
        if not should_auto_launch_emotion():
            self.skipped_reason = "PINKY_AUTO_EMOTION off"
            logger.info("emotion_server auto-launch disabled")
            return

        _ensure_emotion_dir_env()

        if _emotion_service_ready():
            self.skipped_reason = "set_emotion already available"
            self.started = True
            logger.info("emotion_server already running — skip spawn")
            return

        setup = _install_setup()
        log_dir = Path(
            os.environ.get("PINKY_LOG_DIR", str(_pinky_root() / "logs"))
        )
        try:
            log_dir.mkdir(parents=True, exist_ok=True)
        except Exception:
            log_dir = _pinky_root()
        log_path = log_dir / "emotion_server.log"

        env = os.environ.copy()
        if setup is not None:
            shell_cmd = (
                "set -e; "
                "source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash 2>/dev/null || "
                "source /opt/ros/humble/setup.bash 2>/dev/null || true; "
                f"source '{setup}'; "
                "exec ros2 run pinky_emotion emotion_server"
            )
            popen_cmd = ["bash", "-lc", shell_cmd]
        else:
            # Still try — may work if user already sourced the workspace.
            popen_cmd = ["ros2", "run", "pinky_emotion", "emotion_server"]
            logger.warning(
                "pinky_pro install setup.bash 없음 — ros2 run 직접 시도 "
                "(colcon build --packages-select pinky_emotion 권장)"
            )

        try:
            self._log_f = open(log_path, "ab", buffering=0)
        except Exception:
            self._log_f = subprocess.DEVNULL

        try:
            self._proc = subprocess.Popen(
                popen_cmd,
                env=env,
                stdout=self._log_f,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except FileNotFoundError:
            self.skipped_reason = "ros2 not found"
            logger.error("ros2 not found — cannot start emotion_server")
            self._close_log()
            return
        except Exception as exc:
            self.skipped_reason = str(exc)
            logger.exception("failed to spawn emotion_server: %s", exc)
            self._close_log()
            return

        wait_s = float(os.environ.get("PINKY_EMOTION_LAUNCH_WAIT", "2.0"))
        time.sleep(max(0.3, wait_s))
        if self._proc.poll() is not None:
            self.skipped_reason = f"exited early code={self._proc.returncode}"
            logger.error(
                "emotion_server exited early code=%s — see %s",
                self._proc.returncode,
                log_path,
            )
            self._proc = None
            self._close_log()
            return

        self.started = True
        logger.info(
            "emotion_server started pid=%s log=%s",
            self._proc.pid,
            log_path,
        )

    def stop(self) -> None:
        proc = self._proc
        self._proc = None
        if proc is None:
            self.started = False
            return
        logger.info("stopping emotion_server pid=%s", proc.pid)
        try:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGTERM)
        except (ProcessLookupError, PermissionError, OSError):
            try:
                proc.terminate()
            except Exception:
                pass
        try:
            proc.wait(timeout=4.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError, OSError):
                try:
                    proc.kill()
                except Exception:
                    pass
        self._close_log()
        self.started = False

    def _close_log(self) -> None:
        log_f = self._log_f
        self._log_f = None
        if log_f is not None and log_f is not subprocess.DEVNULL:
            try:
                log_f.close()
            except Exception:
                pass


_shared: EmotionServerLauncher | None = None


def get_emotion_launcher() -> EmotionServerLauncher:
    global _shared
    if _shared is None:
        _shared = EmotionServerLauncher()
    return _shared
