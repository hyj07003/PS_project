#!/usr/bin/env python3
from __future__ import annotations

import logging
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from server import create_app
from server.config import (
    get_backend,
    get_controller_url,
    get_device_code,
    get_host,
    get_port,
    load_env,
    should_start_sensor_publisher,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)


def main() -> None:
    loaded = load_env()
    backend = get_backend()
    publish = should_start_sensor_publisher()
    env_note = ", ".join(loaded) if loaded else "(none — defaults / process env only)"

    from controllers.pro_launch import (
        defer_battery_to_pro,
        defer_lidar_to_pro,
        resolve_map_yaml,
        should_auto_launch_pro,
    )

    print(
        f"[pinky] env_files={env_note}\n"
        f"[pinky] backend={backend} sensor_publisher={'on' if publish else 'off'} "
        f"device={get_device_code()} controller={get_controller_url()}\n"
        f"[pinky] host={get_host()} port={get_port()}\n"
        f"[pinky] auto_launch_pro={should_auto_launch_pro()} "
        f"defer_lidar={defer_lidar_to_pro()} defer_battery={defer_battery_to_pro()}\n"
        f"[pinky] map_yaml={resolve_map_yaml()}"
    )
    if backend == "mock":
        print(
            "[pinky] WARNING: PINKY_BACKEND=mock — 모니터링에 더미 센서가 표시됩니다.\n"
            "         실기에서는 pinky.env 에 PINKY_BACKEND=ros2 를 넣고 "
            "~/pinky 에서 재시작하세요."
        )
    app = create_app()

    # emotion_server가 happy로 뜬 뒤에도 charging으로 덮어씀 (pinky_pro 수정 없이)
    try:
        from server import _apply_boot_lcd

        robot = app.extensions.get("robot")
        if robot is not None:
            _apply_boot_lcd(robot, backend, background=True)
            print("[pinky] boot LCD → pinky_charging (async)")
    except Exception as exc:
        print(f"[pinky] boot LCD schedule failed: {exc}")

    def _handle_stop(signum, _frame) -> None:
        print(f"[pinky] signal {signum} — stopping Nav2/bringup leftover processes")
        emotion = app.extensions.get("emotion_launcher")
        if emotion is not None:
            try:
                emotion.stop()
            except Exception as exc:
                print(f"[pinky] emotion_launcher.stop failed: {exc}")
        pro = app.extensions.get("pro_launcher")
        if pro is not None:
            try:
                pro.stop()
            except Exception as exc:
                print(f"[pinky] pro_launcher.stop failed: {exc}")
        raise SystemExit(0)

    import signal as _signal

    _signal.signal(_signal.SIGINT, _handle_stop)
    _signal.signal(_signal.SIGTERM, _handle_stop)
    if hasattr(_signal, "SIGHUP"):
        _signal.signal(_signal.SIGHUP, _handle_stop)

    try:
        nav = app.extensions["robot"].navigation.map_info()
        if nav:
            print(
                f"[pinky] map={nav.get('mapId')} "
                f"{nav.get('width')}x{nav.get('height')} "
                f"res={nav.get('resolution')}"
            )
        else:
            print("[pinky] WARNING: map meta not loaded")
        pro = app.extensions.get("pro_launcher")
        if pro and getattr(pro, "started", False):
            print("[pinky] pinky_pro bringup+nav launch: running (subprocess)")
        elif backend == "ros2" and not should_auto_launch_pro():
            print(
                "[pinky] tip: set PINKY_AUTO_LAUNCH=1 to spawn "
                "pinky_bringup + pinky_navigation from run.py"
            )
        emotion = app.extensions.get("emotion_launcher")
        if emotion and getattr(emotion, "started", False):
            print("[pinky] emotion_server (LCD): running")
        elif backend == "ros2":
            reason = getattr(emotion, "skipped_reason", None) if emotion else "unavailable"
            print(f"[pinky] WARNING: emotion_server (LCD) not running ({reason})")
    except Exception as exc:
        print(f"[pinky] map/pro status warn: {exc}")
    app.run(host=get_host(), port=get_port(), debug=False, threaded=True, use_reloader=False)


if __name__ == "__main__":
    main()
