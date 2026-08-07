"""Recover lidar when pinky_pro sllidar advertises /scan but never publishes."""

from __future__ import annotations

import logging
import os
import subprocess
import time
from typing import Any

logger = logging.getLogger("pinky.lidar_recovery")


def _truthy(name: str, default: str = "1") -> bool:
    return os.environ.get(name, default).lower().strip() in (
        "1",
        "true",
        "on",
        "yes",
        "auto",
    )


def _kill_stale_sllidar() -> None:
    for pattern in ("sllidar", "sllidar_node"):
        try:
            subprocess.run(
                ["pkill", "-f", pattern],
                capture_output=True,
                timeout=3,
            )
        except Exception:
            pass
    time.sleep(0.5)


def _scan_has_data(robot: Any, wait_sec: float) -> bool:
    deadline = time.time() + max(0.5, wait_sec)
    while time.time() < deadline:
        try:
            data = robot.lidar.read()
            d = data.to_dict() if hasattr(data, "to_dict") else data
            if isinstance(d, dict):
                n = int(d.get("rangesCount") or 0)
                pts = d.get("points") or []
                if n > 0 or len(pts) > 0:
                    return True
            # raw backend cache
            backend = getattr(robot, "_backend", None)
            if backend is not None:
                scan = getattr(backend, "_scan", None)
                if scan is not None and getattr(scan, "ranges", None):
                    if any(r and r > 0 for r in scan.ranges[:50]):
                        return True
        except Exception:
            pass
        time.sleep(0.4)
    return False


def ensure_lidar(robot: Any) -> None:
    """
    If deferred sllidar produces no ranges, free the serial port and start
    LidarReader (rplidarc1 / serial / local sllidar).
    """
    if not _truthy("PINKY_LIDAR_RECOVERY", "1"):
        return
    if os.environ.get("PINKY_BACKEND", "mock").lower().strip() != "ros2":
        return

    wait_s = float(os.environ.get("PINKY_LIDAR_RECOVERY_WAIT", "12"))
    logger.info(
        "lidar recovery: waiting up to %.1fs for /scan data...", wait_s
    )
    if _scan_has_data(robot, wait_s):
        logger.info("lidar recovery: /scan data OK — no action")
        return

    logger.warning(
        "lidar recovery: no scan ranges — freeing port and starting LidarReader"
    )
    _kill_stale_sllidar()

    # Allow local reader + optional local sllidar launch
    os.environ["PINKY_DEFER_LIDAR"] = "0"
    if os.environ.get("PINKY_LIDAR_SLLIDAR", "0").lower().strip() in (
        "0",
        "false",
        "off",
        "no",
    ):
        os.environ["PINKY_LIDAR_SLLIDAR"] = "auto"

    try:
        from controllers.lidar import LidarReader

        reader = LidarReader.shared()
        reader.start()
        time.sleep(2.0)
        if _scan_has_data(robot, 8.0):
            logger.info(
                "lidar recovery: LidarReader OK | %s",
                "; ".join(reader.status) or "started",
            )
        else:
            logger.error(
                "lidar recovery: still no data | %s | "
                "check /dev/ttyAMA0, baud 460800, dialout, cable/power",
                "; ".join(reader.status) or "(no status)",
            )
    except Exception:
        logger.exception("lidar recovery failed to start LidarReader")
