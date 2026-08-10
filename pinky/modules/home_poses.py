"""카트별 대기장소(S1/S2) — controller waypoints.py 와 동일 좌표."""

from __future__ import annotations

import os
from typing import Tuple

# server/apps/controller-server/app/waypoints.py WAYPOINTS S1/S2 와 동기화할 것
HOME_POSES: dict[str, Tuple[float, float, float]] = {
    "cart-1": (0.036703343955750284, 0.0005066978948139312, 0.0),  # S1, yaw=0
    "cart-2": (0.038474577957370054, -0.1911947634013857, 0.0),  # S2, yaw=0
}


def home_pose_for_device(device_code: str | None = None) -> Tuple[float, float, float]:
    """
    우선순위:
      1) PINKY_INITIAL_POSE=x,y,yaw
      2) PINKY_DEVICE_CODE → S1/S2
    """
    raw = (os.environ.get("PINKY_INITIAL_POSE") or "").strip()
    if raw:
        parts = [p.strip() for p in raw.split(",")]
        if len(parts) >= 2:
            return (
                float(parts[0]),
                float(parts[1]),
                float(parts[2]) if len(parts) > 2 else 0.0,
            )

    code = (device_code or os.environ.get("PINKY_DEVICE_CODE") or "cart-1")
    code = code.strip().lower()
    if code in ("cart-2", "cart2", "2"):
        return HOME_POSES["cart-2"]
    return HOME_POSES["cart-1"]
