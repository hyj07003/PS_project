from __future__ import annotations

import math
from typing import Any


def path_points(path_payload: dict[str, Any] | None) -> list[tuple[float, float]]:
    """Extract [(x, y), ...] from the Pinky /nav/path or /nav/plan response."""
    if not isinstance(path_payload, dict):
        return []
    path = path_payload.get("path") if "path" in path_payload else path_payload
    if not isinstance(path, dict):
        return []
    out: list[tuple[float, float]] = []
    for pose in path.get("poses") or []:
        if not isinstance(pose, dict):
            continue
        try:
            out.append((float(pose["x"]), float(pose["y"])))
        except (KeyError, TypeError, ValueError):
            continue
    return out


def find_path_conflicts(
    path1: list[tuple[float, float]],
    path2: list[tuple[float, float]],
    clearance_m: float = 0.30,
    max_samples: int = 20,
) -> dict[str, Any]:
    """
    Conservative path-overlap test for the first FMS scenario.

    Two path points closer than clearance_m are treated as the same reserved corridor.
    This is intentionally simple for the initial two-robot test; later this can be
    replaced by segment/corridor/time-window reservation.
    """
    clearance = max(0.001, float(clearance_m))
    if not path1 or not path2:
        return {
            "conflict": False,
            "clearanceM": clearance,
            "minDistanceM": None,
            "samples": [],
        }

    # Spatial hash: O(N)ish instead of comparing every pair for long Nav2 paths.
    cell = clearance
    grid: dict[tuple[int, int], list[tuple[float, float]]] = {}
    for x, y in path2:
        key = (math.floor(x / cell), math.floor(y / cell))
        grid.setdefault(key, []).append((x, y))

    threshold2 = clearance * clearance
    min_d2 = math.inf
    samples: list[dict[str, float]] = []

    for x1, y1 in path1:
        cx, cy = math.floor(x1 / cell), math.floor(y1 / cell)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for x2, y2 in grid.get((cx + dx, cy + dy), []):
                    ddx = x1 - x2
                    ddy = y1 - y2
                    d2 = ddx * ddx + ddy * ddy
                    if d2 < min_d2:
                        min_d2 = d2
                    if d2 <= threshold2 and len(samples) < max_samples:
                        samples.append(
                            {
                                "x": (x1 + x2) / 2.0,
                                "y": (y1 + y2) / 2.0,
                                "distanceM": math.sqrt(d2),
                            }
                        )

    return {
        "conflict": bool(samples),
        "clearanceM": clearance,
        "minDistanceM": None if math.isinf(min_d2) else math.sqrt(min_d2),
        "samples": samples,
    }
