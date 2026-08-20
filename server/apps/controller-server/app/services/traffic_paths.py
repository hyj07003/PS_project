"""Path geometry helpers for multi-robot traffic coordination."""

from __future__ import annotations

import math
from typing import Any


def path_points(path_payload: dict[str, Any] | None) -> list[tuple[float, float]]:
    """Extract [(x, y), ...] from Pinky /nav/plan or /nav/path responses."""
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


def distance_to_point(
    pose: dict[str, float] | None,
    point: tuple[float, float],
) -> float:
    if not isinstance(pose, dict):
        return math.inf
    try:
        return math.hypot(float(pose["x"]) - point[0], float(pose["y"]) - point[1])
    except (KeyError, TypeError, ValueError):
        return math.inf


def path_heading(path: list[tuple[float, float]], index: int) -> float:
    if not path:
        return 0.0
    idx = max(0, min(int(index), len(path) - 1))
    if idx >= len(path) - 1:
        if len(path) < 2:
            return 0.0
        x0, y0 = path[-2]
        x1, y1 = path[-1]
    else:
        x0, y0 = path[idx]
        x1, y1 = path[idx + 1]
    return math.atan2(y1 - y0, x1 - x0)


def path_distance_between(
    path: list[tuple[float, float]],
    start_index: int,
    end_index: int,
) -> float:
    low, high = sorted((int(start_index), int(end_index)))
    if high >= len(path) or low < 0:
        return 0.0
    total = 0.0
    for index in range(low, high):
        if index + 1 >= len(path):
            break
        total += math.hypot(
            path[index + 1][0] - path[index][0],
            path[index + 1][1] - path[index][1],
        )
    return total


def retreat_path_index(
    path: list[tuple[float, float]],
    entry_index: int,
    hold_margin: float,
) -> int:
    """Walk backward from entry_index until hold_margin is covered."""
    if not path:
        return 0
    idx = max(0, min(int(entry_index), len(path) - 1))
    remaining = max(0.0, float(hold_margin))
    while idx > 0 and remaining > 1e-6:
        step = math.hypot(path[idx][0] - path[idx - 1][0], path[idx][1] - path[idx - 1][1])
        if step >= remaining:
            return idx - 1
        remaining -= step
        idx -= 1
    return 0


def advance_path_index(
    path: list[tuple[float, float]],
    exit_index: int,
    release_margin: float,
) -> int:
    """Walk forward from exit_index until release_margin is covered."""
    if not path:
        return 0
    idx = max(0, min(int(exit_index), len(path) - 1))
    remaining = max(0.0, float(release_margin))
    while idx < len(path) - 1 and remaining > 1e-6:
        step = math.hypot(path[idx + 1][0] - path[idx][0], path[idx + 1][1] - path[idx][1])
        if step >= remaining:
            return idx + 1
        remaining -= step
        idx += 1
    return len(path) - 1


def nearest_path_index(
    path: list[tuple[float, float]],
    pose: dict[str, float] | None,
) -> int:
    """Index of the closest path vertex to pose (for progress along a route)."""
    if pose is None:
        return 0
    return _closest_path_index(path, pose)


def densify_path(
    path: list[tuple[float, float]],
    step_m: float = 0.05,
) -> list[tuple[float, float]]:
    """Insert points along segments so sparse Nav2 / fallback paths still conflict-check."""
    if len(path) < 2:
        return list(path)
    step = max(0.01, float(step_m))
    out: list[tuple[float, float]] = [path[0]]
    for i in range(len(path) - 1):
        x0, y0 = path[i]
        x1, y1 = path[i + 1]
        seg_len = math.hypot(x1 - x0, y1 - y0)
        if seg_len < 1e-9:
            continue
        n_steps = max(1, int(math.ceil(seg_len / step)))
        for k in range(1, n_steps + 1):
            t = min(1.0, k / n_steps)
            out.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
    return out


def point_to_segment_distance(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    """Shortest distance from point to a finite segment."""
    px, py = point
    return math.sqrt(
        _dist_point_to_segment_sq(px, py, start[0], start[1], end[0], end[1])
    )


def _append_pair(
    pairs: list[tuple[int, int]],
    samples: list[dict[str, float]],
    i: int,
    j: int,
    x1: float,
    y1: float,
    x2: float,
    y2: float,
    threshold2: float,
    max_samples: int,
) -> None:
    ddx = x1 - x2
    ddy = y1 - y2
    d2 = ddx * ddx + ddy * ddy
    if d2 <= threshold2:
        pairs.append((i, j))
        if len(samples) < max_samples:
            samples.append(
                {
                    "x": (x1 + x2) / 2.0,
                    "y": (y1 + y2) / 2.0,
                    "distanceM": math.sqrt(d2),
                }
            )


def _segment_point_pairs(
    path_a: list[tuple[float, float]],
    path_b: list[tuple[float, float]],
    clearance: float,
    max_samples: int,
    samples: list[dict[str, float]],
) -> list[tuple[int, int]]:
    """Point-on-A vs segment-on-B proximity pairs (complements point-point grid)."""
    threshold2 = clearance * clearance
    pairs: list[tuple[int, int]] = []
    if len(path_b) < 2:
        return pairs
    for i, (x1, y1) in enumerate(path_a):
        for j in range(len(path_b) - 1):
            x0, y0 = path_b[j]
            x2, y2 = path_b[j + 1]
            if point_to_segment_distance((x1, y1), (x0, y0), (x2, y2)) <= clearance:
                d0 = math.hypot(x1 - x0, y1 - y0)
                d2 = math.hypot(x1 - x2, y1 - y2)
                closest = j if d0 <= d2 else j + 1
                bx, by = path_b[closest]
                _append_pair(
                    pairs, samples, i, closest, x1, y1, bx, by, threshold2, max_samples
                )
                if len(pairs) >= max_samples * 4:
                    return pairs
    return pairs


def _closest_path_index(
    path: list[tuple[float, float]],
    pose: dict[str, float],
) -> int:
    try:
        px = float(pose["x"])
        py = float(pose["y"])
    except (KeyError, TypeError, ValueError):
        return 0
    if not path:
        return 0
    best_index = 0
    best_dist = math.inf
    for index, (x, y) in enumerate(path):
        dist = math.hypot(px - x, py - y)
        if dist < best_dist:
            best_dist = dist
            best_index = index
    return best_index


def has_passed_path_index(
    path: list[tuple[float, float]],
    pose: dict[str, float] | None,
    release_index: int,
) -> bool:
    """True when pose has progressed to or beyond release_index along path."""
    if not path or pose is None:
        return False
    return _closest_path_index(path, pose) >= int(release_index)


def _dist_point_to_segment_sq(
    px: float,
    py: float,
    x0: float,
    y0: float,
    x1: float,
    y1: float,
) -> float:
    dx = x1 - x0
    dy = y1 - y0
    if abs(dx) < 1e-12 and abs(dy) < 1e-12:
        ddx = px - x0
        ddy = py - y0
        return ddx * ddx + ddy * ddy
    t = max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / (dx * dx + dy * dy)))
    proj_x = x0 + t * dx
    proj_y = y0 + t * dy
    ddx = px - proj_x
    ddy = py - proj_y
    return ddx * ddx + ddy * ddy


def path_intersects_disc(
    path: list[tuple[float, float]],
    cx: float,
    cy: float,
    radius_m: float,
) -> bool:
    """True if any path segment passes within radius_m of disc center."""
    if not path:
        return False
    r2 = max(0.0, float(radius_m)) ** 2
    if r2 <= 0.0:
        return False
    if len(path) == 1:
        px, py = path[0]
        ddx = px - cx
        ddy = py - cy
        return ddx * ddx + ddy * ddy <= r2
    for i in range(len(path) - 1):
        x0, y0 = path[i]
        x1, y1 = path[i + 1]
        if _dist_point_to_segment_sq(cx, cy, x0, y0, x1, y1) <= r2:
            return True
    return False


def path_intersects_zones(
    path: list[tuple[float, float]],
    zones: list[tuple[float, float, float]],
) -> bool:
    """True if path intersects any (cx, cy, radius_m) zone."""
    for cx, cy, radius_m in zones:
        if path_intersects_disc(path, cx, cy, radius_m):
            return True
    return False


def find_path_conflicts(
    path1: list[tuple[float, float]],
    path2: list[tuple[float, float]],
    clearance_m: float = 0.30,
    max_samples: int = 20,
) -> dict[str, Any]:
    """Conservative path-overlap test with merged conflict segments."""
    clearance = max(0.001, float(clearance_m))
    if not path1 or not path2:
        return {
            "conflict": False,
            "clearanceM": clearance,
            "minDistanceM": None,
            "samples": [],
            "segments": [],
        }

    dense1 = densify_path(path1, step_m=min(clearance * 0.5, 0.05))
    dense2 = densify_path(path2, step_m=min(clearance * 0.5, 0.05))

    cell = clearance
    grid: dict[tuple[int, int], list[tuple[int, float, float]]] = {}
    for j, (x, y) in enumerate(dense2):
        key = (math.floor(x / cell), math.floor(y / cell))
        grid.setdefault(key, []).append((j, x, y))

    threshold2 = clearance * clearance
    min_d2 = math.inf
    samples: list[dict[str, float]] = []
    pairs: list[tuple[int, int]] = []

    for i, (x1, y1) in enumerate(dense1):
        cx, cy = math.floor(x1 / cell), math.floor(y1 / cell)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for j, x2, y2 in grid.get((cx + dx, cy + dy), []):
                    ddx = x1 - x2
                    ddy = y1 - y2
                    d2 = ddx * ddx + ddy * ddy
                    if d2 < min_d2:
                        min_d2 = d2
                    if d2 <= threshold2:
                        pairs.append((i, j))
                        if len(samples) < max_samples:
                            samples.append(
                                {
                                    "x": (x1 + x2) / 2.0,
                                    "y": (y1 + y2) / 2.0,
                                    "distanceM": math.sqrt(d2),
                                }
                            )

    seg_pairs_a = _segment_point_pairs(
        dense1, dense2, clearance, max_samples, samples
    )
    seg_pairs_b = _segment_point_pairs(
        dense2, dense1, clearance, max_samples, samples
    )
    for i, j in seg_pairs_a:
        pairs.append((i, j))
    for i, j in seg_pairs_b:
        pairs.append((j, i))

    segments = _merge_conflict_segments(dense1, dense2, pairs)
    return {
        "conflict": bool(segments),
        "clearanceM": clearance,
        "minDistanceM": None if math.isinf(min_d2) else math.sqrt(min_d2),
        "samples": samples,
        "segments": segments,
    }


def _merge_conflict_segments(
    path1: list[tuple[float, float]],
    path2: list[tuple[float, float]],
    pairs: list[tuple[int, int]],
) -> list[dict[str, Any]]:
    if not pairs:
        return []
    pairs.sort(key=lambda item: (item[0], item[1]))
    segments: list[dict[str, Any]] = []
    cur_i0 = cur_i1 = pairs[0][0]
    cur_j0 = cur_j1 = pairs[0][1]
    for i, j in pairs[1:]:
        if i <= cur_i1 + 1:
            cur_i1 = max(cur_i1, i)
            cur_j0 = min(cur_j0, j)
            cur_j1 = max(cur_j1, j)
            continue
        segments.append(_segment_dict(path1, path2, cur_i0, cur_i1, cur_j0, cur_j1))
        cur_i0 = cur_i1 = i
        cur_j0 = cur_j1 = j
    segments.append(_segment_dict(path1, path2, cur_i0, cur_i1, cur_j0, cur_j1))
    return segments


def _segment_dict(
    path1: list[tuple[float, float]],
    path2: list[tuple[float, float]],
    i0: int,
    i1: int,
    j0: int,
    j1: int,
) -> dict[str, Any]:
    if not path1 or not path2:
        return {
            "entry": {"x": 0.0, "y": 0.0},
            "exit": {"x": 0.0, "y": 0.0},
            "cart1_start_index": 0,
            "cart1_end_index": 0,
            "cart2_start_index": 0,
            "cart2_end_index": 0,
        }
    i0 = max(0, min(int(i0), len(path1) - 1))
    i1 = max(0, min(int(i1), len(path1) - 1))
    j0 = max(0, min(int(j0), len(path2) - 1))
    j1 = max(0, min(int(j1), len(path2) - 1))
    entry = {"x": path1[i0][0], "y": path1[i0][1]}
    exit_point = {"x": path1[i1][0], "y": path1[i1][1]}
    return {
        "entry": entry,
        "exit": exit_point,
        "cart1_start_index": int(i0),
        "cart1_end_index": int(i1),
        "cart2_start_index": int(j0),
        "cart2_end_index": int(j1),
    }
