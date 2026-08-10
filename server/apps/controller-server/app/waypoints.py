"""Map-frame waypoints for demo pick tours (map_test1)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable


def _yaw_from_quat(z: float, w: float) -> float:
    """Yaw from quaternion with x=y=0."""
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


@dataclass(frozen=True)
class Waypoint:
    id: str
    label: str
    x: float
    y: float
    yaw: float


def _wp(wid: str, label: str, x: float, y: float, z: float, w: float) -> Waypoint:
    return Waypoint(id=wid, label=label, x=x, y=y, yaw=_yaw_from_quat(z, w))


WAYPOINTS: dict[str, Waypoint] = {
    # 홈: map +x(오른쪽) 방향 — yaw = 0
    "S1": Waypoint(
        id="S1",
        label="cart-1 대기",
        x=0.036703343955750284,
        y=0.0005066978948139312,
        yaw=0.0,
    ),
    "S2": Waypoint(
        id="S2",
        label="cart-2 대기",
        x=0.038474577957370054,
        y=-0.1911947634013857,
        yaw=0.0,
    ),
    "W1": _wp(
        "W1",
        "케이크",
        0.036703343955750284,
        -1.377,
        -0.7384688405958438,
        0.6742876029329252,
    ),
    "W2": _wp(
        "W2",
        "롤케이크",
        0.036703343955750284,
        -1.828,
        -0.6965217746622735,
        0.7175356558536427,
    ),
    "W3": _wp(
        "W3",
        "샌드위치",
        0.34820253257338263,
        -2.3684238438867913,
        0.034845193901868764,
        0.9993927218375873,
    ),
    "W4": _wp(
        "W4",
        "아이스크림",
        0.8693162124281837,
        -2.3629413403536203,
        0.004420686500293974,
        0.999990228717694,
    ),
    "W5": _wp(
        "W5",
        "우유",
        0.9911321796121323,
        -1.513955691171541,
        0.7036022695483656,
        0.710594009464187,
    ),
    "W6": _wp(
        "W6",
        "콜라",
        1.0094719783231025,
        -0.9942170400336388,
        0.6881169938961149,
        0.7255997537977629,
    ),
    "C": _wp(
        "C",
        "계산대",
        0.873037381331733,
        -0.4628773988164313,
        0.7201343740279695,
        0.6938346224737885,
    ),
    "P": _wp(
        "P",
        "운송대기",
        0.04392549554456563,
        0.3766563561382236,
        0.9983007592473406,
        0.05827172630172554,
    ),
}

# product slug → shelf waypoint
SLUG_TO_WAYPOINT: dict[str, str] = {
    "cake": "W1",
    "roll-cake": "W2",
    "sandwich": "W3",
    "ice-cream": "W4",
    "milk": "W5",
    "cola": "W6",
}

DEVICE_HOME: dict[str, str] = {
    "cart-1": "S1",
    "cart-2": "S2",
}


def get_waypoint(wid: str) -> Waypoint:
    return WAYPOINTS[wid]


def home_for_device(device_code: str) -> Waypoint:
    code = (device_code or "cart-1").strip().lower()
    if code in ("cart-2", "cart2", "2"):
        code = "cart-2"
    elif code in ("cart-1", "cart1", "1"):
        code = "cart-1"
    wid = DEVICE_HOME.get(code, "S1")
    return WAYPOINTS[wid]


def waypoint_ids_for_slugs(slugs: Iterable[str]) -> list[str]:
    """Unique shelf waypoint ids for product slugs (stable first-seen order)."""
    seen: set[str] = set()
    out: list[str] = []
    for slug in slugs:
        wid = SLUG_TO_WAYPOINT.get(slug)
        if wid and wid not in seen:
            seen.add(wid)
            out.append(wid)
    return out


def nearest_neighbor_order(
    start: Waypoint,
    waypoint_ids: list[str],
) -> list[Waypoint]:
    """Greedy NN tour over shelf waypoints."""
    remaining = [WAYPOINTS[i] for i in waypoint_ids if i in WAYPOINTS]
    ordered: list[Waypoint] = []
    cx, cy = start.x, start.y
    while remaining:
        best_i = 0
        best_d = float("inf")
        for i, wp in enumerate(remaining):
            d = (wp.x - cx) ** 2 + (wp.y - cy) ** 2
            if d < best_d:
                best_d = d
                best_i = i
        nxt = remaining.pop(best_i)
        ordered.append(nxt)
        cx, cy = nxt.x, nxt.y
    return ordered
