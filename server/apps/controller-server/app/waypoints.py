"""Map-frame waypoints for demo pick tours (map_test1)."""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class Waypoint:
    id: str
    label: str
    x: float
    y: float
    yaw: float


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
    "W1": Waypoint(
        id="W1",
        label="케이크",
        x=0.036703343955750284,
        y=-1.377,
        yaw=-1.66159348963124,
    ),
    "W2": Waypoint(
        id="W2",
        label="롤케이크",
        x=0.036703343955750284,
        y=-1.828,
        yaw=-1.54107711732223,
    ),
    "W3": Waypoint(
        id="W3",
        label="샌드위치",
        x=0.34820253257338263,
        y=-2.3684238438867913,
        yaw=0.0697044983816298,
    ),
    "W4": Waypoint(
        id="W4",
        label="아이스크림",
        x=0.8693162124281837,
        y=-2.3629413403536203,
        yaw=0.00884140179788436,
    ),
    "W5": Waypoint(
        id="W5",
        label="우유",
        x=0.9911321796121323,
        y=-1.513955691171541,
        yaw=0.0,
    ),
    "W6": Waypoint(
        id="W6",
        label="콜라",
        x=1.0094719783231025,
        y=-0.9942170400336388,
        yaw=0.0,
    ),
    "C": Waypoint(
        id="C",
        label="계산대",
        x=0.873037381331733,
        y=-0.4628773988164313,
        yaw=1.6079919362854,
    ),
    "P": Waypoint(
        id="P",
        label="운송대기",
        x=0.04392549554456563,
        y=0.3766563561382236,
        yaw=3.02498314429096,
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


_DEFAULT_ARUCO_IDS = "W1:1,W2:2,W3:3,W4:4,W5:5,W6:6,C:10,P:11"


def parse_aruco_marker_map(raw: str | None = None) -> dict[str, int]:
    text = (
        raw
        if raw is not None
        else (
            os.environ.get("ARUCO_MARKER_BY_WAYPOINT")
            or os.environ.get("PINKY_ARUCO_IDS")
            or _DEFAULT_ARUCO_IDS
        )
    )
    out: dict[str, int] = {}
    for part in (text or "").split(","):
        part = part.strip()
        if not part or ":" not in part:
            continue
        key, val = part.split(":", 1)
        try:
            out[key.strip()] = int(val.strip())
        except ValueError:
            continue
    return out


def aruco_marker_id_for_waypoint(waypoint_id: str) -> int | None:
    """Marker ID for W*/C/P precision dock; None for S1/S2 or unknown."""
    if not waypoint_id:
        return None
    if waypoint_id.startswith("S"):
        return None
    return parse_aruco_marker_map().get(waypoint_id)
