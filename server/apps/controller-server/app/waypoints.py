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
    "S1": Waypoint(
        id="S1",
        label="cart-1 대기",
        x=0.009931882239292611,
        y=0.021114122581406713,
        yaw=0.0,
    ),
    "S2": Waypoint(
        id="S2",
        label="cart-2 대기",
        x=0.04742698442363813,
        y=-0.20226078567130157,
        yaw=-0.0,
    ),
    "W1": Waypoint(
        id="W1",
        label="케이크",
        x=0.2468828953013069,
        y=-1.0380566412490777,
        yaw=3.1099279885559223,
    ),
    "W2": Waypoint(
        id="W2",
        label="롤케이크",
        x=0.2132442625034366,
        y=-1.9311329088412792,
        yaw=3.1234749332087595,
    ),
    "W3": Waypoint(
        id="W3",
        label="샌드위치",
        x=0.6507310400275261,
        y=-2.094829182972384,
        yaw=-1.5863988018711392,
    ),
    "W4": Waypoint(
        id="W4",
        label="아이스크림",
        x=0.9105411110542923,
        y=-1.5031533209001975,
        yaw=-0.0,
    ),
    "W5": Waypoint(
        id="W5",
        label="우유",
        x=0.46365843764447773,
        y=-1.0666614320651615,
        yaw=-1.5770314553459353,
    ),
    "W6": Waypoint(
        id="W6",
        label="콜라",
        x=0.8402836518740733,
        y=-0.9553571393194527,
        yaw=0.0,
    ),
    "C": Waypoint(
        id="C",
        label="계산대",
        x=0.8402836518740733,
        y=-0.9553571393194527,
        yaw=1.6079919362854,
    ),
    "P": Waypoint(
        id="P",
        label="운송대기",
        x=0.38512756535468234,
        y=0.38716129241689257,
        yaw=3.135292688148418,
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


def aruco_standoff_for_waypoint(waypoint_id: str) -> float:
    """Parking standoff (m): 7cm default, 35cm for checkout (C)."""
    wid = (waypoint_id or "").strip().upper()
    if wid == "C":
        return float(os.environ.get("ARUCO_DOCK_STANDOFF_C_M", "0.35"))
    return float(os.environ.get("ARUCO_DOCK_STANDOFF_M", "0.07"))
