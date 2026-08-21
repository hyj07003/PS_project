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
        x=0.009931882239292611,
        y=-0.20226078567130157,
        yaw=0.0,
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
        label="우유",
        x=0.6507310400275261,
        y=-2.094829182972384,
        yaw=-1.5863988018711392,
    ),
    "W4": Waypoint(
        id="W4",
        label="비스킷",
        x=0.9105411110542923,
        y=-1.7831533209001975,
        yaw=-0.0,
    ),
    "W5": Waypoint(
        id="W5",
        label="아이스크림",
        x=0.46365843764447773,
        y=-1.0666614320651615,
        yaw=-1.5770314553459353,
    ),
    "W6": Waypoint(
        id="W6",
        label="샌드위치",
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
    "W7": Waypoint(
        id="W7",
        label="충돌 대기",
        x=0.090,
        y=-0.498,
        yaw=0.0,
    ),
}

STAGING_WAYPOINT_ID = "W7"

ZONE_OCCUPIABLE_IDS = frozenset({"W1", "W2", "W3", "W4", "W5", "W6", "C", "P"})

# W6(샌드위치)와 C(계산대)는 맵상 동일 좌표 — 한쪽 점유 = 양쪽 점유.
ZONE_EQUIVALENTS: dict[str, frozenset[str]] = {
    "W6": frozenset({"W6", "C"}),
    "C": frozenset({"W6", "C"}),
}


def zone_equivalent_ids(waypoint_id: str) -> frozenset[str]:
    wid = (waypoint_id or "").strip().upper()
    return ZONE_EQUIVALENTS.get(wid, frozenset({wid}) if wid else frozenset())


def zones_overlap(a: str | None, b: str | None) -> bool:
    if not a or not b:
        return False
    return bool(zone_equivalent_ids(a) & zone_equivalent_ids(b))


# product slug → shelf waypoint
SLUG_TO_WAYPOINT: dict[str, str] = {
    "cake": "W1",
    "roll-cake": "W2",
    "milk": "W3",
    "biscuit": "W4",
    "ice-cream": "W5",
    "sandwich": "W6",
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
    return _nn_order_ids(start, waypoint_ids, defer_ids=set())


def waypoint_zone_radius_m(waypoint_id: str | None = None) -> float:
    del waypoint_id
    return float(os.environ.get("TRAFFIC_ZONE_RADIUS_M", "0.45"))


def waypoint_zone_center(waypoint_id: str) -> tuple[float, float]:
    """Zone center by waypoint id (W6/C share XY — treat as one footprint)."""
    wp = get_waypoint(waypoint_id.strip().upper())
    return (float(wp.x), float(wp.y))


def is_zone_occupiable(waypoint_id: str) -> bool:
    return (waypoint_id or "").strip().upper() in ZONE_OCCUPIABLE_IDS


def staging_waypoint_id() -> str:
    raw = (os.environ.get("TRAFFIC_STAGING_WAYPOINT") or STAGING_WAYPOINT_ID).strip().upper()
    return raw if raw in WAYPOINTS else STAGING_WAYPOINT_ID


def conflict_aware_tour_order(
    start: Waypoint,
    waypoint_ids: list[str],
    defer_ids: set[str] | frozenset[str] | None = None,
) -> list[Waypoint]:
    """NN tour visiting non-deferred shelves first, deferred at the tail."""
    defer = {d.strip().upper() for d in (defer_ids or set())}
    preferred = [i for i in waypoint_ids if i not in defer]
    deferred = [i for i in waypoint_ids if i in defer]
    ordered = _nn_order_ids(start, preferred, defer_ids=set())
    if deferred:
        tail_start = ordered[-1] if ordered else start
        ordered.extend(_nn_order_ids(tail_start, deferred, defer_ids=set()))
    return ordered


def _nn_order_ids(
    start: Waypoint,
    waypoint_ids: list[str],
    *,
    defer_ids: set[str],
) -> list[Waypoint]:
    del defer_ids
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
    """Parking standoff (m): 7cm default, 40cm for checkout (C)."""
    wid = (waypoint_id or "").strip().upper()
    if wid == "C":
        return float(os.environ.get("ARUCO_DOCK_STANDOFF_C_M", "0.40"))
    return float(os.environ.get("ARUCO_DOCK_STANDOFF_M", "0.07"))


def shelf_undock_after_aruco(waypoint_id: str) -> bool:
    """W*/P 아루코 도킹 후 후진할지. 계산대(C)·대기장소(S*)는 후진하지 않음."""
    wid = (waypoint_id or "").strip().upper()
    if not wid or wid.startswith("S") or wid == "C":
        return False
    if aruco_marker_id_for_waypoint(wid) is None:
        return False
    raw = (os.environ.get("PICK_SHELF_UNDOCK_AFTER_ARUCO") or "1").strip().lower()
    return raw not in ("0", "false", "off", "no")


def shelf_undock_distance_m(
    waypoint_id: str, approach_travel_m: float | None
) -> float:
    """후진 목표 거리 = 접근 직전 마커까지 거리 (측정값이 이 값에 도달하면 정지)."""
    if not shelf_undock_after_aruco(waypoint_id):
        return 0.0
    try:
        marker_m = max(0.0, float(approach_travel_m or 0.0))
    except (TypeError, ValueError):
        marker_m = 0.0
    max_m = float(os.environ.get("PICK_SHELF_UNDOCK_MAX_M", "0.80"))
    return min(marker_m, max_m)


def shelf_undock_odom_travel_m(
    waypoint_id: str,
    approach_range_m: float | None,
    *,
    final_range_m: float | None = None,
) -> float:
    """Odom fallback 후진량 = 접근 전 range − 도킹 후 range (range 전체를 미터로 쓰지 않음)."""
    target = shelf_undock_distance_m(waypoint_id, approach_range_m)
    if target <= 0.0:
        return 0.0
    max_m = float(os.environ.get("PICK_SHELF_UNDOCK_MAX_M", "0.80"))
    slack = float(os.environ.get("PINKY_ARUCO_UNDOCK_TRAVEL_SLACK_M", "0.025"))
    if final_range_m is not None:
        try:
            docked = max(0.0, float(final_range_m))
        except (TypeError, ValueError):
            docked = aruco_standoff_for_waypoint(waypoint_id)
    else:
        docked = aruco_standoff_for_waypoint(waypoint_id)
    travel = max(0.0, target - docked) + max(0.0, slack)
    return min(travel, max_m)
