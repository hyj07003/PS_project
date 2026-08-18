"""ArUco visual docking: SEARCH → FACE → SHIFT (micro) → APPROACH.

Differential-drive flow:
  1) SEARCH  — in-place yaw sweep until marker seen
  2) FACE    — rotate until marker is face-on (정면) and roughly centered
  3) SHIFT   — short wall-parallel steps (cap), then re-FACE / re-detect;
                repeat until lateral (tx) and center are within tolerance
  4) APPROACH — creep forward only after face+center+lateral; stop at standoff (~7cm)
  5) US_APPROACH — marker lost while close; finish with ultrasonic (2nd priority)

거리 측정: 마커(1순위) → 초음파(2순위, APPROACH 중 마커 소실 시).
도착(ARRIVED)은 거리만으로 판단하지 않음. 정자세(중앙·횡·정면)가 우선이고,
정렬된 상태에서 standoff 이내일 때만 완료 (US_APPROACH는 자세 정렬 후만 진입).
"""

from __future__ import annotations

import math
import os
import time
from pathlib import Path
from typing import Any, Callable

import numpy as np

_PINKY_ROOT = Path(__file__).resolve().parents[1]
_DEFAULT_CALIB = _PINKY_ROOT / "camera_calibration.npz"
_DEFAULT_IDS = "W1:1,W2:2,W3:3,W4:4,W5:5,W6:6,C:10,P:11"

DriveFn = Callable[[float, float], Any]
CancelFn = Callable[[], Any]
HoldFn = Callable[[], Any]
ReleaseHoldFn = Callable[[], Any]
LidarFn = Callable[[], Any]
UltrasonicFn = Callable[[], Any]
ProgressFn = Callable[[dict[str, Any]], Any]
OdomFn = Callable[[], tuple[float, float, float] | None]  # () -> (x, y, yaw) or None

ARUCO_PHASE_LABELS_KO: dict[str, str] = {
    "SEARCH": "마커 탐색 중",
    "FACE": "정면·자세 정렬 중",
    "SHIFT": "횡방향 위치 조정 중",
    "APPROACH": "접근·파킹 중",
    "US_APPROACH": "초음파 접근 중",
    "ARRIVED": "도킹 완료",
    "LOST": "마커 재탐색 중",
    "TIMEOUT": "도킹 타임아웃",
    "NO_MARKER": "마커 미검출",
    "FAILED": "도킹 실패",
}


def phase_label_ko(phase: str | None) -> str:
    if not phase:
        return "대기"
    return ARUCO_PHASE_LABELS_KO.get(str(phase), str(phase))


def parse_aruco_ids(raw: str | None = None) -> dict[str, int]:
    text = (raw if raw is not None else os.environ.get("PINKY_ARUCO_IDS", _DEFAULT_IDS)) or ""
    out: dict[str, int] = {}
    for part in text.split(","):
        part = part.strip()
        if not part or ":" not in part:
            continue
        key, val = part.split(":", 1)
        key = key.strip()
        try:
            out[key] = int(val.strip())
        except ValueError:
            continue
    return out


def marker_id_for_waypoint(waypoint_id: str, mapping: dict[str, int] | None = None) -> int | None:
    m = mapping if mapping is not None else parse_aruco_ids()
    return m.get(waypoint_id)


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return float(default)


def _env_int(name: str, default: int) -> int:
    try:
        return int(float(os.environ.get(name, default)))
    except (TypeError, ValueError):
        return int(default)


def _calib_path() -> Path:
    raw = (os.environ.get("PINKY_CAMERA_CALIB_PATH") or "").strip()
    if raw:
        p = Path(raw)
        if not p.is_absolute():
            p = (_PINKY_ROOT / p).resolve()
        return p
    return _DEFAULT_CALIB


def camera_matrix_from_fov(
    width: int,
    height: int,
    hfov_deg: float | None = None,
) -> np.ndarray:
    """Pinhole K from horizontal FOV (broken calib fallback)."""
    hfov = float(
        hfov_deg
        if hfov_deg is not None
        else _env_float("PINKY_CAMERA_HFOV_DEG", 62.0)
    )
    hfov = min(170.0, max(20.0, hfov))
    fx = (0.5 * float(width)) / math.tan(math.radians(hfov) * 0.5)
    fy = fx
    cx = 0.5 * float(width)
    cy = 0.5 * float(height)
    return np.array(
        [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def sanitize_camera_calib(
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    *,
    image_size: tuple[int, int] | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Fix unusable calib (extreme distortion / implausible fx).

    Current repo npz has fx≈1608 on 640px and |k2|≈90 → distances ~10–30× too large.
    """
    cam = np.asarray(camera_matrix, dtype=np.float64).copy()
    dist = np.asarray(dist_coeffs, dtype=np.float64).reshape(-1).copy()
    width = int(
        image_size[0]
        if image_size is not None
        else _env_int("PINKY_CAMERA_WIDTH", 640)
    )
    height = int(
        image_size[1]
        if image_size is not None
        else _env_int("PINKY_CAMERA_HEIGHT", 480)
    )
    mode = (os.environ.get("PINKY_CAMERA_INTRINSICS") or "auto").strip().lower()

    # Scale K if calib resolution differs from capture
    calib_w = os.environ.get("PINKY_CAMERA_CALIB_WIDTH", "").strip()
    calib_h = os.environ.get("PINKY_CAMERA_CALIB_HEIGHT", "").strip()
    if calib_w and calib_h:
        try:
            cw, ch = float(calib_w), float(calib_h)
            if cw > 1 and ch > 1:
                sx, sy = width / cw, height / ch
                cam[0, 0] *= sx
                cam[0, 2] *= sx
                cam[1, 1] *= sy
                cam[1, 2] *= sy
        except ValueError:
            pass

    dist_bad = bool(dist.size and float(np.max(np.abs(dist))) > 5.0)
    fx = float(cam[0, 0])
    fx_bad = fx > float(width) * 1.15  # narrower than ~50° HFOV on this width
    use_fov = mode in ("fov", "hfov") or (
        mode in ("auto", "") and (dist_bad or fx_bad)
    )
    if use_fov:
        cam = camera_matrix_from_fov(width, height)
        dist = np.zeros(5, dtype=np.float64)
    elif dist_bad:
        dist = np.zeros(5, dtype=np.float64)

    # Extra manual scale (true_m / previously_shown_m) if still tuning
    scale = _env_float("PINKY_ARUCO_DISTANCE_SCALE", 1.0)
    if scale > 0 and abs(scale - 1.0) > 1e-6:
        cam[0, 0] *= scale
        cam[1, 1] *= scale

    return cam, dist


def load_camera_calib(
    path: Path | None = None,
    *,
    image_size: tuple[int, int] | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    p = path or _calib_path()
    if not p.is_file():
        # No file — FOV pinhole so preview/dock still get metric distances
        w = int(image_size[0] if image_size else _env_int("PINKY_CAMERA_WIDTH", 640))
        h = int(image_size[1] if image_size else _env_int("PINKY_CAMERA_HEIGHT", 480))
        return camera_matrix_from_fov(w, h), np.zeros(5, dtype=np.float64)
    data = np.load(str(p))
    if "camera_matrix" not in data:
        raise KeyError(f"camera_matrix missing in {p}")
    cam = np.asarray(data["camera_matrix"], dtype=np.float64)
    dist_key = (
        "distortion_coefficients" if "distortion_coefficients" in data else "dist_coeffs"
    )
    if dist_key not in data:
        raise KeyError(f"distortion coefficients missing in {p}")
    dist = np.asarray(data[dist_key], dtype=np.float64).reshape(-1)
    return sanitize_camera_calib(cam, dist, image_size=image_size)


def marker_side_px(corners: np.ndarray) -> float:
    c = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    lengths = [
        float(np.linalg.norm(c[i] - c[(i + 1) % 4])) for i in range(4)
    ]
    return float(sum(lengths) / max(len(lengths), 1))


def estimate_marker_distance_m(
    corners: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    marker_length_m: float,
) -> float | None:
    """Camera-frame depth (m). Prefer pinhole from pixel size; PnP as fallback."""
    if marker_length_m <= 0:
        return None
    pix = marker_side_px(corners)
    if pix < 1.0:
        return None
    fx = float(camera_matrix[0, 0])
    fy = float(camera_matrix[1, 1])
    f = 0.5 * (fx + fy)
    z_pinhole = float(f * marker_length_m / pix)

    half = float(marker_length_m) * 0.5
    obj = np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )
    img = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    import cv2

    ok, _rvec, tvec = cv2.solvePnP(
        obj,
        img,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not ok:
        ok, _rvec, tvec = cv2.solvePnP(obj, img, camera_matrix, dist_coeffs)
    if ok:
        t = np.asarray(tvec, dtype=np.float64).reshape(3)
        z_pnp = float(abs(t[2])) if abs(t[2]) > 0.05 else float(np.linalg.norm(t))
        # If PnP blows up vs pinhole, trust pinhole (bad dist/K leftover)
        if z_pnp > 1e-3 and 0.25 <= (z_pinhole / z_pnp) <= 4.0:
            return float(0.5 * (z_pinhole + z_pnp))
    return z_pinhole


def _aruco_dict_id(name: str) -> int:
    import cv2

    key = (name or "DICT_5X5_50").strip().upper()
    if not key.startswith("DICT_"):
        key = f"DICT_{key}"
    if not hasattr(cv2.aruco, key):
        raise ValueError(f"unknown ArUco dict: {name}")
    return int(getattr(cv2.aruco, key))


def _detect_marker(
    frame_bgr: np.ndarray,
    marker_id: int,
    dictionary,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    marker_length_m: float,
) -> dict[str, float] | None:
    """Return image center + camera-frame pose for target marker, or None."""
    import cv2

    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    corners = None
    ids = None
    try:
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        corners, ids, _ = detector.detectMarkers(gray)
    except Exception:
        params = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)

    if ids is None or len(ids) == 0:
        return None

    ids_flat = ids.flatten()
    match_i = None
    for i, mid in enumerate(ids_flat):
        if int(mid) == int(marker_id):
            match_i = i
            break
    if match_i is None:
        return None

    c = corners[match_i][0]  # (4, 2)
    cx = float(np.mean(c[:, 0]))
    cy = float(np.mean(c[:, 1]))

    half = float(marker_length_m) * 0.5
    obj = np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )
    img = c.astype(np.float64)
    ok, rvec, tvec = cv2.solvePnP(
        obj,
        img,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not ok:
        ok, rvec, tvec = cv2.solvePnP(obj, img, camera_matrix, dist_coeffs)
    if not ok:
        return None

    t = np.asarray(tvec, dtype=np.float64).reshape(3)
    r = np.asarray(rvec, dtype=np.float64).reshape(3)
    rot, _ = cv2.Rodrigues(r)
    # Marker +Z (out of plane) in camera frame — face-on ⇒ mostly ±camera Z
    normal = rot[:, 2]
    # Camera: +X right, +Z forward. Face-on ⇒ normal ≈ (0,0,-1) toward camera.
    yaw_err = float(math.atan2(float(normal[0]), float(-normal[2])))

    tx = float(t[0])
    ty = float(t[1])
    tz = float(abs(t[2]))
    distance_m = estimate_marker_distance_m(
        c, camera_matrix, dist_coeffs, marker_length_m
    )
    if distance_m is None:
        distance_m = tz if tz > 0.05 else float(np.linalg.norm(t))
    # Keep lateral in meters consistent with distance scale:
    # when pinhole depth differs from PnP tz, rescale tx/ty.
    if tz > 1e-3 and distance_m > 1e-3:
        scale = float(distance_m) / tz
        if 0.2 <= scale <= 5.0:
            tx *= scale
            ty *= scale
            tz = float(distance_m)

    # Corner aspect: near 1 when marker looks square (정자) in image
    w_img = float(np.linalg.norm(c[0] - c[1]))
    h_img = float(np.linalg.norm(c[1] - c[2]))
    aspect = (w_img / h_img) if h_img > 1e-3 else 1.0

    return {
        "cx": cx,
        "cy": cy,
        "tx": tx,
        "ty": ty,
        "tz": tz,
        "distanceM": distance_m,
        "yawErr": yaw_err,
        "aspect": aspect,
    }


def _read_ultrasonic_m(get_ultrasonic: UltrasonicFn | None) -> float | None:
    """Front ultrasonic range (m), or None if unavailable."""
    if get_ultrasonic is None:
        return None
    try:
        data = get_ultrasonic()
    except Exception:
        return None
    if data is None:
        return None
    r: float | None
    if isinstance(data, dict):
        raw = data.get("rangeM", data.get("range_m"))
        r = float(raw) if raw is not None else None
        rmin = float(data.get("minRange", data.get("min_range", 0.02)) or 0.02)
        rmax = float(data.get("maxRange", data.get("max_range", 3.0)) or 3.0)
    else:
        r = getattr(data, "range_m", None)
        rmin = float(getattr(data, "min_range", 0.02) or 0.02)
        rmax = float(getattr(data, "max_range", 3.0) or 3.0)
        if r is not None:
            r = float(r)
    if r is None or r <= 0 or math.isnan(r) or math.isinf(r):
        return None
    if r < rmin or r > rmax:
        return None
    return r


def _us_fallback_enabled() -> bool:
    raw = (os.environ.get("PINKY_ARUCO_US_FALLBACK") or "1").strip().lower()
    return raw not in ("0", "false", "off", "no")


def _front_wall_angle_rad(get_lidar: LidarFn | None) -> float | None:
    """Estimate wall angle (rad in base frame) from frontal lidar points. 0 = wall facing robot."""
    if get_lidar is None:
        return None
    try:
        scan = get_lidar()
    except Exception:
        return None
    if scan is None:
        return None
    pairs = list(getattr(scan, "raw_pairs", None) or [])
    pts: list[tuple[float, float]] = []
    if pairs:
        for ang_deg, r in pairs:
            try:
                a = math.radians(float(ang_deg))
                rr = float(r)
            except (TypeError, ValueError):
                continue
            if not (0.12 < rr < 2.5):
                continue
            # Front sector ±50°
            if abs(a) > math.radians(50.0):
                continue
            pts.append((rr * math.cos(a), rr * math.sin(a)))
    else:
        ranges = list(getattr(scan, "ranges", None) or [])
        amin = float(getattr(scan, "angle_min", -math.pi) or -math.pi)
        ainc = float(getattr(scan, "angle_increment", 0.0) or 0.0)
        rmin = float(getattr(scan, "range_min", 0.05) or 0.05)
        rmax = float(getattr(scan, "range_max", 8.0) or 8.0)
        for i, r in enumerate(ranges):
            try:
                rr = float(r)
            except (TypeError, ValueError):
                continue
            if not (rmin < rr < min(rmax, 2.5)):
                continue
            a = amin + i * ainc
            if abs(a) > math.radians(50.0):
                continue
            pts.append((rr * math.cos(a), rr * math.sin(a)))

    if len(pts) < 8:
        return None
    arr = np.asarray(pts, dtype=np.float64)
    # PCA: wall tangent = principal direction of frontal cloud
    mean = arr.mean(axis=0)
    centered = arr - mean
    try:
        _, _, vt = np.linalg.svd(centered, full_matrices=False)
    except Exception:
        return None
    tangent = vt[0]
    # Wall normal ≈ perpendicular to tangent, pointing toward robot (origin)
    normal = np.array([-tangent[1], tangent[0]], dtype=np.float64)
    if float(np.dot(normal, mean)) > 0:
        normal = -normal
    # Angle of wall normal relative to +x (forward): 0 ⇒ face wall squarely
    return float(math.atan2(float(normal[1]), float(normal[0])))


def run_aruco_dock(
    *,
    marker_id: int,
    drive: DriveFn,
    cancel_nav: CancelFn | None = None,
    hold_pose: HoldFn | None = None,
    release_hold: ReleaseHoldFn | None = None,
    get_lidar: LidarFn | None = None,
    get_ultrasonic: UltrasonicFn | None = None,
    on_progress: ProgressFn | None = None,
    get_odom: OdomFn | None = None,
    standoff_m: float | None = None,
    timeout_sec: float | None = None,
    mock: bool = False,
) -> dict[str, Any]:
    """Blocking visual dock. Stops Nav2 once, freezes pose, then cmd_vel servo."""
    standoff = float(
        standoff_m
        if standoff_m is not None
        else _env_float("PINKY_ARUCO_DOCK_STANDOFF_M", 0.07)
    )
    us_fallback = _us_fallback_enabled() and get_ultrasonic is not None
    us_lost_marker_m = max(
        standoff * 2.5,
        _env_float("PINKY_ARUCO_US_LOST_MARKER_M", standoff + 0.25),
    )
    timeout = float(
        timeout_sec
        if timeout_sec is not None
        else _env_float("PINKY_ARUCO_TIMEOUT_SEC", 60.0)
    )
    center_tol = _env_float("PINKY_ARUCO_CENTER_TOL_PX", 28.0)
    face_yaw_tol = math.radians(_env_float("PINKY_ARUCO_FACE_YAW_DEG", 22.0))
    lateral_tol = _env_float("PINKY_ARUCO_LATERAL_TOL_M", 0.07)
    settle_frames = max(2, _env_int("PINKY_ARUCO_ALIGN_SETTLE_FRAMES", 3))
    face_max_sec = max(3.0, _env_float("PINKY_ARUCO_FACE_MAX_SEC", 10.0))
    search_w = abs(_env_float("PINKY_ARUCO_SEARCH_W", 0.15))
    search_amp = math.radians(
        max(20.0, _env_float("PINKY_ARUCO_SEARCH_AMPLITUDE_DEG", 70.0))
    )
    # Time to rotate from center → one extreme at search_w
    search_half_sec = max(0.8, search_amp / max(search_w, 0.05))
    align_w = abs(_env_float("PINKY_ARUCO_ALIGN_W", 0.14))
    shift_w = abs(_env_float("PINKY_ARUCO_SHIFT_W", 0.22))
    shift_v = min(0.035, abs(_env_float("PINKY_ARUCO_SHIFT_V", 0.02)))
    # 한 사이클에 미끄러지는 최대 거리 — 천천히 짧게 이동 후 마커 재검출
    shift_step_m = max(0.008, min(0.12, _env_float("PINKY_ARUCO_SHIFT_STEP_M", 0.02)))
    # 측정 tx 중 이번 스텝에 보정할 비율 — 1.0 미만으로 언더슈트 편향 (반대편 치우침 방지)
    shift_step_gain = max(0.15, min(0.85, _env_float("PINKY_ARUCO_SHIFT_STEP_GAIN", 0.30)))
    shift_max_iters = max(1, _env_int("PINKY_ARUCO_SHIFT_MAX_ITERS", 16))
    shift_settle_sec = max(0.05, _env_float("PINKY_ARUCO_SHIFT_SETTLE_SEC", 0.40))
    # 재진입 히스테리시스: 한 번 맞춘 뒤 |tx|가 이보다 커져야 다시 SHIFT
    lateral_reenter = max(
        lateral_tol * 1.35,
        _env_float("PINKY_ARUCO_LATERAL_REENTER_M", lateral_tol * 1.5),
    )
    # 제로 통과(부호 반전) 후 |tx|가 이하면 추가 SHIFT 없이 APPROACH
    lateral_cross_ok = max(
        lateral_tol,
        _env_float("PINKY_ARUCO_LATERAL_CROSS_OK_M", lateral_tol * 1.8),
    )
    approach_v_max = min(0.05, abs(_env_float("PINKY_ARUCO_APPROACH_V", 0.035)))
    approach_gain = max(0.05, _env_float("PINKY_ARUCO_APPROACH_GAIN", 0.30))
    marker_len = _env_float("PINKY_ARUCO_MARKER_LENGTH_M", 0.037)
    width = _env_int("PINKY_CAMERA_WIDTH", 640)
    height = _env_int("PINKY_CAMERA_HEIGHT", 480)
    device_raw = (os.environ.get("PINKY_CAMERA_DEVICE") or "/dev/video0").strip()

    if mock:
        return {
            "success": True,
            "status": "ARRIVED",
            "message": "mock aruco dock",
            "markerId": int(marker_id),
            "distanceM": standoff,
            "centerErrorPx": 0.0,
            "approachTravelM": standoff,
        }

    if marker_len <= 0:
        return {
            "success": False,
            "status": "FAILED",
            "message": "PINKY_ARUCO_MARKER_LENGTH_M must be > 0",
            "markerId": int(marker_id),
        }

    try:
        camera_matrix, dist_coeffs = load_camera_calib(
            image_size=(width, height)
        )
    except Exception as exc:
        return {
            "success": False,
            "status": "FAILED",
            "message": f"calib load failed: {exc}",
            "markerId": int(marker_id),
        }

    try:
        import cv2
    except ImportError as exc:
        return {
            "success": False,
            "status": "FAILED",
            "message": f"opencv missing: {exc}",
            "markerId": int(marker_id),
        }

    try:
        dict_name = os.environ.get("PINKY_ARUCO_DICT", "DICT_5X5_50")
        dictionary = cv2.aruco.getPredefinedDictionary(_aruco_dict_id(dict_name))
    except Exception as exc:
        return {
            "success": False,
            "status": "FAILED",
            "message": f"aruco dict: {exc}",
            "markerId": int(marker_id),
        }

    if cancel_nav is not None:
        try:
            cancel_nav()
        except Exception:
            pass
        time.sleep(0.15)

    # Pin AMCL/monitor pose for the whole dock — cmd_vel must not jump map pose
    if hold_pose is not None:
        try:
            hold_pose()
        except Exception:
            pass

    from .camera_source import open_frame_source

    source = open_frame_source(device_raw, width, height, quiet=True)
    if source is None:
        return {
            "success": False,
            "status": "FAILED",
            "message": (
                "camera open failed "
                f"(device={device_raw}, backend="
                f"{os.environ.get('PINKY_CAMERA_BACKEND', 'auto')}). "
                "Pi CSI: set PINKY_CAMERA_BACKEND=picamera2"
            ),
            "markerId": int(marker_id),
        }

    deadline = time.time() + max(1.0, timeout)
    phase = "SEARCH"
    z_last: float | None = None
    center_err: float | None = None
    tx_last: float | None = None
    yaw_last: float | None = None
    settle_ok = 0
    last_status = "NO_MARKER"
    face_started_at: float | None = None
    last_progress_t = 0.0

    # Wall-clock sweep (paused while not in SEARCH):
    #   leg0: +w for T   → left extreme
    #   leg1: -w for 2T  → through center to right extreme
    #   leg2: +w for 2T  → through center to left extreme
    #   … covers BOTH sides of arrival yaw (not left-only).
    search_clock = 0.0
    search_clock_t = time.time()
    search_started = False

    # SHIFT sub-FSM: pivot → short slide → pivot back → FACE (re-detect) → repeat
    shift_stage = "idle"  # idle | pivot_out | slide | pivot_in | settle
    shift_stage_until = 0.0
    shift_sign = 1.0
    shift_slide_m = 0.0
    shift_iters = 0
    shift_tx_before: float | None = None  # SHIFT 직전 tx (오버슈트 판정)
    shift_dir_last = 0.0  # 직전 횡이동 부호 (+1 CCW pivot / -1 CW)
    shift_stop_further = False  # 제로 통과·악화 시 추가 SHIFT 금지
    tx_samples: list[float] = []
    us_fallback_active = False
    pose_ready_before_approach = False
    us_last: float | None = None
    approach_travel_m = 0.0
    approach_odom_start: tuple[float, float] | None = None
    loop_dt = 0.04

    def _snap_approach_odom() -> None:
        """Record odom position when APPROACH begins."""
        nonlocal approach_odom_start
        if approach_odom_start is not None:
            return
        if get_odom is not None:
            pose = get_odom()
            if pose is not None:
                approach_odom_start = (pose[0], pose[1])

    def _odom_approach_distance() -> float | None:
        """Euclidean distance from approach start using odom."""
        if approach_odom_start is None or get_odom is None:
            return None
        pose = get_odom()
        if pose is None:
            return None
        dx = pose[0] - approach_odom_start[0]
        dy = pose[1] - approach_odom_start[1]
        return math.sqrt(dx * dx + dy * dy)

    def _final_approach_travel() -> float:
        """Return best estimate of approach distance: odom if available, else cmd_vel estimate."""
        odom_d = _odom_approach_distance()
        if odom_d is not None and odom_d > 1e-4:
            return odom_d
        return approach_travel_m

    def _cmd_vel(linear_x: float, angular_z: float) -> Any:
        nonlocal approach_travel_m
        if phase in ("APPROACH", "US_APPROACH") and float(linear_x) > 0.0:
            _snap_approach_odom()
            approach_travel_m += float(linear_x) * loop_dt
        return drive(linear_x, angular_z)

    def _search_yaw_rate() -> float:
        """Return ±search_w from paused wall-clock schedule (both sides)."""
        nonlocal search_clock, search_clock_t, search_started
        now = time.time()
        if not search_started:
            search_started = True
            search_clock = 0.0
            search_clock_t = now
        else:
            search_clock += max(0.0, now - search_clock_t)
            search_clock_t = now

        t = search_clock
        T = search_half_sec
        if t < T:
            return float(search_w)
        u = t - T
        leg = int(u / (2.0 * T))
        # leg 0,2,4… → −ω (right); leg 1,3,5… → +ω (left)
        return float(-search_w if (leg % 2 == 0) else search_w)

    def _enter_search(*, fresh: bool) -> None:
        nonlocal phase, last_status, settle_ok, shift_stage, face_started_at
        nonlocal search_clock, search_clock_t, search_started, shift_iters
        nonlocal shift_tx_before, shift_dir_last, shift_stop_further, tx_samples
        phase = "SEARCH"
        settle_ok = 0
        shift_stage = "idle"
        shift_iters = 0
        shift_tx_before = None
        shift_dir_last = 0.0
        shift_stop_further = False
        tx_samples = []
        face_started_at = None
        search_clock_t = time.time()
        if fresh:
            search_clock = 0.0
            search_started = False
            search_clock_t = time.time()
        _report("SEARCH" if fresh else "LOST", force=True)

    def _leave_search_for(next_phase: str, next_status: str) -> None:
        """Pause sweep clock when leaving SEARCH so FACE time does not skip legs."""
        nonlocal phase, settle_ok, search_clock, search_clock_t, face_started_at
        if phase == "SEARCH" and search_started:
            now = time.time()
            search_clock += max(0.0, now - search_clock_t)
            search_clock_t = now
        phase = next_phase
        settle_ok = 0
        face_started_at = time.time() if next_phase == "FACE" else None
        stop()
        _report(next_status, force=True)

    def _face_ok(yaw_err: float, err_x: float) -> bool:
        # Center is required; yaw is soft (PnP yaw is often noisy)
        return abs(err_x) <= center_tol

    def _yaw_ok(yaw_err: float) -> bool:
        return abs(yaw_err) <= face_yaw_tol

    def _lateral_ok(tx: float, err_x: float | None = None) -> bool:
        if abs(tx) <= lateral_tol:
            return True
        # 이미 화면 중앙이면 PnP tx 노이즈로 같은 방향 SHIFT를 반복하지 않음
        if err_x is not None and abs(err_x) <= center_tol * 0.55 and abs(tx) <= lateral_tol * 1.6:
            return True
        return False

    def _arrive_pose_ok(
        *,
        center_ok: bool,
        lat_ready: bool,
        yaw_ready: bool,
        err_x: float,
    ) -> bool:
        """거리보다 정자세(정면·중앙·횡) 우선. 도착 허용 자세인지."""
        if not center_ok or not lat_ready:
            return False
        # yaw 는 소프트 — 중앙이 아주 좋으면 yaw 약간 느슨
        if yaw_ready:
            return True
        return abs(err_x) <= center_tol * 0.7

    def _tx_for_shift(tx: float) -> float | None:
        """최근 tx 샘플 중앙값. 부호가 흔들리면 None(대기)."""
        nonlocal tx_samples
        tx_samples.append(float(tx))
        if len(tx_samples) > 7:
            tx_samples = tx_samples[-7:]
        if len(tx_samples) < 3:
            return None
        recent = tx_samples[-3:]
        if any(a * b < 0 for a, b in zip(recent, recent[1:])):
            return None
        return float(sorted(recent)[len(recent) // 2])

    def _align_yaw_cmd(err_x: float, yaw_err: float) -> float:
        # Prefer image center; yaw only as light assist
        w_img = (-align_w if err_x > 0 else align_w) * min(
            1.0, abs(err_x) / max(center_tol * 2.0, 1.0)
        )
        w_face = -align_w * 0.45 * float(
            np.clip(yaw_err / max(face_yaw_tol, 1e-3), -1.0, 1.0)
        )
        return float(np.clip(0.75 * w_img + 0.25 * w_face, -align_w, align_w))

    def _report(phase_name: str, *, force: bool = False) -> None:
        nonlocal last_status, last_progress_t
        last_status = phase_name
        if on_progress is None:
            return
        now = time.time()
        if not force and (now - last_progress_t) < 0.35:
            return
        last_progress_t = now
        try:
            on_progress(
                {
                    "active": True,
                    "phase": phase_name,
                    "phaseLabel": phase_label_ko(phase_name),
                    "markerId": int(marker_id),
                    "distanceM": z_last,
                    "centerErrorPx": center_err,
                    "lateralM": tx_last,
                    "yawErrRad": yaw_last,
                    "shiftIter": shift_iters,
                    "shiftMaxIters": shift_max_iters,
                    "ultrasonicM": us_last,
                    "distanceSource": (
                        "ultrasonic"
                        if phase == "US_APPROACH"
                        else ("marker" if z_last is not None else None)
                    ),
                }
            )
        except Exception:
            pass

    def _approach_speed(distance_m: float) -> float:
        remain = max(0.0, distance_m - standoff)
        if remain <= 0.0:
            return 0.0
        return min(approach_v_max, max(0.012, approach_gain * remain))

    def _can_us_fallback() -> bool:
        """APPROACH 중 가까워져 마커가 안 보일 때만 초음파로 이어감."""
        if not us_fallback or z_last is None:
            return False
        if phase not in ("APPROACH", "US_APPROACH"):
            return False
        if not pose_ready_before_approach and not us_fallback_active:
            return False
        return z_last <= us_lost_marker_m or us_fallback_active

    def _run_us_approach() -> dict[str, Any] | None:
        """초음파 거리로 standoff까지 전진. ARRIVED 시 result dict, 아니면 None."""
        nonlocal phase, us_fallback_active, us_last, z_last
        us = _read_ultrasonic_m(get_ultrasonic)
        if us is not None:
            us_last = us
            z_last = us
        phase = "US_APPROACH"
        us_fallback_active = True
        _report("US_APPROACH")
        if us is None:
            stop()
            return None
        if us <= standoff:
            stop()
            _report("ARRIVED", force=True)
            if release_hold is None:
                _rehold()
            return {
                "success": True,
                "status": "ARRIVED",
                "message": "aruco dock arrived (ultrasonic)",
                "markerId": int(marker_id),
                "distanceM": us,
                "distanceSource": "ultrasonic",
                "centerErrorPx": center_err,
                "lateralM": tx_last,
                "yawErrRad": yaw_last,
                "phase": "ARRIVED",
                "phaseLabel": phase_label_ko("ARRIVED"),
                "approachTravelM": _final_approach_travel(),
            }
        v_cmd = _approach_speed(us)
        if v_cmd <= 0.0:
            stop()
            _report("ARRIVED", force=True)
            if release_hold is None:
                _rehold()
            return {
                "success": True,
                "status": "ARRIVED",
                "message": "aruco dock arrived (ultrasonic standoff)",
                "markerId": int(marker_id),
                "distanceM": us,
                "distanceSource": "ultrasonic",
                "centerErrorPx": center_err,
                "lateralM": tx_last,
                "yawErrRad": yaw_last,
                "phase": "ARRIVED",
                "phaseLabel": phase_label_ko("ARRIVED"),
                "approachTravelM": _final_approach_travel(),
            }
        _cmd_vel(v_cmd, 0.0)
        return None

    def _start_shift(tx: float) -> bool:
        """One micro wall-parallel step, then FACE re-detect. Returns False if should stop shifting."""
        nonlocal shift_stage, shift_stage_until, shift_sign, shift_slide_m
        nonlocal phase, last_status, settle_ok, face_started_at, shift_iters
        nonlocal shift_tx_before, shift_dir_last, shift_stop_further, tx_samples
        if shift_stop_further or shift_iters >= shift_max_iters:
            return False
        if abs(tx) <= lateral_tol:
            return False

        # --- 오버슈트 가드 (직전 SHIFT 이후 재측정) ---
        if shift_tx_before is not None and shift_iters > 0:
            # 1) 제로 통과: 같은 방향으로 더 밀면 반대편으로 치우침
            if tx * shift_tx_before < 0.0:
                shift_stop_further = True
                tx_samples = []
                return False
            # 2) 같은 쪽인데 |tx|가 커짐 → 방향/모델 오류, 중단
            if abs(tx) > abs(shift_tx_before) * 1.12:
                shift_stop_further = True
                tx_samples = []
                return False
            # 3) 이미 꽤 줄었으면 재진입 히스테리시스
            if abs(tx) < lateral_reenter:
                return False

        # tx>0 → marker right → move right: rotate CW (-90°), drive forward
        new_sign = -1.0 if tx > 0 else 1.0
        # 직전과 같은 방향인데 잔차가 작으면 더 짧게 (누적 오버슈트 방지)
        under = shift_step_gain * (0.82 ** min(shift_iters, 5))
        raw = abs(tx) * under
        # 절대 측정치의 55% 초과 이동 금지 (언더슈트 편향)
        shift_slide_m = min(shift_step_m, max(0.008, raw), abs(tx) * 0.55)
        if abs(tx) <= lateral_tol * 2.5:
            shift_slide_m = min(shift_slide_m, max(0.008, abs(tx) * 0.45))
        if shift_dir_last != 0.0 and new_sign == shift_dir_last and shift_iters > 0:
            shift_slide_m *= 0.65

        pivot_t = (math.pi / 2.0) / max(shift_w, 0.05)
        shift_sign = new_sign
        shift_dir_last = new_sign
        shift_tx_before = float(tx)
        shift_stage = "pivot_out"
        shift_stage_until = time.time() + pivot_t
        shift_iters += 1
        phase = "SHIFT"
        settle_ok = 0
        face_started_at = None
        tx_samples = []
        _report("SHIFT", force=True)
        return True

    def stop() -> None:
        try:
            _cmd_vel(0.0, 0.0)
        except Exception:
            pass

    def _rehold() -> None:
        if hold_pose is not None:
            try:
                hold_pose()
            except Exception:
                pass

    _report("SEARCH", force=True)

    try:
        while time.time() < deadline:
            ok, frame = source.read()
            if not ok or frame is None:
                time.sleep(0.05)
                continue

            h, w = frame.shape[:2]
            mid_x = w * 0.5
            det = _detect_marker(
                frame,
                int(marker_id),
                dictionary,
                camera_matrix,
                dist_coeffs,
                marker_len,
            )

            # ---- SHIFT open-loop micro-step (may briefly lose marker while turned) ----
            if phase == "SHIFT" and shift_stage in (
                "pivot_out",
                "slide",
                "pivot_in",
                "settle",
            ):
                now = time.time()
                if shift_stage == "pivot_out":
                    _cmd_vel(0.0, float(shift_sign) * shift_w)
                    if now >= shift_stage_until:
                        slide_t = shift_slide_m / max(shift_v, 0.01)
                        shift_stage = "slide"
                        shift_stage_until = now + slide_t
                elif shift_stage == "slide":
                    _cmd_vel(shift_v, 0.0)
                    if now >= shift_stage_until:
                        pivot_t = (math.pi / 2.0) / max(shift_w, 0.05)
                        shift_stage = "pivot_in"
                        shift_stage_until = now + pivot_t
                elif shift_stage == "pivot_in":
                    _cmd_vel(0.0, float(-shift_sign) * shift_w)
                    if now >= shift_stage_until:
                        stop()
                        shift_stage = "settle"
                        shift_stage_until = now + shift_settle_sec
                elif shift_stage == "settle":
                    stop()
                    if now >= shift_stage_until:
                        shift_stage = "idle"
                        phase = "FACE"
                        settle_ok = 0
                        face_started_at = time.time()
                        _rehold()
                        _report("FACE", force=True)
                        time.sleep(0.08)
                _report("SHIFT")
                time.sleep(0.04)
                continue

            if det is not None:
                if phase == "US_APPROACH":
                    phase = "APPROACH"
                    us_fallback_active = False
                z = float(det["distanceM"])
                z_last = z
                tx = float(det["tx"])
                yaw_err = float(det["yawErr"])
                center_err = float(det["cx"]) - mid_x
                tx_last = tx
                yaw_last = yaw_err

                # Optional lidar wall angle blends into face yaw when marker yaw noisy
                wall_yaw = _front_wall_angle_rad(get_lidar)
                if wall_yaw is not None and abs(yaw_err) < math.radians(25.0):
                    yaw_err = 0.65 * yaw_err + 0.35 * float(wall_yaw)

                if phase == "SEARCH":
                    _leave_search_for("FACE", "FACE")
                    time.sleep(0.08)
                    continue

                center_ok = _face_ok(yaw_err, center_err)
                yaw_ready = _yaw_ok(yaw_err)
                lat_ready = _lateral_ok(tx, center_err)

                # SHIFT 직후 부호가 바뀌었고 잔차가 작으면 추가 횡이동 없이 접근
                if (
                    shift_tx_before is not None
                    and shift_iters > 0
                    and tx * shift_tx_before < 0.0
                    and abs(tx) <= lateral_cross_ok
                ):
                    shift_stop_further = True
                    lat_ready = True

                pose_arrive_ok = _arrive_pose_ok(
                    center_ok=center_ok,
                    lat_ready=lat_ready,
                    yaw_ready=yaw_ready,
                    err_x=center_err,
                )

                # 도착: standoff(~12cm) 이내 + 정자세. 거리만으로는 ARRIVED 금지.
                if z <= standoff and pose_arrive_ok:
                    stop()
                    _report("ARRIVED", force=True)
                    if release_hold is None:
                        _rehold()
                    return {
                        "success": True,
                        "status": "ARRIVED",
                        "message": "aruco dock arrived (pose+standoff)",
                        "markerId": int(marker_id),
                        "distanceM": z,
                        "centerErrorPx": center_err,
                        "lateralM": tx,
                        "yawErrRad": yaw_err,
                        "phase": "ARRIVED",
                        "phaseLabel": phase_label_ko("ARRIVED"),
                        "approachTravelM": _final_approach_travel(),
                    }

                # 가깝더라도 정자세 미달이면 FACE/SHIFT 우선 (거리 도착보다 자세 우선)
                need_pose_first = (not center_ok) or (
                    not lat_ready and not shift_stop_further and phase != "APPROACH"
                )
                # APPROACH 중에는 중앙이 크게 벗어날 때만 FACE 복귀 (close_zone 으로 전진 금지 방지)
                approach_recenter = (
                    phase == "APPROACH"
                    and abs(center_err) > center_tol * 2.0
                )

                # FACE: center marker in frame first (yaw soft). Force progress after face_max_sec.
                if (
                    phase in ("FACE", "ALIGN")
                    or need_pose_first
                    or approach_recenter
                ):
                    if need_pose_first or not pose_arrive_ok:
                        phase = "FACE"
                    if face_started_at is None:
                        face_started_at = time.time()
                    _report("FACE")
                    if not center_ok:
                        settle_ok = 0
                        _cmd_vel(0.0, _align_yaw_cmd(center_err, yaw_err))
                        time.sleep(0.04)
                        continue
                    settle_ok += 1
                    if not yaw_ready:
                        _cmd_vel(0.0, _align_yaw_cmd(center_err, yaw_err) * 0.5)
                    else:
                        stop()
                    face_elapsed = time.time() - face_started_at
                    ready = settle_ok >= settle_frames or face_elapsed >= face_max_sec
                    if not ready:
                        time.sleep(0.04)
                        continue
                    face_started_at = None
                    settle_ok = 0
                    if not lat_ready and not shift_stop_further:
                        tx_cmd = _tx_for_shift(tx)
                        if tx_cmd is None:
                            # 부호 흔들림 — 횡이 애매하면 접근으로 진행 (파킹 정체 방지)
                            phase = "APPROACH"
                            pose_ready_before_approach = True
                            _report("APPROACH", force=True)
                            time.sleep(0.04)
                            continue
                        if _start_shift(tx_cmd):
                            time.sleep(0.04)
                            continue
                        # iter 한도·오버슈트 가드 — 자세 최대한 맞춘 뒤 APPROACH
                        phase = "APPROACH"
                        pose_ready_before_approach = True
                        _report("APPROACH", force=True)
                        time.sleep(0.05)
                        continue
                    # 정자세 OK. 이미 standoff 안이면 여기서 도착 (위에서 놓친 경우)
                    if z <= standoff and _arrive_pose_ok(
                        center_ok=True,
                        lat_ready=True,
                        yaw_ready=yaw_ready,
                        err_x=center_err,
                    ):
                        stop()
                        _report("ARRIVED", force=True)
                        if release_hold is None:
                            _rehold()
                        return {
                            "success": True,
                            "status": "ARRIVED",
                            "message": "aruco dock arrived after face align",
                            "markerId": int(marker_id),
                            "distanceM": z,
                            "centerErrorPx": center_err,
                            "lateralM": tx,
                            "yawErrRad": yaw_err,
                            "phase": "ARRIVED",
                            "phaseLabel": phase_label_ko("ARRIVED"),
                            "approachTravelM": _final_approach_travel(),
                        }
                    phase = "APPROACH"
                    if lat_ready:
                        shift_iters = 0
                        # 오버슈트 가드(shift_stop_further)는 유지 — 같은 방향 재SHIFT 방지
                        if not shift_stop_further:
                            shift_tx_before = None
                    pose_ready_before_approach = True
                    _report("APPROACH", force=True)
                    time.sleep(0.05)
                    continue

                if not lat_ready and not shift_stop_further:
                    if shift_iters < shift_max_iters:
                        phase = "FACE"
                        face_started_at = time.time()
                        settle_ok = 0
                        _report("FACE", force=True)
                        continue
                    # exhausted: keep creeping while trimmed

                phase = "APPROACH"
                if lat_ready and not shift_stop_further:
                    shift_iters = 0
                    shift_tx_before = None
                pose_ready_before_approach = True
                _report("APPROACH")
                # 접근 중에도 자세 깨지면 전진 중단 → FACE
                if not center_ok or (
                    not lat_ready
                    and not shift_stop_further
                    and abs(tx) > lateral_reenter
                ):
                    stop()
                    phase = "FACE"
                    face_started_at = time.time()
                    settle_ok = 0
                    _report("FACE", force=True)
                    continue
                v_cmd = _approach_speed(z)
                if v_cmd <= 0.0:
                    # standoff 도달 — 자세 재확인 후에만 ARRIVED
                    if not _arrive_pose_ok(
                        center_ok=center_ok,
                        lat_ready=lat_ready or shift_stop_further,
                        yaw_ready=yaw_ready,
                        err_x=center_err,
                    ):
                        stop()
                        phase = "FACE"
                        face_started_at = time.time()
                        settle_ok = 0
                        _report("FACE", force=True)
                        continue
                    stop()
                    _report("ARRIVED", force=True)
                    return {
                        "success": True,
                        "status": "ARRIVED",
                        "message": "aruco dock at standoff",
                        "markerId": int(marker_id),
                        "distanceM": z,
                        "centerErrorPx": center_err,
                        "lateralM": tx,
                        "yawErrRad": yaw_err,
                        "phase": "ARRIVED",
                        "phaseLabel": phase_label_ko("ARRIVED"),
                        "approachTravelM": _final_approach_travel(),
                    }
                # 접근 중 횡오차 커지면 다시 micro-SHIFT (한도·오버슈트 가드 내)
                if (
                    not shift_stop_further
                    and shift_iters < shift_max_iters
                    and abs(tx) > lateral_reenter
                    and abs(center_err) <= center_tol * 1.5
                ):
                    tx_cmd = _tx_for_shift(tx)
                    if tx_cmd is not None and _start_shift(tx_cmd):
                        time.sleep(0.04)
                        continue
                yaw_trim = 0.0
                if abs(center_err) > 4.0 or abs(yaw_err) > face_yaw_tol * 0.5:
                    yaw_trim = _align_yaw_cmd(center_err, yaw_err) * 0.4
                _cmd_vel(v_cmd, yaw_trim)
            else:
                stop()
                if phase == "US_APPROACH" or _can_us_fallback():
                    arrived = _run_us_approach()
                    if arrived is not None:
                        return arrived
                    time.sleep(0.04)
                    continue
                if phase != "SEARCH":
                    _enter_search(fresh=False)
                else:
                    _report("SEARCH")
                _cmd_vel(0.0, _search_yaw_rate())

            time.sleep(0.04)

        stop()
        status = "TIMEOUT"
        if last_status in ("NO_MARKER", "SEARCH", "LOST") and z_last is None:
            status = "NO_MARKER"
        _report(status, force=True)
        return {
            "success": False,
            "status": status,
            "message": f"aruco dock {status.lower()}",
            "markerId": int(marker_id),
            "distanceM": z_last,
            "centerErrorPx": center_err,
            "lateralM": tx_last,
            "yawErrRad": yaw_last,
            "phase": status,
            "phaseLabel": phase_label_ko(status),
        }
    finally:
        stop()
        if on_progress is not None:
            try:
                on_progress(
                    {
                        "active": False,
                        "phase": last_status,
                        "phaseLabel": phase_label_ko(last_status),
                        "markerId": int(marker_id),
                        "distanceM": z_last,
                        "centerErrorPx": center_err,
                        "lateralM": tx_last,
                        "yawErrRad": yaw_last,
                    }
                )
            except Exception:
                pass
        if release_hold is not None:
            try:
                release_hold()
            except Exception:
                pass
        try:
            source.release()
        except Exception:
            pass
