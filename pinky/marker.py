#!/usr/bin/env python3
"""ArUco preview: capture on robot, GUI on PC (uses modules.camera_source).

Robot:
  PINKY_CAMERA_BACKEND=picamera2 python3 marker.py --serve 0.0.0.0:8787

PC:
  python3 marker.py --view 192.168.x.x:8787

Keys on PC: q / ESC = quit

Recognized markers show estimated distance (m) using camera_calibration.npz
and PINKY_ARUCO_MARKER_LENGTH_M (검은 사각형 한 변 실측, m).
Loads pinky.env automatically (same as run.py).
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Iterator

import cv2
import numpy as np

# Allow `python3 marker.py` from pinky/ without installing package
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

# pinky.env 를 먼저 로드해야 MARKER_LENGTH / INTRINSICS 가 적용됨
try:
    from server.config import load_env

    _loaded_env = load_env()
except Exception:
    _loaded_env = []

from modules.camera_source import FrameSource, flip_mode, open_frame_source  # noqa: E402


def _load_dictionary(dict_name: str):
    name = dict_name.strip().upper()
    if not name.startswith("DICT_"):
        name = f"DICT_{name}"
    if not hasattr(cv2.aruco, name):
        raise ValueError(f"unknown ArUco dict: {name}")
    return name, cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def _detect(gray, dictionary):
    try:
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        return detector.detectMarkers(gray)
    except Exception:
        try:
            params = cv2.aruco.DetectorParameters_create()
        except Exception:
            params = None
        if params is not None:
            return cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
        return cv2.aruco.detectMarkers(gray, dictionary)


def _load_calib(image_size: tuple[int, int] | None = None) -> tuple[np.ndarray, np.ndarray] | None:
    try:
        from modules.aruco_dock import load_camera_calib

        return load_camera_calib(image_size=image_size)
    except Exception as exc:
        print(f"calib load failed ({exc}) — distance overlay disabled", file=sys.stderr)
        return None


def _marker_length_m() -> float:
    try:
        return float(os.environ.get("PINKY_ARUCO_MARKER_LENGTH_M", "0.037"))
    except (TypeError, ValueError):
        return 0.037


def _estimate_distance_m(
    corners: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    marker_length_m: float,
) -> float | None:
    from modules.aruco_dock import estimate_marker_distance_m

    return estimate_marker_distance_m(
        corners, camera_matrix, dist_coeffs, marker_length_m
    )


def annotate(
    frame: np.ndarray,
    dictionary,
    dict_name: str,
    *,
    camera_matrix: np.ndarray | None = None,
    dist_coeffs: np.ndarray | None = None,
    marker_length_m: float = 0.40,
) -> tuple[np.ndarray, list[int]]:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = _detect(gray, dictionary)
    vis = frame.copy()
    id_list: list[int] = []
    dist_parts: list[str] = []

    if ids is not None and len(ids) > 0:
        cv2.aruco.drawDetectedMarkers(vis, corners, ids)
        id_list = [int(x) for x in ids.flatten().tolist()]
        for i, mid in enumerate(id_list):
            dist_m: float | None = None
            if camera_matrix is not None and dist_coeffs is not None:
                dist_m = _estimate_distance_m(
                    corners[i][0],
                    camera_matrix,
                    dist_coeffs,
                    marker_length_m,
                )
            if dist_m is not None:
                dist_parts.append(f"{mid}:{dist_m:.2f}m")
                c = corners[i][0]
                cx = int(float(np.mean(c[:, 0])))
                cy = int(float(np.mean(c[:, 1])))
                cv2.putText(
                    vis,
                    f"{dist_m:.2f}m",
                    (cx - 40, max(24, cy - 12)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
            else:
                dist_parts.append(f"{mid}:?")

        label = "IDs: " + ", ".join(str(i) for i in id_list)
        label2 = "dist: " + ", ".join(dist_parts)
    else:
        label = "IDs: (none)"
        label2 = "dist: —"

    color = (0, 255, 0) if id_list else (0, 0, 255)
    cv2.putText(
        vis,
        label,
        (12, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.85,
        color,
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        vis,
        label2,
        (12, 64),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 255, 255) if id_list else (120, 120, 120),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        vis,
        f"{dict_name} | L={marker_length_m:.2f}m | stream",
        (12, vis.shape[0] - 16),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (220, 220, 220),
        1,
        cv2.LINE_AA,
    )
    return vis, id_list


def _parse_hostport(raw: str, default_port: int = 8787) -> tuple[str, int]:
    text = raw.strip()
    if text.startswith("http://"):
        text = text[len("http://") :]
    if text.startswith("https://"):
        text = text[len("https://") :]
    text = text.split("/", 1)[0]
    if ":" in text:
        host, port_s = text.rsplit(":", 1)
        return host or "0.0.0.0", int(port_s)
    return text or "0.0.0.0", default_port


class _MjpegHandler(BaseHTTPRequestHandler):
    source: FrameSource | None = None
    dictionary = None
    dict_name: str = "DICT_5X5_50"
    jpeg_quality: int = 80
    camera_matrix: np.ndarray | None = None
    dist_coeffs: np.ndarray | None = None
    marker_length_m: float = 0.40

    def log_message(self, fmt: str, *args) -> None:
        sys.stderr.write("%s - %s\n" % (self.address_string(), fmt % args))

    def do_GET(self) -> None:
        if self.path not in ("/", "/stream", "/stream.mjpg"):
            self.send_error(404)
            return
        if self.source is None or self.dictionary is None:
            self.send_error(503, "camera not ready")
            return

        self.send_response(200)
        self.send_header(
            "Content-Type", "multipart/x-mixed-replace; boundary=frame"
        )
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Connection", "close")
        self.end_headers()

        try:
            while True:
                ok, frame = self.source.read()
                if not ok or frame is None:
                    time.sleep(0.02)
                    continue
                vis, _ids = annotate(
                    frame,
                    self.dictionary,
                    self.dict_name,
                    camera_matrix=self.camera_matrix,
                    dist_coeffs=self.dist_coeffs,
                    marker_length_m=self.marker_length_m,
                )
                ok_j, buf = cv2.imencode(
                    ".jpg",
                    vis,
                    [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
                )
                if not ok_j:
                    continue
                data = buf.tobytes()
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(data)}\r\n\r\n".encode())
                self.wfile.write(data)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            return


def run_serve(host: str, port: int) -> int:
    dict_name, dictionary = _load_dictionary(
        os.environ.get("PINKY_ARUCO_DICT") or "DICT_5X5_50"
    )
    marker_len = _marker_length_m()
    width = int(float(os.environ.get("PINKY_CAMERA_WIDTH", "640")))
    height = int(float(os.environ.get("PINKY_CAMERA_HEIGHT", "480")))
    calib = _load_calib(image_size=(width, height))

    source = open_frame_source(quiet=False)
    if source is None:
        return 1

    _MjpegHandler.source = source
    _MjpegHandler.dictionary = dictionary
    _MjpegHandler.dict_name = dict_name
    _MjpegHandler.marker_length_m = marker_len
    if calib is not None:
        _MjpegHandler.camera_matrix, _MjpegHandler.dist_coeffs = calib
        fx = float(calib[0][0, 0])
        print(
            f"intrinsics: fx={fx:.1f} (PINKY_CAMERA_INTRINSICS=auto|fov|calib, "
            f"HFOV≈{os.environ.get('PINKY_CAMERA_HFOV_DEG', '62')}°)"
        )
    else:
        _MjpegHandler.camera_matrix = None
        _MjpegHandler.dist_coeffs = None

    server = ThreadingHTTPServer((host, port), _MjpegHandler)
    if _loaded_env:
        print("loaded env:", ", ".join(_loaded_env))
    else:
        print(
            "WARNING: pinky.env not loaded — using process env / defaults. "
            "Run from pinky/ or set PINKY_ARUCO_MARKER_LENGTH_M"
        )
    print(f"camera flip: {flip_mode()} (PINKY_CAMERA_FLIP=hv|v|h|none; use hv for upside-down)")
    print(
        f"marker length: {marker_len:.4f} m "
        "(PINKY_ARUCO_MARKER_LENGTH_M = 검은 사각형 한 변 실측)"
    )
    if marker_len >= 0.2:
        print(
            "NOTE: L≥20cm 인데 근거리(약 20cm)에서 보면 거리가 크게 나옵니다. "
            "테스트 마커가 작으면 예: export PINKY_ARUCO_MARKER_LENGTH_M=0.037"
        )
    if calib is not None:
        print("distance overlay: ON")
    else:
        print("distance overlay: OFF (calib missing)")
    print(f"robot stream: http://{host}:{port}/stream")
    print(f"on PC run:  python3 marker.py --view <robot-ip>:{port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nstop serve")
    finally:
        server.server_close()
        source.release()
    return 0


def _iter_mjpeg_frames(url: str) -> Iterator[np.ndarray]:
    import urllib.request

    stream_url = url if url.endswith("/stream") else url.rstrip("/") + "/stream"
    req = urllib.request.Request(stream_url, method="GET")
    with urllib.request.urlopen(req, timeout=10) as res:
        buf = b""
        while True:
            chunk = res.read(4096)
            if not chunk:
                break
            buf += chunk
            while True:
                start = buf.find(b"\xff\xd8")
                end = buf.find(b"\xff\xd9")
                if start < 0 or end < 0 or end < start:
                    if len(buf) > 2_000_000:
                        buf = buf[-500_000:]
                    break
                jpg = buf[start : end + 2]
                buf = buf[end + 2 :]
                arr = np.frombuffer(jpg, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is not None:
                    yield frame


def run_view(target: str) -> int:
    host, port = _parse_hostport(target)
    url = f"http://{host}:{port}"
    win = f"ArUco @ {host}:{port} (q=quit)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    print(f"viewing {url}/stream  (q=quit)")
    print("distance is drawn on the robot stream when calib is available")

    try:
        while True:
            try:
                for frame in _iter_mjpeg_frames(url):
                    cv2.imshow(win, frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        return 0
            except Exception as exc:
                print(f"stream error: {exc}; retry in 1s", file=sys.stderr)
                time.sleep(1.0)
                blank = np.zeros((360, 640, 3), dtype=np.uint8)
                cv2.putText(
                    blank,
                    f"waiting for {url}/stream",
                    (40, 180),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                )
                cv2.imshow(win, blank)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    return 0
    finally:
        cv2.destroyAllWindows()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="ArUco: robot stream / PC GUI")
    g = parser.add_mutually_exclusive_group(required=True)
    g.add_argument(
        "--serve",
        metavar="HOST:PORT",
        help="robot: serve annotated MJPEG (e.g. 0.0.0.0:8787)",
    )
    g.add_argument(
        "--view",
        metavar="HOST:PORT",
        help="PC: open GUI from robot stream (e.g. 192.168.129.xx:8787)",
    )
    args = parser.parse_args()

    if args.serve:
        host, port = _parse_hostport(args.serve)
        if host in ("", "localhost"):
            host = "0.0.0.0"
        return run_serve(host, port)
    return run_view(args.view)


if __name__ == "__main__":
    raise SystemExit(main())
