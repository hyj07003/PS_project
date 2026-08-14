"""Shared camera open for ArUco dock + marker.py preview.

Pi 5 CSI: prefer picamera2 (libcamera). V4L2 /dev/video0 often has no frames.
Env:
  PINKY_CAMERA_BACKEND=auto|v4l2|picamera2|gstreamer
  PINKY_CAMERA_DEVICE=/dev/video0
  PINKY_CAMERA_WIDTH / PINKY_CAMERA_HEIGHT
  PINKY_CAMERA_FLIP=hv|v|h|none  (default hv = 180°; upside-down mount)

Note: vertical-only flip (v) mirrors the image and breaks ArUco bit patterns.
Prefer hv / 180 for upside-down cameras so detection still works.
"""

from __future__ import annotations

import glob
import os
import subprocess
import sys
import time
from typing import Any, Protocol

import cv2
import numpy as np


class FrameSource(Protocol):
    def read(self) -> tuple[bool, Any]: ...

    def release(self) -> None: ...


def flip_mode() -> str:
    """hv=180° (default), v=vertical-only (mirrors!), h=horizontal, none=off."""
    raw = (os.environ.get("PINKY_CAMERA_FLIP") or "hv").strip().lower()
    if raw in ("0", "off", "none", "false", ""):
        return "none"
    if raw in ("v", "vertical", "ud", "updown"):
        return "v"
    if raw in ("h", "horizontal", "lr", "leftright"):
        return "h"
    if raw in ("hv", "vh", "both", "180", "rot180"):
        return "hv"
    return "hv"


def apply_camera_flip(frame: np.ndarray) -> np.ndarray:
    mode = flip_mode()
    if mode == "v":
        # Mirrors chirality — ArUco often fails. Prefer hv for mount correction.
        return cv2.flip(frame, 0)
    if mode == "h":
        return cv2.flip(frame, 1)
    if mode == "hv":
        # Both axes = ROTATE_180: upright view without mirroring markers
        return cv2.rotate(frame, cv2.ROTATE_180)
    return frame


class CvFrameSource:
    def __init__(self, cap: cv2.VideoCapture, label: str):
        self._cap = cap
        self.label = label

    def read(self) -> tuple[bool, Any]:
        ok, frame = self._cap.read()
        if not ok or frame is None:
            return ok, frame
        return True, apply_camera_flip(frame)

    def release(self) -> None:
        self._cap.release()


class Picamera2Source:
    """Raspberry Pi CSI / libcamera via picamera2."""

    def __init__(self, width: int, height: int):
        from picamera2 import Picamera2  # type: ignore

        self._picam = Picamera2()
        cfg = self._picam.create_preview_configuration(
            main={"size": (width or 640, height or 480), "format": "RGB888"}
        )
        self._picam.configure(cfg)
        self._picam.start()
        time.sleep(0.3)
        self.label = "picamera2"

    def read(self) -> tuple[bool, Any]:
        try:
            rgb = self._picam.capture_array()
            if rgb is None:
                return False, None
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            return True, apply_camera_flip(bgr)
        except Exception:
            return False, None

    def release(self) -> None:
        try:
            self._picam.stop()
        except Exception:
            pass
        try:
            self._picam.close()
        except Exception:
            pass


def _list_video_nodes(*, quiet: bool = False) -> list[str]:
    nodes = sorted(glob.glob("/dev/video*"), key=lambda p: (len(p), p))
    if quiet:
        return nodes
    print("video nodes:", ", ".join(nodes) if nodes else "(none)")
    try:
        out = subprocess.check_output(
            ["v4l2-ctl", "--list-devices"],
            stderr=subprocess.STDOUT,
            text=True,
            timeout=5,
        )
        print(out.rstrip())
    except Exception as exc:
        print(f"(v4l2-ctl unavailable: {exc})")
    return nodes


def _try_v4l2_device(
    path: str, width: int, height: int, *, quiet: bool = False
) -> FrameSource | None:
    attempts: list[tuple[Any, int]] = [(path, cv2.CAP_V4L2), (path, 0)]
    if path.startswith("/dev/video"):
        try:
            idx = int(path.replace("/dev/video", ""))
            attempts.append((idx, cv2.CAP_V4L2))
            attempts.append((idx, 0))
        except ValueError:
            pass

    fourccs = [
        None,
        cv2.VideoWriter_fourcc(*"MJPG"),
        cv2.VideoWriter_fourcc(*"YUYV"),
    ]

    for arg, api in attempts:
        for fourcc in fourccs:
            if api:
                cap = cv2.VideoCapture(arg, api)
            else:
                cap = cv2.VideoCapture(arg)
            if not cap.isOpened():
                continue
            if fourcc is not None:
                cap.set(cv2.CAP_PROP_FOURCC, fourcc)
            if width > 0:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            if height > 0:
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            for _ in range(40):
                ok, frame = cap.read()
                if ok and frame is not None and getattr(frame, "size", 0) > 0:
                    tag = f"{arg} api={api} fourcc={fourcc}"
                    if not quiet:
                        print(f"camera ok: {tag} shape={frame.shape}")
                    return CvFrameSource(cap, tag)
                time.sleep(0.03)
            cap.release()
    return None


def _try_gstreamer(
    width: int, height: int, *, quiet: bool = False
) -> FrameSource | None:
    w, h = width or 640, height or 480
    pipelines = [
        (
            f"libcamerasrc ! video/x-raw,width={w},height={h},format=RGB "
            f"! videoconvert ! video/x-raw,format=BGR ! appsink drop=1"
        ),
        (
            f"v4l2src device=/dev/video0 ! video/x-raw,width={w},height={h} "
            f"! videoconvert ! appsink drop=1"
        ),
        (
            f"v4l2src device=/dev/video1 ! video/x-raw,width={w},height={h} "
            f"! videoconvert ! appsink drop=1"
        ),
    ]
    for pipe in pipelines:
        cap = cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            continue
        for _ in range(30):
            ok, frame = cap.read()
            if ok and frame is not None and getattr(frame, "size", 0) > 0:
                if not quiet:
                    print(f"camera ok: gstreamer shape={frame.shape}")
                return CvFrameSource(cap, "gstreamer")
            time.sleep(0.05)
        cap.release()
    return None


def _try_picamera2(
    width: int, height: int, *, quiet: bool = False
) -> FrameSource | None:
    try:
        src = Picamera2Source(width, height)
    except Exception as exc:
        if not quiet:
            print(f"picamera2 failed: {exc}", file=sys.stderr)
        return None
    for _ in range(20):
        ok, frame = src.read()
        if ok and frame is not None:
            if not quiet:
                print(f"camera ok: picamera2 shape={frame.shape}")
            return src
        time.sleep(0.05)
    src.release()
    if not quiet:
        print("picamera2 opened but no frame", file=sys.stderr)
    return None


def open_frame_source(
    device_raw: str | None = None,
    width: int | None = None,
    height: int | None = None,
    *,
    quiet: bool = False,
) -> FrameSource | None:
    device = (
        device_raw
        if device_raw is not None
        else (os.environ.get("PINKY_CAMERA_DEVICE") or "/dev/video0")
    ).strip()
    if width is None:
        try:
            width = int(os.environ.get("PINKY_CAMERA_WIDTH", "640"))
        except ValueError:
            width = 640
    if height is None:
        try:
            height = int(os.environ.get("PINKY_CAMERA_HEIGHT", "480"))
        except ValueError:
            height = 480

    backend = (os.environ.get("PINKY_CAMERA_BACKEND") or "auto").strip().lower()
    if not quiet:
        print(
            f"PINKY_CAMERA_BACKEND={backend} device={device} flip={flip_mode()}"
        )
    nodes = _list_video_nodes(quiet=quiet)

    if backend in ("picamera2", "auto"):
        src = _try_picamera2(width, height, quiet=quiet)
        if src is not None:
            return src
        if backend == "picamera2":
            return None

    if backend in ("gstreamer", "auto"):
        src = _try_gstreamer(width, height, quiet=quiet)
        if src is not None:
            return src
        if backend == "gstreamer":
            return None

    ordered: list[str] = []
    if device:
        ordered.append(device)
    for n in nodes:
        if n not in ordered:
            ordered.append(n)

    for path in ordered:
        if not quiet:
            print(f"trying V4L2 {path} ...")
        src = _try_v4l2_device(path, width, height, quiet=quiet)
        if src is not None:
            return src
        if not quiet:
            print(f"  no frame from {path}", file=sys.stderr)

    if not quiet:
        print(
            "camera failed.\n"
            "  - CSI Pi cam: sudo apt install python3-picamera2\n"
            "    PINKY_CAMERA_BACKEND=picamera2\n"
            "  - USB: fuser -v /dev/video0 /dev/video1",
            file=sys.stderr,
        )
    return None
