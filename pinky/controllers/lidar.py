from __future__ import annotations

import math
import os
import subprocess
import threading
import time
from dataclasses import dataclass, field
from typing import Any


@dataclass
class LidarScan:
    ranges: list[float] = field(default_factory=list)  # meters (균일 각도 빈)
    angle_min: float = 0.0
    angle_max: float = 2 * math.pi
    angle_increment: float = 0.0
    range_min: float = 0.15
    range_max: float = 12.0
    frame_id: str = "rplidar_link"
    source: str = "unavailable"
    stamp: float | None = None
    # 맵용 원시 포인트 (angle_deg, range_m) — 빈 샘플링 전
    raw_pairs: list[tuple[float, float]] = field(default_factory=list)


class LidarReader:
    """
    RPLidar C1 스캔 → 최신 프레임 보관 + (옵션) /scan 발행용 데이터 제공.

    우선순위:
      1) rplidarc1 (C1 전용)
      2) pyserial 직접 프로토콜
      3) sllidar_ros2 launch 위임
    """

    _shared: "LidarReader | None" = None

    @classmethod
    def shared(cls) -> "LidarReader":
        if cls._shared is None:
            cls._shared = cls()
        return cls._shared

    def __init__(self) -> None:
        self._port = os.environ.get("PINKY_LIDAR_PORT", "/dev/ttyAMA0")
        self._baud = int(os.environ.get("PINKY_LIDAR_BAUD", "460800"))
        self._lock = threading.Lock()
        self._latest: LidarScan | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._status: list[str] = []
        self._device: Any = None
        self._serial: Any = None
        self._sllidar_proc: subprocess.Popen | None = None
        self.external_sllidar = False
        self._fallback = os.environ.get("PINKY_SENSOR_FALLBACK", "0") in (
            "1",
            "true",
            "True",
        )

    @property
    def status(self) -> list[str]:
        return list(self._status)

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="pinky-lidar"
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._close_device()
        if self._sllidar_proc is not None:
            try:
                self._sllidar_proc.terminate()
            except Exception:
                pass
            self._sllidar_proc = None

    def read(self) -> LidarScan | None:
        with self._lock:
            if self._latest is not None:
                return self._latest
        if self._fallback:
            return self._synthetic()
        return None

    def _loop(self) -> None:
        # C1 전용 드라이버 우선
        if self._loop_rplidarc1():
            return
        if self._loop_pyserial():
            return
        if self._start_sllidar_external():
            while not self._stop.is_set():
                time.sleep(0.5)
            return

        self._status.append(
            f"lidar unavailable on {self._port} "
            "(pip install rplidarc1 또는 sllidar_ros2 / 시리얼 권한 확인)"
        )
        while not self._stop.is_set():
            if self._fallback:
                with self._lock:
                    self._latest = self._synthetic()
            time.sleep(0.2)

    def _loop_rplidarc1(self) -> bool:
        """SLAMTEC RPLidar C1 전용 패키지 (pip install rplidarc1)."""
        try:
            from rplidarc1 import RPLidar
        except ImportError:
            self._status.append("rplidarc1 missing — pip install rplidarc1")
            return False

        import asyncio

        try:
            lidar = RPLidar(self._port, self._baud)
            self._device = lidar
            try:
                lidar.healthcheck()
            except Exception as exc:
                self._status.append(f"rplidarc1 healthcheck warn: {exc}")
            self._status.append(
                f"rplidarc1 ok port={self._port} baud={self._baud}"
            )
        except Exception as exc:
            self._status.append(f"rplidarc1 connect failed: {exc}")
            self._close_device()
            return False

        async def _run() -> None:
            try:
                async with asyncio.TaskGroup() as tg:
                    # make_return_dict=False: angle→dict 는 덮어써서 밀도 손실
                    # output_queue 의 원시 샘플을 1회전 단위로 모은다
                    tg.create_task(lidar.simple_scan(make_return_dict=False))
                    tg.create_task(self._async_consume_c1(lidar))
                    tg.create_task(self._async_stop_watch(lidar))
            except* Exception as eg:
                self._status.append(f"rplidarc1 scan error: {eg.exceptions}")
            finally:
                try:
                    lidar.stop_event.set()
                except Exception:
                    pass
                try:
                    lidar.shutdown()
                except Exception:
                    try:
                        lidar.reset()
                    except Exception:
                        pass

        try:
            asyncio.run(_run())
        except Exception as exc:
            self._status.append(f"rplidarc1 asyncio error: {exc}")
            self._close_device()
            return False
        self._close_device()
        return True

    async def _async_stop_watch(self, lidar: Any) -> None:
        import asyncio

        while not self._stop.is_set():
            await asyncio.sleep(0.2)
        try:
            lidar.stop_event.set()
        except Exception:
            pass

    async def _async_consume_c1(self, lidar: Any) -> None:
        """output_queue 원시 샘플을 각도 wrap(1회전) 단위로 모아 고밀도 프레임 생성."""
        import asyncio

        bins = int(os.environ.get("PINKY_LIDAR_BINS", "2160"))
        bins = max(360, min(8640, bins))
        # 한 회전에 쌓을 최대 원시 포인트 (C1 ~5–9kHz / 8–15Hz ≈ 500–1000+, Dense 여유)
        max_raw = int(os.environ.get("PINKY_LIDAR_RAW_MAX", str(max(bins * 2, 4320))))
        max_raw = max(720, min(12000, max_raw))
        min_pts = int(os.environ.get("PINKY_LIDAR_MIN_POINTS", "200"))
        min_pts = max(50, min_pts)

        accum: list[tuple[float, float]] = []
        last_a: float | None = None
        last_publish = 0.0

        def _publish(pairs: list[tuple[float, float]]) -> None:
            nonlocal last_publish
            if len(pairs) < min_pts:
                return
            scan = self._from_angle_dist(pairs, source="rplidarc1")
            with self._lock:
                self._latest = scan
            last_publish = time.time()

        while not self._stop.is_set():
            drained = 0
            try:
                while drained < 4000:
                    data = lidar.output_queue.get_nowait()
                    drained += 1
                    try:
                        a = float(data.get("a_deg", 0.0)) % 360.0
                        d_mm = data.get("d_mm")
                        if d_mm is None:
                            continue
                        d_m = float(d_mm) / 1000.0
                    except (TypeError, ValueError, AttributeError):
                        continue
                    if d_m <= 0.05 or d_m > 16.0:
                        continue

                    # 각도 wrap: 350° → 10° 등 새 회전 시작
                    if last_a is not None and a + 50.0 < last_a:
                        _publish(accum)
                        accum = []
                    accum.append((a, d_m))
                    last_a = a
                    if len(accum) >= max_raw:
                        _publish(accum)
                        accum = []
                        last_a = None
            except asyncio.QueueEmpty:
                pass
            except Exception:
                # queue.Empty (stdlib) 등
                pass

            # 안전 플러시: wrap 감지가 안 되면 시간 기준으로 발행
            if accum and time.time() - last_publish > 0.35:
                _publish(accum)
                accum = []
                last_a = None

            await asyncio.sleep(0.005 if drained else 0.015)

    def _loop_pyserial(self) -> bool:
        """Slamtec 표준 SCAN(0x20) 프로토콜로 포인트 수집."""
        try:
            import serial
        except ImportError:
            self._status.append("pyserial missing")
            return False

        try:
            ser = serial.Serial(
                self._port,
                self._baud,
                timeout=1,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            self._serial = ser
        except Exception as exc:
            self._status.append(f"serial open failed: {exc}")
            return False

        try:
            # STOP + RESET
            ser.write(bytes([0xA5, 0x25]))
            time.sleep(0.05)
            ser.reset_input_buffer()
            ser.write(bytes([0xA5, 0x40]))
            time.sleep(0.5)
            ser.reset_input_buffer()

            # START SCAN
            ser.write(bytes([0xA5, 0x20]))
            # descriptor: 7 bytes starting with 0xA5 0x5A
            desc = self._read_exact(ser, 7)
            if not desc or desc[0] != 0xA5 or desc[1] != 0x5A:
                self._status.append(f"bad scan descriptor: {desc!r}")
                return False

            self._status.append(f"pyserial lidar ok port={self._port} baud={self._baud}")
            points: list[tuple[float, float]] = []
            last_publish = 0.0

            while not self._stop.is_set():
                raw = self._read_exact(ser, 5)
                if not raw:
                    continue
                # node packet: quality/start | angle_q6 | distance_q2
                b0, b1, b2, b3, b4 = raw
                start = bool(b0 & 0x01)
                # check sync bits roughly
                angle_q6 = ((b1 >> 1) | (b2 << 7)) & 0xFFFF
                dist_q2 = (b3 | (b4 << 8)) & 0xFFFF
                angle_deg = angle_q6 / 64.0
                dist_m = dist_q2 / 4000.0  # q2 mm → m (/4 /1000)

                if start and points:
                    scan = self._from_angle_dist(points, source="serial")
                    with self._lock:
                        self._latest = scan
                    points = []
                    last_publish = time.time()

                if 0.05 < dist_m < 16.0:
                    points.append((angle_deg, dist_m))

                # safety flush
                if points and time.time() - last_publish > 1.0:
                    scan = self._from_angle_dist(points, source="serial")
                    with self._lock:
                        self._latest = scan
                    points = []
                    last_publish = time.time()
        except Exception as exc:
            self._status.append(f"pyserial scan error: {exc}")
            return False
        finally:
            self._close_device()
        return True

    def _start_sllidar_external(self) -> bool:
        mode = os.environ.get("PINKY_LIDAR_SLLIDAR", "auto").lower()
        if mode in ("0", "false", "off", "no"):
            return False
        try:
            # sllidar 패키지 존재 여부
            chk = subprocess.run(
                ["ros2", "pkg", "prefix", "sllidar_ros2"],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if chk.returncode != 0:
                self._status.append("sllidar_ros2 not installed")
                return False
        except Exception as exc:
            self._status.append(f"sllidar check failed: {exc}")
            return False

        try:
            self._sllidar_proc = subprocess.Popen(
                [
                    "ros2",
                    "launch",
                    "sllidar_ros2",
                    "sllidar_c1_launch.py",
                    f"serial_port:={self._port}",
                    "frame_id:=rplidar_link",
                    "scan_mode:=DenseBoost",
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
            )
            self.external_sllidar = True
            self._status.append(
                f"sllidar_ros2 launched pid={self._sllidar_proc.pid} "
                "( /scan via ROS, serial not opened by pinky )"
            )
            return True
        except Exception as exc:
            self._status.append(f"sllidar launch failed: {exc}")
            return False

    def _from_angle_dist(
        self, pairs: list[tuple[float, float]], source: str
    ) -> LidarScan:
        # LaserScan 균일 빈 (기본 2160 ≈ 0.167°). 맵은 raw_pairs 사용.
        bins = int(os.environ.get("PINKY_LIDAR_BINS", "2160"))
        bins = max(360, min(8640, bins))
        ranges = [float("inf")] * bins
        raw: list[tuple[float, float]] = []
        for angle_deg, dist_m in pairs:
            if dist_m <= 0:
                continue
            a = float(angle_deg) % 360.0
            d = min(float(dist_m), 12.0)
            raw.append((a, d))
            idx = int(a / 360.0 * bins) % bins
            if d < ranges[idx]:
                ranges[idx] = d
        out = [
            0.0 if (r == float("inf") or r <= 0) else r for r in ranges
        ]
        return LidarScan(
            ranges=out,
            angle_min=0.0,
            angle_max=2 * math.pi,
            angle_increment=2 * math.pi / bins,
            range_min=0.15,
            range_max=12.0,
            frame_id="rplidar_link",
            source=source,
            stamp=time.time(),
            raw_pairs=raw,
        )

    def _synthetic(self) -> LidarScan:
        bins = int(os.environ.get("PINKY_LIDAR_BINS", "2160"))
        bins = max(360, min(8640, bins))
        t = time.time()
        ranges = []
        raw: list[tuple[float, float]] = []
        for i in range(bins):
            angle = 360.0 * i / bins
            r = round(max(0.2, 1.5 + 0.4 * math.sin(math.radians(angle) * 2 + t * 0.5)), 3)
            ranges.append(r)
            raw.append((angle, r))
        return LidarScan(
            ranges=ranges,
            angle_min=0.0,
            angle_max=2 * math.pi,
            angle_increment=2 * math.pi / bins,
            range_min=0.15,
            range_max=12.0,
            source="synthetic",
            stamp=time.time(),
            raw_pairs=raw,
        )

    @staticmethod
    def _read_exact(ser: Any, n: int) -> bytes | None:
        buf = bytearray()
        while len(buf) < n:
            chunk = ser.read(n - len(buf))
            if not chunk:
                return None
            buf.extend(chunk)
        return bytes(buf)

    def _close_device(self) -> None:
        if self._device is not None:
            try:
                if hasattr(self._device, "stop_event"):
                    self._device.stop_event.set()
            except Exception:
                pass
            try:
                if hasattr(self._device, "shutdown"):
                    self._device.shutdown()
                elif hasattr(self._device, "reset"):
                    self._device.reset()
                elif hasattr(self._device, "stop"):
                    self._device.stop()
            except Exception:
                pass
            try:
                if hasattr(self._device, "disconnect"):
                    self._device.disconnect()
            except Exception:
                pass
            self._device = None
        if self._serial is not None:
            try:
                self._serial.write(bytes([0xA5, 0x25]))
            except Exception:
                pass
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
