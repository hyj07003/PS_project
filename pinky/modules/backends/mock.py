from __future__ import annotations

import math
import random
import time
from typing import Any

from ..types import BatteryData, ImuData, LidarData, UltrasonicData
from .base import RobotBackend

EMOTIONS = (
    "hello",
    "basic",
    "angry",
    "bored",
    "fun",
    "happy",
    "interest",
    "sad",
)


class MockBackend(RobotBackend):
    """Dev PC용 Mock — pinky_pro ROS 토픽/서비스 계약을 흉내 냄."""

    name = "mock"

    def __init__(self) -> None:
        self._started = False
        self._t0 = time.time()
        self._led = {"r": 0, "g": 0, "b": 0, "command": "clear"}
        self._brightness = 128
        self._emotion = "basic"
        self._cmd_vel = {"linearX": 0.0, "angularZ": 0.0}
        self._nav_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self._nav_goal: dict[str, float] | None = None
        self._is_navigating = False
        self._nav_goal_t0 = 0.0

    def start(self) -> None:
        self._started = True
        self._t0 = time.time()

    def stop(self) -> None:
        self._started = False

    def is_online(self) -> bool:
        return self._started

    def get_battery(self) -> BatteryData:
        # ~7.4V nominal Li-ion pack with mild drain simulation
        age = time.time() - self._t0
        voltage = max(6.5, 7.6 - age * 0.00005 + random.uniform(-0.02, 0.02))
        percent = min(100.0, max(0.0, (voltage - 6.5) / (8.4 - 6.5) * 100))
        return BatteryData(percent=round(percent, 1), voltage=round(voltage, 3), source="mock")

    def get_lidar(self) -> LidarData:
        n = 360
        ranges = []
        t = time.time() - self._t0
        for i in range(n):
            angle = math.radians(i)
            base = 1.2 + 0.3 * math.sin(angle * 3 + t)
            ranges.append(round(max(0.15, base + random.uniform(-0.05, 0.05)), 3))
        return LidarData(
            ranges=ranges,
            angle_min=0.0,
            angle_max=2 * math.pi,
            angle_increment=2 * math.pi / n,
            range_min=0.15,
            range_max=12.0,
            frame_id="rplidar_link",
            stamp=time.time(),
        )

    def get_imu(self) -> ImuData:
        t = time.time() - self._t0
        yaw = 0.1 * math.sin(t * 0.2)
        return ImuData(
            orientation={
                "x": 0.0,
                "y": 0.0,
                "z": math.sin(yaw / 2),
                "w": math.cos(yaw / 2),
            },
            angular_velocity={"x": 0.0, "y": 0.0, "z": round(0.02 * math.cos(t * 0.2), 4)},
            linear_acceleration={"x": 0.0, "y": 0.0, "z": 9.81},
            frame_id="imu_link",
            stamp=time.time(),
        )

    def get_ultrasonic(self) -> UltrasonicData:
        t = time.time() - self._t0
        dist = 0.4 + 0.2 * math.sin(t * 0.5) + random.uniform(-0.02, 0.02)
        return UltrasonicData(
            range_m=round(max(0.02, dist), 3),
            ir_raw=[
                int(800 + 50 * math.sin(t)),
                int(750 + 40 * math.cos(t)),
                int(780 + 30 * math.sin(t * 1.3)),
            ],
        )

    def set_led(
        self,
        command: str = "fill",
        r: int = 0,
        g: int = 0,
        b: int = 0,
        pixels: list[int] | None = None,
    ) -> dict[str, Any]:
        self._led = {
            "command": command,
            "r": int(r),
            "g": int(g),
            "b": int(b),
            "pixels": pixels or [],
        }
        return {"success": True, "message": "mock led ok", "state": self._led}

    def set_brightness(self, brightness: int) -> dict[str, Any]:
        self._brightness = max(0, min(255, int(brightness)))
        return {"success": True, "message": "mock brightness ok", "brightness": self._brightness}

    def set_emotion(self, emotion: str) -> dict[str, Any]:
        if emotion not in EMOTIONS:
            return {
                "success": False,
                "message": f"unknown emotion; use one of {EMOTIONS}",
            }
        self._emotion = emotion
        return {"success": True, "message": "mock emotion ok", "emotion": self._emotion}

    def drive(self, linear_x: float, angular_z: float) -> dict[str, Any]:
        self._cmd_vel = {"linearX": float(linear_x), "angularZ": float(angular_z)}
        return {"success": True, "message": "mock cmd_vel ok", "cmdVel": self._cmd_vel}

    def get_nav_pose(self) -> dict[str, float] | None:
        self._tick_nav()
        return dict(self._nav_pose)

    def is_navigating(self) -> bool:
        self._tick_nav()
        return self._is_navigating

    def set_initial_pose(self, x: float, y: float, yaw: float = 0.0) -> dict[str, Any]:
        self._nav_pose = {"x": float(x), "y": float(y), "yaw": float(yaw)}
        self._is_navigating = False
        self._nav_goal = None
        return {
            "success": True,
            "message": "mock initialpose set",
            "pose": dict(self._nav_pose),
        }

    def navigate_to(self, x: float, y: float, yaw: float = 0.0) -> dict[str, Any]:
        self._nav_goal = {"x": float(x), "y": float(y), "yaw": float(yaw)}
        self._nav_goal_t0 = time.time()
        self._is_navigating = True
        return {
            "success": True,
            "message": "mock goal accepted",
            "goal": dict(self._nav_goal),
        }

    def navigate_to_wait(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        timeout_sec: float = 180.0,
    ) -> dict[str, Any]:
        self.navigate_to(x, y, yaw)
        deadline = time.time() + max(0.5, float(timeout_sec))
        while time.time() < deadline:
            self._tick_nav()
            if not self._is_navigating:
                pose = self.get_nav_pose() or {}
                return {
                    "success": True,
                    "status": "SUCCEEDED",
                    "message": "mock arrived",
                    "pose": pose,
                    "goal": {"x": x, "y": y, "yaw": yaw},
                }
            time.sleep(0.05)
        self._is_navigating = False
        self._nav_goal = None
        return {
            "success": False,
            "status": "TIMEOUT",
            "message": "mock nav timeout",
        }

    def cancel_navigation(self) -> dict[str, Any]:
        self._is_navigating = False
        self._nav_goal = None
        return {"success": True, "message": "mock cancel ok"}

    def _tick_nav(self) -> None:
        if not self._is_navigating or not self._nav_goal:
            return
        # ~2s lerp to goal
        t = min(1.0, (time.time() - self._nav_goal_t0) / 2.0)
        g = self._nav_goal
        sx, sy, syaw = self._nav_pose["x"], self._nav_pose["y"], self._nav_pose["yaw"]
        # blend from start stored at goal time — approximate from current
        self._nav_pose = {
            "x": sx + (g["x"] - sx) * min(1.0, t + 0.15),
            "y": sy + (g["y"] - sy) * min(1.0, t + 0.15),
            "yaw": syaw + (g["yaw"] - syaw) * min(1.0, t + 0.15),
        }
        if t >= 1.0:
            self._nav_pose = dict(g)
            self._is_navigating = False
            self._nav_goal = None

    def get_actuator_state(self) -> dict[str, Any]:
        return {
            "led": self._led,
            "brightness": self._brightness,
            "emotion": self._emotion,
            "cmdVel": self._cmd_vel,
        }
