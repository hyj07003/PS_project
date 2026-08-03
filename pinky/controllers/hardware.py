from __future__ import annotations

import math
import os
import struct
import time
from dataclasses import dataclass
from typing import Any


@dataclass
class BatteryReading:
    percent: float | None
    voltage: float | None
    source: str


@dataclass
class ImuReading:
    orientation: tuple[float, float, float, float]  # x,y,z,w
    angular_velocity: tuple[float, float, float]
    linear_acceleration: tuple[float, float, float]
    source: str


@dataclass
class UltrasonicReading:
    range_m: float | None
    ir_raw: list[int]
    source: str


class HardwareSensors:
    """
    로봇 하드웨어에서 센서 값을 읽는다.
    우선순위: pinkylib → smbus I2C → (실패 시 None, publisher가 skip/폴백).
    """

    def __init__(self) -> None:
        self._battery_lib = None
        self._smbus = None
        self._adc_bus: Any = None
        self._imu_bus: Any = None
        self._log: list[str] = []

        try:
            from pinkylib import Battery

            self._battery_lib = Battery()
            self._log.append("pinkylib.Battery ok")
        except Exception as exc:
            self._log.append(f"pinkylib.Battery unavailable: {exc}")

        try:
            from smbus2 import SMBus

            self._smbus = SMBus
            adc_dev = os.environ.get("PINKY_ADC_I2C", "/dev/i2c-1")
            imu_dev = os.environ.get("PINKY_IMU_I2C", "/dev/i2c-0")
            adc_num = int(adc_dev.replace("/dev/i2c-", ""))
            imu_num = int(imu_dev.replace("/dev/i2c-", ""))
            try:
                self._adc_bus = SMBus(adc_num)
                self._log.append(f"ADC I2C bus {adc_num} ok")
            except Exception as exc:
                self._log.append(f"ADC I2C open failed: {exc}")
            try:
                self._imu_bus = SMBus(imu_num)
                self._log.append(f"IMU I2C bus {imu_num} ok")
            except Exception as exc:
                self._log.append(f"IMU I2C open failed: {exc}")
        except Exception as exc:
            self._log.append(f"smbus2 unavailable: {exc}")

    @property
    def status(self) -> list[str]:
        return list(self._log)

    def read_battery(self) -> BatteryReading:
        if self._battery_lib is not None:
            try:
                voltage = float(self._battery_lib.get_voltage())
                percent = float(self._battery_lib.battery_percentage())
                return BatteryReading(percent, voltage, "pinkylib")
            except Exception:
                pass

        # ADC ch battery (pinky_sensor_adc 동일 식)
        adc = self._read_adc_channel(0xF8)
        if adc is not None:
            voltage = (adc / 4096.0) * 4.096 / (13.0 / 28.0)
            percent = min(100.0, max(0.0, (voltage - 6.5) / (8.4 - 6.5) * 100.0))
            return BatteryReading(round(percent, 1), round(voltage, 3), "adc")

        return BatteryReading(None, None, "unavailable")

    def read_ultrasonic(self) -> UltrasonicReading:
        us_adc = self._read_adc_channel(0xD8)
        ir = [
            self._read_adc_channel(0x98),
            self._read_adc_channel(0xC8),
            self._read_adc_channel(0x88),
        ]
        if us_adc is None and all(v is None for v in ir):
            return UltrasonicReading(None, [], "unavailable")

        range_m = None
        if us_adc is not None:
            range_m = max(0.02, 1.0 * (us_adc / 4096.0) - 0.03)
        ir_raw = [int(v) for v in ir if v is not None]
        return UltrasonicReading(range_m, ir_raw, "adc")

    def read_imu(self) -> ImuReading | None:
        """BNO055 최소 읽기 (quaternion + gyro + accel). 실패 시 None."""
        if self._imu_bus is None:
            return None
        addr = int(os.environ.get("PINKY_IMU_ADDR", "0x28"), 0)
        try:
            # Operation mode NDOF = 0x0C at OPR_MODE 0x3D
            self._imu_bus.write_byte_data(addr, 0x3D, 0x0C)
            time.sleep(0.02)
            # Quaternion wxyz at 0x20 (8 bytes, LSB units 1/16384)
            q = self._imu_bus.read_i2c_block_data(addr, 0x20, 8)
            w, x, y, z = struct.unpack("<hhhh", bytes(q))
            scale_q = 1.0 / 16384.0
            # Gyro at 0x14 (rad/s after scale 1/16 dps → rad)
            g = self._imu_bus.read_i2c_block_data(addr, 0x14, 6)
            gx, gy, gz = struct.unpack("<hhh", bytes(g))
            dps = 1.0 / 16.0
            to_rad = math.pi / 180.0
            # Lin accel 0x28
            a = self._imu_bus.read_i2c_block_data(addr, 0x28, 6)
            ax, ay, az = struct.unpack("<hhh", bytes(a))
            acc_scale = 1.0 / 100.0  # m/s^2
            return ImuReading(
                orientation=(x * scale_q, y * scale_q, z * scale_q, w * scale_q),
                angular_velocity=(
                    gx * dps * to_rad,
                    gy * dps * to_rad,
                    gz * dps * to_rad,
                ),
                linear_acceleration=(ax * acc_scale, ay * acc_scale, az * acc_scale),
                source="bno055",
            )
        except Exception:
            return None

    def _read_adc_channel(self, register: int) -> int | None:
        if self._adc_bus is None:
            return None
        addr = int(os.environ.get("PINKY_ADC_ADDR", "0x08"), 0)
        try:
            self._adc_bus.write_byte(addr, register)
            time.sleep(0.006)
            data = self._adc_bus.read_i2c_block_data(addr, 0, 2)
            if len(data) < 2:
                return None
            return (data[0] << 4) + (data[1] >> 4)
        except Exception:
            return None
