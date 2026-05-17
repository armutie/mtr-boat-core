from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass

import smbus2


@dataclass
class GyroBias:
    x_dps: float = 0.0
    y_dps: float = 0.0
    z_dps: float = 0.0


@dataclass
class ImuSample:
    timestamp: float
    source: str
    accel_x_g: float
    accel_y_g: float
    accel_z_g: float
    gyro_x_dps: float
    gyro_y_dps: float
    gyro_z_dps: float
    accel_mag_g: float

    def to_record(self) -> dict:
        return asdict(self)


class Mpu6050:
    PWR_MGMT_1 = 0x6B
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43

    def __init__(self, bus: int = 2, address: int = 0x68, source: str | None = None):
        self.bus_id = bus
        self.address = address
        self.source = source or f"i2c-{bus}:0x{address:02x}"
        self.bus = smbus2.SMBus(bus)
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0)

    def read_sample(self, bias: GyroBias | None = None) -> ImuSample:
        ax = self._read_word_2c(self.ACCEL_XOUT_H) / 16384.0
        ay = self._read_word_2c(self.ACCEL_XOUT_H + 2) / 16384.0
        az = self._read_word_2c(self.ACCEL_XOUT_H + 4) / 16384.0
        gx = self._read_word_2c(self.GYRO_XOUT_H) / 131.0
        gy = self._read_word_2c(self.GYRO_XOUT_H + 2) / 131.0
        gz = self._read_word_2c(self.GYRO_XOUT_H + 4) / 131.0

        if bias is not None:
            gx -= bias.x_dps
            gy -= bias.y_dps
            gz -= bias.z_dps

        return ImuSample(
            timestamp=time.time(),
            source=self.source,
            accel_x_g=ax,
            accel_y_g=ay,
            accel_z_g=az,
            gyro_x_dps=gx,
            gyro_y_dps=gy,
            gyro_z_dps=gz,
            accel_mag_g=math.sqrt(ax * ax + ay * ay + az * az),
        )

    def calibrate_gyro(self, samples: int = 200, delay_s: float = 0.01) -> GyroBias:
        gx_sum = gy_sum = gz_sum = 0.0
        for _ in range(samples):
            sample = self.read_sample()
            gx_sum += sample.gyro_x_dps
            gy_sum += sample.gyro_y_dps
            gz_sum += sample.gyro_z_dps
            time.sleep(delay_s)
        return GyroBias(gx_sum / samples, gy_sum / samples, gz_sum / samples)

    def close(self) -> None:
        self.bus.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def _read_word_2c(self, register: int) -> int:
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)
        value = (high << 8) | low
        if value >= 0x8000:
            value -= 65536
        return value
