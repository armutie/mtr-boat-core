from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from .mpu6050 import ImuSample


@dataclass
class ImuReplaySummary:
    sample_count: int
    duration_s: float
    average_hz: float
    accel_mag_min_g: float
    accel_mag_max_g: float
    gyro_x_avg_dps: float
    gyro_y_avg_dps: float
    gyro_z_avg_dps: float
    yaw_z_delta_deg: float


def load_imu_log(path: str | Path) -> list[ImuSample]:
    samples: list[ImuSample] = []
    with Path(path).open("r", encoding="utf-8-sig") as file:
        for line in file:
            line = line.strip()
            if not line:
                continue
            record = json.loads(line)
            samples.append(ImuSample(**record))
    samples.sort(key=lambda sample: sample.timestamp)
    return samples


def integrate_yaw_z(samples: list[ImuSample]) -> float:
    yaw = 0.0
    previous: ImuSample | None = None
    for sample in samples:
        if previous is not None:
            dt = sample.timestamp - previous.timestamp
            if 0.0 < dt < 1.0:
                yaw += sample.gyro_z_dps * dt
        previous = sample
    return yaw


def summarize_imu_log(samples: list[ImuSample]) -> ImuReplaySummary:
    if not samples:
        return ImuReplaySummary(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    duration = max(0.0, samples[-1].timestamp - samples[0].timestamp)
    average_hz = (len(samples) - 1) / duration if duration > 0 and len(samples) > 1 else 0.0
    accel_mags = [sample.accel_mag_g for sample in samples]
    return ImuReplaySummary(
        sample_count=len(samples),
        duration_s=duration,
        average_hz=average_hz,
        accel_mag_min_g=min(accel_mags),
        accel_mag_max_g=max(accel_mags),
        gyro_x_avg_dps=sum(sample.gyro_x_dps for sample in samples) / len(samples),
        gyro_y_avg_dps=sum(sample.gyro_y_dps for sample in samples) / len(samples),
        gyro_z_avg_dps=sum(sample.gyro_z_dps for sample in samples) / len(samples),
        yaw_z_delta_deg=integrate_yaw_z(samples),
    )
