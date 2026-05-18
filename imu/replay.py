from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from .mpu6050 import ImuSample


@dataclass
class IntegratedAxis:
    final_deg: float
    min_deg: float
    max_deg: float
    range_deg: float


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
    integrated_x_deg: IntegratedAxis
    integrated_y_deg: IntegratedAxis
    integrated_z_deg: IntegratedAxis


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


def integrate_axis(samples: list[ImuSample], axis: str) -> IntegratedAxis:
    angle = 0.0
    min_angle = 0.0
    max_angle = 0.0
    previous: ImuSample | None = None
    for sample in samples:
        if previous is not None:
            dt = sample.timestamp - previous.timestamp
            if 0.0 < dt < 1.0:
                angle += getattr(sample, f"gyro_{axis}_dps") * dt
                min_angle = min(min_angle, angle)
                max_angle = max(max_angle, angle)
        previous = sample
    return IntegratedAxis(
        final_deg=angle,
        min_deg=min_angle,
        max_deg=max_angle,
        range_deg=max_angle - min_angle,
    )


def summarize_imu_log(samples: list[ImuSample]) -> ImuReplaySummary:
    if not samples:
        empty_axis = IntegratedAxis(0.0, 0.0, 0.0, 0.0)
        return ImuReplaySummary(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, empty_axis, empty_axis, empty_axis)

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
        integrated_x_deg=integrate_axis(samples, "x"),
        integrated_y_deg=integrate_axis(samples, "y"),
        integrated_z_deg=integrate_axis(samples, "z"),
    )
