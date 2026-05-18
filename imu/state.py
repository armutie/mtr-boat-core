from __future__ import annotations

from dataclasses import dataclass

from .mpu6050 import ImuSample


@dataclass
class RelativeYawTracker:
    yaw_deg: float = 0.0
    last_timestamp: float | None = None

    def update(self, sample: ImuSample) -> tuple[float, float]:
        if self.last_timestamp is None:
            self.last_timestamp = sample.timestamp
            return self.yaw_deg, 0.0

        dt = sample.timestamp - self.last_timestamp
        self.last_timestamp = sample.timestamp
        if 0.0 < dt < 1.0:
            self.yaw_deg += sample.gyro_z_dps * dt
        return self.yaw_deg, dt
