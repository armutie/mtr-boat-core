from __future__ import annotations

import math
import time
from dataclasses import dataclass

from gnss.geo import normalize_angle_deg


@dataclass
class HeadingEstimatorConfig:
    imu_stale_s: float = 2.0
    command_stale_s: float = 0.75
    min_forward_us: int = 1565
    max_thruster_delta_us: int = 20
    min_speed_mps: float = 0.3
    learn_duration_s: float = 1.0
    learn_min_samples: int = 5
    course_stability_deg: float = 10.0
    course_agreement_deg: float = 45.0
    correction_blend: float = 0.02


class HeadingEstimator:
    """Anchor BNO yaw to world heading only during known straight-ahead motion."""

    def __init__(
        self,
        config: HeadingEstimatorConfig | None = None,
        clock=None,
    ) -> None:
        self.config = config or HeadingEstimatorConfig()
        self._clock = clock or time.monotonic
        self._offset_deg: float | None = None
        self._candidate_started_at: float | None = None
        self._candidate_offsets: list[float] = []
        self._ever_locked = False
        self._recovery_count: int | None = None

    def reset(self) -> None:
        self._offset_deg = None
        self._candidate_started_at = None
        self._candidate_offsets = []

    def update(
        self,
        gnss: dict,
        imu_health: str,
        imu: dict,
        command_health: str,
        command: dict,
    ) -> dict:
        now = float(self._clock())
        imu_yaw, imu_reason = self._imu_yaw(imu_health, imu)
        gnss_heading, speed = self._gnss_course(gnss)
        command_ok, command_reason = self._straight_forward_command(
            command_health,
            command,
        )
        recovered = self._observe_recovery(imu)
        course_ok = (
            gnss_heading is not None
            and speed is not None
            and speed >= self.config.min_speed_mps
        )
        eligible = imu_yaw is not None and command_ok and course_ok
        correction_accepted = False
        course_error = None

        if imu_yaw is None:
            self._clear_candidate()
            state = "degraded" if self._ever_locked else "uninitialized"
            reason = imu_reason
            heading = None
        elif self._offset_deg is None:
            heading = None
            if not eligible:
                self._clear_candidate()
                state = "provisional"
                reason = (
                    "BNO recovered; repeat the straight manual run"
                    if recovered
                    else self._learning_reason(
                        command_ok,
                        command_reason,
                        gnss_heading,
                        speed,
                    )
                )
            else:
                new_offset = normalize_angle_deg(gnss_heading - imu_yaw)
                if self._candidate_is_unstable(new_offset):
                    self._candidate_started_at = now
                    self._candidate_offsets = [new_offset]
                    reason = "course changed; restarting straight-run sample"
                else:
                    if self._candidate_started_at is None:
                        self._candidate_started_at = now
                    self._candidate_offsets.append(new_offset)
                    reason = "learning heading from straight forward motion"

                elapsed = now - self._candidate_started_at
                if (
                    elapsed >= self.config.learn_duration_s
                    and len(self._candidate_offsets)
                    >= self.config.learn_min_samples
                ):
                    self._offset_deg = self._circular_mean(
                        self._candidate_offsets
                    )
                    self._ever_locked = True
                    heading = (imu_yaw + self._offset_deg) % 360.0
                    state = "locked"
                    reason = "heading locked to BNO yaw"
                    self._clear_candidate()
                else:
                    state = "learning"
        else:
            heading = (imu_yaw + self._offset_deg) % 360.0
            state = "locked"
            reason = "tracking BNO yaw"
            if eligible:
                course_error = normalize_angle_deg(gnss_heading - heading)
                if abs(course_error) <= self.config.course_agreement_deg:
                    blend = max(
                        0.0,
                        min(1.0, self.config.correction_blend),
                    )
                    self._offset_deg = normalize_angle_deg(
                        self._offset_deg + course_error * blend
                    )
                    heading = (imu_yaw + self._offset_deg) % 360.0
                    correction_accepted = True
                    reason = "tracking BNO yaw; GNSS trim accepted"
                else:
                    reason = "tracking BNO yaw; conflicting GNSS course ignored"

        calibration = imu.get("calibration")
        learning_elapsed = (
            0.0
            if self._candidate_started_at is None
            else max(0.0, now - self._candidate_started_at)
        )
        return {
            "state": state,
            "reason": reason,
            "heading_deg": self._rounded(heading, 2),
            "source": "bno055" if self._offset_deg is not None else "uninitialized",
            "confidence": (
                "locked"
                if state == "locked"
                else "degraded"
                if state == "degraded"
                else "learning"
                if state == "learning"
                else "none"
            ),
            "imu_yaw_relative_deg": self._rounded(imu_yaw, 2),
            "gnss_heading_deg": self._rounded(gnss_heading, 2),
            "speed_mps": self._rounded(speed, 3),
            "calibration": calibration if isinstance(calibration, dict) else {},
            "command": {
                "left_us": self._int_or_none(command.get("left_us")),
                "right_us": self._int_or_none(command.get("right_us")),
                "age_s": self._rounded(command.get("age_s"), 2),
                "health": command_health,
                "eligible": command_ok,
                "reason": command_reason,
            },
            "learning": {
                "eligible": eligible,
                "samples": len(self._candidate_offsets),
                "elapsed_s": round(learning_elapsed, 2),
                "required_s": self.config.learn_duration_s,
            },
            "gnss_correction": {
                "accepted": correction_accepted,
                "course_error_deg": self._rounded(course_error, 2),
            },
        }

    def _imu_yaw(
        self,
        health: str,
        imu: dict,
    ) -> tuple[float | None, str]:
        if health != "live":
            return None, f"IMU {health}"
        age = self._float_or_none(imu.get("age_s"))
        if age is not None and age > self.config.imu_stale_s:
            return None, "IMU stale"
        calibration = imu.get("calibration")
        if (
            isinstance(calibration, dict)
            and calibration.get("status") == "error"
        ):
            return None, "IMU diagnostic error"
        yaw = self._float_or_none(imu.get("yaw_relative_deg"))
        if yaw is None:
            return None, "BNO fused orientation unavailable"
        return yaw % 360.0, "IMU ok"

    @staticmethod
    def _gnss_course(gnss: dict) -> tuple[float | None, float | None]:
        heading = HeadingEstimator._float_or_none(gnss.get("heading_deg"))
        speed = HeadingEstimator._float_or_none(gnss.get("speed_mps"))
        return (
            None if heading is None else heading % 360.0,
            speed,
        )

    def _straight_forward_command(
        self,
        health: str,
        command: dict,
    ) -> tuple[bool, str]:
        if health != "live":
            return False, f"thruster command {health}"
        age = self._float_or_none(command.get("age_s"))
        if age is not None and age > self.config.command_stale_s:
            return False, "thruster command stale"
        left = self._int_or_none(command.get("left_us"))
        right = self._int_or_none(command.get("right_us"))
        if left is None or right is None:
            return False, "thruster command unavailable"
        if (
            left < self.config.min_forward_us
            or right < self.config.min_forward_us
        ):
            return False, "waiting for forward thrust"
        if abs(left - right) > self.config.max_thruster_delta_us:
            return False, "waiting for nearly straight thrust"
        return True, "straight forward thrust"

    def _learning_reason(
        self,
        command_ok: bool,
        command_reason: str,
        gnss_heading: float | None,
        speed: float | None,
    ) -> str:
        if not command_ok:
            return command_reason
        if gnss_heading is None:
            return "waiting for GNSS course"
        if speed is None or speed < self.config.min_speed_mps:
            return "waiting for sufficient GNSS speed"
        return "waiting for a stable straight run"

    def _candidate_is_unstable(self, new_offset: float) -> bool:
        if not self._candidate_offsets:
            return False
        mean = self._circular_mean(self._candidate_offsets)
        return (
            abs(normalize_angle_deg(new_offset - mean))
            > self.config.course_stability_deg
        )

    def _observe_recovery(self, imu: dict) -> bool:
        calibration = imu.get("calibration")
        if not isinstance(calibration, dict):
            return False
        count = self._int_or_none(calibration.get("recovery_count"))
        if count is None:
            return False
        if self._recovery_count is None:
            self._recovery_count = count
            return False
        if count == self._recovery_count:
            return False
        self._recovery_count = count
        self.reset()
        return True

    def _clear_candidate(self) -> None:
        self._candidate_started_at = None
        self._candidate_offsets = []

    @staticmethod
    def _circular_mean(values: list[float]) -> float:
        x = sum(math.cos(math.radians(value)) for value in values)
        y = sum(math.sin(math.radians(value)) for value in values)
        return normalize_angle_deg(math.degrees(math.atan2(y, x)))

    @staticmethod
    def _float_or_none(value) -> float | None:
        if value is None:
            return None
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        return number if math.isfinite(number) else None

    @staticmethod
    def _int_or_none(value) -> int | None:
        number = HeadingEstimator._float_or_none(value)
        return None if number is None else int(round(number))

    @staticmethod
    def _rounded(value, digits: int) -> float | None:
        number = HeadingEstimator._float_or_none(value)
        return None if number is None else round(number, digits)
