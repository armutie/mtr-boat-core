from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from imu import Bno055  # noqa: E402


def parse_address(value: str) -> int:
    return int(value, 0)


def quaternion_to_rpy(
    w: float,
    x: float,
    y: float,
    z: float,
) -> tuple[float, float, float]:
    """Return ROS-style roll, pitch, yaw in degrees from a unit quaternion."""

    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)

    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))

    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


def print_sample(elapsed_s: float, imu: Bno055, rate_hz: float) -> None:
    sample = imu.read_sample()
    status = imu.read_status()
    roll, pitch, yaw = quaternion_to_rpy(
        sample.orientation_w,
        sample.orientation_x,
        sample.orientation_y,
        sample.orientation_z,
    )
    magnetic_heading = (90.0 - yaw) % 360.0
    heading_ready = status.fully_calibrated
    calibration = (
        f"sys={status.system_calibration}/3 "
        f"gyro={status.gyroscope_calibration}/3 "
        f"acc={status.accelerometer_calibration}/3 "
        f"mag={status.magnetometer_calibration}/3"
    )
    absolute_state = (
        "ABSOLUTE HEADING READY" if heading_ready else "CALIBRATING"
    )

    print(
        f"\n[{elapsed_s:6.1f}s] {absolute_state} | {calibration} "
        f"| system={status.system_status} error={status.system_error}"
    )
    print(
        "  quaternion [w x y z] = "
        f"[{sample.orientation_w:+.4f} {sample.orientation_x:+.4f} "
        f"{sample.orientation_y:+.4f} {sample.orientation_z:+.4f}]"
    )
    print(
        "  RPY deg (ROS ENU, frame-dependent) = "
        f"roll {roll:+7.2f}  pitch {pitch:+7.2f}  yaw {yaw:+7.2f}"
    )
    print(
        "  compass heading approx = "
        f"{magnetic_heading:7.2f} deg "
        "(valid only after calibration and mounting setup)"
    )
    print(
        "  accel m/s² [incl. gravity] = "
        f"[{sample.acceleration_x_mps2:+6.2f} "
        f"{sample.acceleration_y_mps2:+6.2f} "
        f"{sample.acceleration_z_mps2:+6.2f}]"
    )
    print(
        "  linear m/s² [gravity removed] = "
        f"[{sample.linear_acceleration_x_mps2:+6.2f} "
        f"{sample.linear_acceleration_y_mps2:+6.2f} "
        f"{sample.linear_acceleration_z_mps2:+6.2f}]"
    )
    print(
        "  gyro rad/s = "
        f"[{sample.angular_velocity_x_rad_s:+6.3f} "
        f"{sample.angular_velocity_y_rad_s:+6.3f} "
        f"{sample.angular_velocity_z_rad_s:+6.3f}]"
    )
    print(
        "  magnetic µT = "
        f"[{sample.magnetic_field_x_t * 1e6:+7.2f} "
        f"{sample.magnetic_field_y_t * 1e6:+7.2f} "
        f"{sample.magnetic_field_z_t * 1e6:+7.2f}]  "
        f"temperature={sample.temperature_c:+.1f}°C"
    )
    time.sleep(1.0 / max(rate_hz, 0.1))


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Show live BNO055 orientation, axes, and calibration status."
        )
    )
    parser.add_argument("--bus", type=int, default=2, help="I2C bus number")
    parser.add_argument(
        "--address",
        type=parse_address,
        default=0x29,
        help="I2C address, for example 0x29",
    )
    parser.add_argument(
        "--placement", default="P1", choices=sorted(Bno055.PLACEMENTS)
    )
    parser.add_argument("--rate-hz", type=float, default=2.0)
    parser.add_argument(
        "--duration-s",
        type=float,
        default=0.0,
        help="Stop after this many seconds; zero runs until Ctrl-C",
    )
    parser.add_argument(
        "--no-reset",
        action="store_true",
        help=(
            "Preserve current volatile calibration instead of resetting "
            "the sensor"
        ),
    )
    args = parser.parse_args()

    print(f"[BNO055] Opening I2C bus {args.bus}, address 0x{args.address:02x}")
    if args.no_reset:
        print("[BNO055] Preserving current volatile calibration.")
    else:
        print(
            "[BNO055] Resetting sensor; keep it still, then calibrate it "
            "by moving it."
        )
    print(
        "[BNO055] Ctrl-C exits. Rotate slowly and watch "
        "quaternion/RPY/magnetic values change."
    )

    started = time.monotonic()
    with Bno055(
        bus=args.bus,
        address=args.address,
        placement=args.placement,
        reset_on_start=not args.no_reset,
    ) as imu:
        while (
            args.duration_s <= 0.0
            or time.monotonic() - started < args.duration_s
        ):
            elapsed_s = time.monotonic() - started
            try:
                print_sample(elapsed_s, imu, args.rate_hz)
            except KeyboardInterrupt:
                break

    print("[BNO055] Done.")


if __name__ == "__main__":
    main()
