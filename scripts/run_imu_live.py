from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from boat_core.config import choose, load_boat_config, section
from imu import GyroBias, ImuSample, Mpu6050
from imu.viz import ImuViz


def apply_config(args) -> None:
    config = load_boat_config(args.config)
    imu = section(config, "imu")
    args.bus = choose(args.bus, imu, "bus", 2)
    args.address = choose(args.address, imu, "address", "0x68")


def parse_address(value: int | str) -> int:
    if isinstance(value, int):
        return value
    return int(value, 0)


def default_log_path() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("logs") / f"imu_{stamp}.jsonl"


def print_sample(sample: ImuSample) -> None:
    print(
        "[IMU] "
        f"accel_g=({sample.accel_x_g:+.3f},{sample.accel_y_g:+.3f},{sample.accel_z_g:+.3f}) "
        f"|a|={sample.accel_mag_g:.3f} "
        f"gyro_dps=({sample.gyro_x_dps:+.2f},{sample.gyro_y_dps:+.2f},{sample.gyro_z_dps:+.2f})"
    )


def main() -> None:
    ap = argparse.ArgumentParser(description="Read MPU-6050 IMU data over I2C.")
    ap.add_argument("--config", default="config/boat.local.json", help="Boat config JSON path")
    ap.add_argument("--bus", type=int, help="I2C bus number")
    ap.add_argument("--address", help="MPU-6050 I2C address, e.g. 0x68")
    ap.add_argument("--rate-hz", type=float, default=10.0, help="Terminal/log sample rate")
    ap.add_argument("--calibration-samples", type=int, default=200, help="Stationary gyro samples for zero calibration")
    ap.add_argument("--no-calibrate", action="store_true", help="Skip startup gyro calibration")
    ap.add_argument("--log", action="store_true", help="Write parsed IMU samples to logs/ as JSONL")
    ap.add_argument("--log-path", help="Custom JSONL log path")
    ap.add_argument("--viz", action="store_true", help="Show the pygame IMU visualization")
    args = ap.parse_args()
    apply_config(args)
    args.address = parse_address(args.address)

    log_file = None
    if args.log or args.log_path:
        path = Path(args.log_path) if args.log_path else default_log_path()
        path.parent.mkdir(parents=True, exist_ok=True)
        log_file = path.open("a", encoding="utf-8")
        print(f"[IMU] Logging to {path}")

    delay_s = 1.0 / max(args.rate_hz, 0.1)
    viz = ImuViz() if args.viz else None

    print(f"[IMU] Opening I2C bus {args.bus}, address 0x{args.address:02x}. Ctrl+C to exit.")
    try:
        with Mpu6050(bus=args.bus, address=args.address) as imu:
            if args.no_calibrate:
                bias = GyroBias()
            else:
                print("[IMU] Keep MPU-6050 still. Calibrating gyro...")
                bias = imu.calibrate_gyro(samples=args.calibration_samples)
                print(f"[IMU] Gyro bias gx={bias.x_dps:.2f}, gy={bias.y_dps:.2f}, gz={bias.z_dps:.2f} dps")

            while True:
                sample = imu.read_sample(bias=bias)
                if log_file is not None:
                    log_file.write(json.dumps(sample.to_record(), separators=(",", ":")) + "\n")
                    log_file.flush()
                if viz is not None:
                    if not viz.update(sample, bias):
                        break
                else:
                    print_sample(sample)
                    time.sleep(delay_s)
    except KeyboardInterrupt:
        print("\n[IMU] Stopped.")
    finally:
        if viz is not None:
            viz.close()
        if log_file is not None:
            log_file.close()


if __name__ == "__main__":
    main()
