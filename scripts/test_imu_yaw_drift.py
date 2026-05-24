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
from imu import GyroBias, Mpu6050, RelativeYawTracker


def parse_address(value: int | str) -> int:
    return value if isinstance(value, int) else int(value, 0)


def default_log_path() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("logs") / f"imu_yaw_drift_{stamp}.jsonl"


def phase_for_elapsed(elapsed_s: float, hold_s: float, rotate_s: float, return_s: float) -> str:
    if elapsed_s < hold_s:
        return "hold_still"
    if elapsed_s < hold_s + rotate_s:
        return "rotate_90_then_hold"
    if elapsed_s < hold_s + rotate_s + return_s:
        return "return_to_start"
    return "final_hold"


def main() -> None:
    parser = argparse.ArgumentParser(description="Measure MPU-6050 yaw drift and rotate/return error.")
    parser.add_argument("--config", default="config/boat.local.json")
    parser.add_argument("--bus", type=int)
    parser.add_argument("--address")
    parser.add_argument("--calibration-samples", type=int, default=400)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--hold-s", type=float, default=120.0, help="Initial still phase duration")
    parser.add_argument("--rotate-s", type=float, default=30.0, help="Time window to rotate about 90 deg and hold")
    parser.add_argument("--return-s", type=float, default=30.0, help="Time window to rotate back to start")
    parser.add_argument("--final-hold-s", type=float, default=60.0)
    parser.add_argument("--log-path")
    args = parser.parse_args()

    config = load_boat_config(args.config)
    imu_config = section(config, "imu")
    bus = choose(args.bus, imu_config, "bus", 2)
    address = parse_address(choose(args.address, imu_config, "address", "0x68"))
    delay_s = 1.0 / max(args.rate_hz, 1.0)
    total_s = args.hold_s + args.rotate_s + args.return_s + args.final_hold_s
    log_path = Path(args.log_path) if args.log_path else default_log_path()
    log_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"[IMU drift] Opening I2C bus {bus}, address 0x{address:02x}")
    print(f"[IMU drift] Keep sensor still for gyro calibration ({args.calibration_samples} samples).")
    print(f"[IMU drift] Logging to {log_path}")

    with Mpu6050(bus=bus, address=address) as imu, log_path.open("a", encoding="utf-8") as log:
        bias = imu.calibrate_gyro(samples=args.calibration_samples) if args.calibration_samples > 0 else GyroBias()
        yaw = RelativeYawTracker()
        print(f"[IMU drift] Bias gx={bias.x_dps:+.3f}, gy={bias.y_dps:+.3f}, gz={bias.z_dps:+.3f} dps")
        print("[IMU drift] Phase 1: keep still.")
        print("[IMU drift] Later prompts: rotate about 90 deg, then rotate back to the original orientation.")

        started = time.time()
        last_print_phase = ""
        while True:
            now = time.time()
            elapsed = now - started
            if elapsed > total_s:
                break

            phase = phase_for_elapsed(elapsed, args.hold_s, args.rotate_s, args.return_s)
            if phase != last_print_phase:
                print(f"[IMU drift] Phase: {phase}")
                last_print_phase = phase

            sample = imu.read_sample(bias=bias)
            yaw_relative_deg, dt_s = yaw.update(sample)
            record = sample.to_record()
            record.update({
                "elapsed_s": round(elapsed, 3),
                "phase": phase,
                "yaw_relative_deg": yaw_relative_deg,
                "dt_s": dt_s,
                "bias": {"x_dps": bias.x_dps, "y_dps": bias.y_dps, "z_dps": bias.z_dps},
            })
            log.write(json.dumps(record, separators=(",", ":")) + "\n")
            log.flush()

            if int(elapsed * 2) != int((elapsed - delay_s) * 2):
                drift_rate = yaw_relative_deg / max(elapsed / 60.0, 1e-6)
                print(
                    f"[IMU drift] t={elapsed:6.1f}s phase={phase:18s} "
                    f"yaw={yaw_relative_deg:+8.2f} deg drift={drift_rate:+7.2f} deg/min "
                    f"gz={sample.gyro_z_dps:+6.3f} dps"
                )
            time.sleep(delay_s)

    print("[IMU drift] Done.")
    print(f"[IMU drift] Review log: {log_path}")


if __name__ == "__main__":
    main()
