from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from imu import GyroBias
from imu.replay import load_imu_log, summarize_imu_log
from imu.viz import ImuViz


def print_summary(path: str, summary) -> None:
    print(f"[IMU replay] {path}")
    print(f"[IMU replay] samples={summary.sample_count} duration={summary.duration_s:.2f}s avg_rate={summary.average_hz:.1f} Hz")
    print(f"[IMU replay] accel_mag={summary.accel_mag_min_g:.3f}..{summary.accel_mag_max_g:.3f} g")
    print(
        "[IMU replay] gyro_avg_dps="
        f"({summary.gyro_x_avg_dps:+.3f},{summary.gyro_y_avg_dps:+.3f},{summary.gyro_z_avg_dps:+.3f})"
    )
    print(f"[IMU replay] integrated_z_yaw={summary.yaw_z_delta_deg:+.2f} deg")


def replay_viz(samples, speed: float) -> None:
    if not samples:
        return
    viz = ImuViz()
    bias = GyroBias()
    try:
        previous = samples[0]
        for sample in samples:
            dt = sample.timestamp - previous.timestamp
            if dt > 0:
                time.sleep(min(dt / max(speed, 0.1), 0.25))
            if not viz.update(sample, bias, clock_hz=120.0):
                break
            previous = sample
    finally:
        viz.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Replay an IMU JSONL log.")
    parser.add_argument("log_path", help="Path to logs/imu_*.jsonl")
    parser.add_argument("--no-viz", action="store_true", help="Only print the replay summary")
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier")
    args = parser.parse_args()

    samples = load_imu_log(args.log_path)
    summary = summarize_imu_log(samples)
    print_summary(args.log_path, summary)
    if not args.no_viz:
        replay_viz(samples, args.speed)


if __name__ == "__main__":
    main()
