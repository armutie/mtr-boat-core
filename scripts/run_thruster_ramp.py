from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from radar_nav.esp32_bridge import Esp32ThrusterSerial


def main() -> None:
    ap = argparse.ArgumentParser(description="Slow ESP32 thruster PWM ramp tester")
    ap.add_argument("--esp32-port", required=True, help="ESP32 serial port, e.g. COM3")
    ap.add_argument("--esp32-baud", type=int, default=115200)
    ap.add_argument("--neutral-us", type=int, default=1500)
    ap.add_argument("--start-us", type=int, default=1510)
    ap.add_argument("--end-us", type=int, default=1600)
    ap.add_argument("--step-us", type=int, default=5)
    ap.add_argument("--hold-s", type=float, default=2.0)
    ap.add_argument("--neutral-hold-s", type=float, default=5.0)
    ap.add_argument("--down", action="store_true", help="Ramp downward from neutral instead of upward")
    args = ap.parse_args()

    if args.step_us <= 0:
        ap.error("--step-us must be positive")
    if args.hold_s <= 0:
        ap.error("--hold-s must be positive")

    direction = -1 if args.down else 1
    start = args.start_us
    end = args.end_us
    if args.down and start > args.neutral_us:
        start = args.neutral_us - abs(start - args.neutral_us)
    if args.down and end > args.neutral_us:
        end = args.neutral_us - abs(end - args.neutral_us)

    writer = Esp32ThrusterSerial(args.esp32_port, args.esp32_baud)
    try:
        print(f"[RAMP] Neutral {args.neutral_us} us for {args.neutral_hold_s:.1f}s")
        writer.send_pwm(args.neutral_us)
        time.sleep(args.neutral_hold_s)

        pwm = start
        while (direction > 0 and pwm <= end) or (direction < 0 and pwm >= end):
            print(f"[RAMP] PWM {pwm} us")
            writer.send_pwm(pwm)
            time.sleep(args.hold_s)
            pwm += direction * args.step_us

        print(f"[RAMP] Back to neutral {args.neutral_us} us")
        writer.send_pwm(args.neutral_us)
    finally:
        writer.stop()
        writer.close()


if __name__ == "__main__":
    main()
