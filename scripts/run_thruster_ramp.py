from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from boat_core.config import choose, load_boat_config, section
from thruster_control import Esp32ThrusterSerial


def apply_config(args) -> None:
    config = load_boat_config(args.config)
    esp32 = section(config, "esp32")
    thruster = section(config, "thruster")
    ramp = section(config, "ramp")
    runtime = section(config, "runtime")

    args.esp32_port = choose(args.esp32_port, esp32, "port")
    args.esp32_baud = choose(args.esp32_baud, esp32, "baud", 115200)
    args.neutral_us = choose(args.neutral_us, thruster, "neutral_us", 1500)
    args.start_us = choose(args.start_us, ramp, "start_us", 1510)
    args.end_us = choose(args.end_us, ramp, "end_us", 1600)
    args.step_us = choose(args.step_us, ramp, "step_us", 5)
    args.hold_s = choose(args.hold_s, ramp, "hold_s", 2.0)
    args.neutral_hold_s = choose(args.neutral_hold_s, ramp, "neutral_hold_s", 5.0)
    args.send_hz = choose(args.send_hz, runtime, "send_hz", 20.0)


def pwm_pair(channel: str, pwm_us: int, neutral_us: int) -> tuple[int, int]:
    if channel == "left":
        return pwm_us, neutral_us
    if channel == "right":
        return neutral_us, pwm_us
    return pwm_us, pwm_us


def send_selected(
    writer: Esp32ThrusterSerial,
    channel: str,
    pwm_us: int,
    neutral_us: int,
) -> str:
    left_us, right_us = pwm_pair(channel, pwm_us, neutral_us)
    response = writer.send_pwm_pair(left_us, right_us)
    if response is None:
        raise RuntimeError("ESP32 did not acknowledge the PWM pair")
    if response.startswith("ERR "):
        raise RuntimeError(response)
    return response


def hold_pwm(
    writer: Esp32ThrusterSerial,
    channel: str,
    pwm_us: int,
    neutral_us: int,
    duration_s: float,
    send_hz: float,
    *,
    clock=time.monotonic,
    sleep=time.sleep,
) -> None:
    period_s = 1.0 / max(1.0, send_hz)
    deadline = clock() + max(0.0, duration_s)
    while clock() < deadline:
        send_selected(writer, channel, pwm_us, neutral_us)
        sleep(min(period_s, max(0.0, deadline - clock())))


def main() -> None:
    ap = argparse.ArgumentParser(description="Slow ESP32 thruster PWM ramp tester")
    ap.add_argument("--config", default="config/boat.local.json", help="Boat config JSON path")
    ap.add_argument("--esp32-port", help="ESP32 serial port, e.g. /dev/ttyACM0 or COM3")
    ap.add_argument("--esp32-baud", type=int)
    ap.add_argument("--neutral-us", type=int)
    ap.add_argument("--start-us", type=int)
    ap.add_argument("--end-us", type=int)
    ap.add_argument("--step-us", type=int)
    ap.add_argument("--hold-s", type=float)
    ap.add_argument("--neutral-hold-s", type=float)
    ap.add_argument("--send-hz", type=float)
    ap.add_argument(
        "--channel",
        choices=("left", "right", "both"),
        default="both",
        help="Thruster channel to ramp; the other channel stays neutral",
    )
    ap.add_argument("--down", action="store_true", help="Ramp downward from neutral instead of upward")
    args = ap.parse_args()
    apply_config(args)

    if not args.esp32_port:
        ap.error("--esp32-port is required unless set in --config")

    if args.step_us <= 0:
        ap.error("--step-us must be positive")
    if args.hold_s <= 0:
        ap.error("--hold-s must be positive")
    if args.send_hz <= 0:
        ap.error("--send-hz must be positive")

    direction = -1 if args.down else 1
    start = args.start_us
    end = args.end_us
    if args.down and start > args.neutral_us:
        start = args.neutral_us - abs(start - args.neutral_us)
    if args.down and end > args.neutral_us:
        end = args.neutral_us - abs(end - args.neutral_us)
    if args.down and start < end:
        ap.error("downward ramp requires start-us >= end-us")
    if not args.down and start > end:
        ap.error("upward ramp requires start-us <= end-us")

    writer = Esp32ThrusterSerial(args.esp32_port, args.esp32_baud)
    try:
        identity = writer.probe_dual_firmware()
        if identity is None:
            raise RuntimeError(
                "dual-thruster firmware did not answer PING"
            )
        print(f"[RAMP] Firmware: {identity}")
        print(
            f"[RAMP] Channel: {args.channel}; "
            f"heartbeat: {args.send_hz:.1f} Hz"
        )
        print(
            f"[RAMP] Neutral {args.neutral_us} us for "
            f"{args.neutral_hold_s:.1f}s"
        )
        hold_pwm(
            writer,
            "both",
            args.neutral_us,
            args.neutral_us,
            args.neutral_hold_s,
            args.send_hz,
        )

        pwm = start
        while (direction > 0 and pwm <= end) or (direction < 0 and pwm >= end):
            left_us, right_us = pwm_pair(
                args.channel,
                pwm,
                args.neutral_us,
            )
            print(
                f"[RAMP] PWM L{left_us} R{right_us} "
                f"for {args.hold_s:.1f}s"
            )
            hold_pwm(
                writer,
                args.channel,
                pwm,
                args.neutral_us,
                args.hold_s,
                args.send_hz,
            )
            pwm += direction * args.step_us

        print(f"[RAMP] Back to neutral {args.neutral_us} us")
        send_selected(
            writer,
            "both",
            args.neutral_us,
            args.neutral_us,
        )
    finally:
        writer.stop()
        writer.close()


if __name__ == "__main__":
    main()
