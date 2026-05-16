from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from mmwave_uart import MmwaveUartParser, send_cfg
from radar_nav import RadarNavPipeline
from radar_nav.pygame_viz import RadarPygameViz
from scripts.run_nav_live import add_nav_args, build_config
from thruster_control import Esp32ThrusterSerial, ThrusterMapping, nav_output_to_thruster


def add_thruster_args(ap: argparse.ArgumentParser) -> None:
    ap.add_argument("--esp32-port", required=True, help="ESP32 serial port, e.g. /dev/ttyACM0 or COM3")
    ap.add_argument("--esp32-baud", type=int, default=115200)
    ap.add_argument("--dry-run", action="store_true", help="Print commands without writing to ESP32")
    ap.add_argument("--send-hz", type=float, default=5.0, help="Max command send rate")
    ap.add_argument("--stale-timeout-s", type=float, default=1.0, help="Neutral if no fresh nav output arrives")
    ap.add_argument("--viz", action="store_true", help="Show the pygame radar navigation visualization")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--neutral-us", type=int, default=1500)
    ap.add_argument("--forward-min-us", type=int, default=1520)
    ap.add_argument("--forward-max-us", type=int, default=1600, help="Gentle default forward output")
    ap.add_argument("--hard-min-us", type=int, default=1350)
    ap.add_argument("--hard-max-us", type=int, default=2000)
    ap.add_argument("--steering-slowdown", type=float, default=0.35)


def build_mapping(args) -> ThrusterMapping:
    return ThrusterMapping(
        neutral_us=args.neutral_us,
        forward_min_us=args.forward_min_us,
        forward_max_us=args.forward_max_us,
        hard_min_us=args.hard_min_us,
        hard_max_us=args.hard_max_us,
        steering_slowdown=args.steering_slowdown,
    )


def send_or_print(writer: Esp32ThrusterSerial | None, dry_run: bool, pwm_us: int, reason: str) -> None:
    if dry_run:
        print(f"[DRY] PWM {pwm_us}  {reason}")
        return
    assert writer is not None
    if writer.last_pwm_us != pwm_us:
        print(f"[ESP32] PWM {pwm_us}  {reason}")
    writer.send_pwm(pwm_us)


def run_direct_bridge(args) -> None:
    cfg = build_config(args)
    cfg.clamp_values()

    if args.cfg_port and args.cfg_file:
        send_cfg(args.cfg_port, args.cfg_file)

    parser = MmwaveUartParser(args.data_port, baud=args.baud)
    pipeline = RadarNavPipeline(cfg)
    writer = None if args.dry_run else Esp32ThrusterSerial(args.esp32_port, args.esp32_baud)
    viz = RadarPygameViz(cfg, width=args.width, height=args.height) if args.viz else None
    mapping = build_mapping(args)
    min_period_s = 1.0 / max(args.send_hz, 0.1)
    last_send = 0.0
    last_output = None
    last_output_time = 0.0

    try:
        print("[DIRECT] Reading radar UART and sending ESP32 thruster commands. Ctrl+C sends STOP/neutral.")
        running = True
        while running:
            if viz is not None:
                running, actions = viz.handle_events()
                for action in actions:
                    if action == "reset":
                        pipeline.reset()
                        print("[DIRECT] Pipeline state reset.")
                    elif action == "config_changed":
                        print(
                            "[DIRECT] "
                            f"alpha={cfg.alpha:.2f} eps={cfg.cluster_eps_m:.2f} "
                            f"min_snr_raw={cfg.min_snr_raw} block={cfg.front_on_thresh:.2f}/{cfg.front_off_thresh:.2f}"
                        )

            decoded = None if viz is not None and viz.paused else parser.read_decoded_frame()
            now = time.monotonic()
            if decoded is not None:
                last_output = pipeline.process_frame(decoded)
                last_output_time = now

            if now - last_send < min_period_s:
                continue

            stale = last_output is None or now - last_output_time > args.stale_timeout_s
            command = nav_output_to_thruster(None if stale else last_output, mapping)
            send_or_print(writer, args.dry_run, command.pwm_us, "stale" if stale else command.reason)
            last_send = now

            if viz is not None:
                viz.draw(last_output)
    finally:
        if writer is not None:
            writer.stop()
            writer.close()
        parser.close()
        if viz is not None:
            viz.close()


def main() -> None:
    ap = argparse.ArgumentParser(description="Direct radar UART to ESP32 thruster bridge for bench/field tests")
    ap.add_argument("--cfg-port", help="COM port for CFG / CLI UART, e.g. COM6 or /dev/ttyUSB0")
    ap.add_argument("--cfg-file", help="Path to TI .cfg file")
    ap.add_argument("--data-port", required=True, help="COM port for DATA UART, e.g. COM5 or /dev/ttyUSB1")
    ap.add_argument("--baud", type=int, default=921600)
    add_nav_args(ap)
    add_thruster_args(ap)
    args = ap.parse_args()
    run_direct_bridge(args)


if __name__ == "__main__":
    main()
