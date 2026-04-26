import argparse

from mmwave_uart import MmwaveUartParser, send_cfg
from radar_nav import NavConfig, RadarNavPipeline
from radar_nav.logging import JsonlNavLogger
from radar_nav.pygame_viz import RadarPygameViz


def build_config(args) -> NavConfig:
    return NavConfig(
        min_y=args.min_y,
        max_y=args.max_y,
        lateral_limit=args.lateral_limit,
        min_snr_raw=args.min_snr_raw,
        cluster_eps_m=args.cluster_eps_m,
        front_half_width=args.front_half_width,
        alpha=args.alpha,
        front_on_thresh=args.front_on_thresh,
        front_off_thresh=args.front_off_thresh,
        command_lock_s=args.command_lock_s,
        emergency_stop_thresh=args.emergency_stop_thresh,
    )


def add_nav_args(ap: argparse.ArgumentParser) -> None:
    ap.add_argument("--min-y", type=float, default=0.15)
    ap.add_argument("--max-y", type=float, default=2.5)
    ap.add_argument("--lateral-limit", type=float, default=1.2)
    ap.add_argument("--min-snr-raw", type=int, default=120)
    ap.add_argument("--cluster-eps-m", type=float, default=0.35)
    ap.add_argument("--front-half-width", type=float, default=0.25)
    ap.add_argument("--alpha", type=float, default=0.10)
    ap.add_argument("--front-on-thresh", type=float, default=0.70)
    ap.add_argument("--front-off-thresh", type=float, default=0.40)
    ap.add_argument("--command-lock-s", type=float, default=0.35)
    ap.add_argument("--emergency-stop-thresh", type=float, default=0.90)


def main() -> None:
    ap = argparse.ArgumentParser(description="Live AWR1843 radar navigation visualization")
    ap.add_argument("--cfg-port", help="COM port for CFG / CLI UART, e.g. COM6")
    ap.add_argument("--cfg-file", help="Path to TI .cfg file")
    ap.add_argument("--data-port", required=True, help="COM port for DATA UART, e.g. COM5")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--log", action="store_true", help="Start with JSONL logging enabled")
    ap.add_argument("--log-path", help="Optional JSONL output path")
    add_nav_args(ap)
    args = ap.parse_args()

    cfg = build_config(args)
    cfg.clamp_values()

    if args.cfg_port and args.cfg_file:
        send_cfg(args.cfg_port, args.cfg_file)

    parser = MmwaveUartParser(args.data_port, baud=args.baud)
    pipeline = RadarNavPipeline(cfg)
    viz = RadarPygameViz(cfg, width=args.width, height=args.height)
    logger = JsonlNavLogger(args.log_path) if args.log else None
    last_output = None

    print("[LIVE] Keys: Q/ESC quit, R reset, P pause, L toggle log, arrows/[ ]/- = tune.")
    try:
        running = True
        while running:
            running, actions = viz.handle_events()
            for action in actions:
                if action == "reset":
                    pipeline.reset()
                    print("[LIVE] Pipeline state reset.")
                elif action == "toggle_logging":
                    if logger is None:
                        logger = JsonlNavLogger(args.log_path)
                        print(f"[LIVE] Logging enabled: {logger.path}")
                    else:
                        print(f"[LIVE] Logging disabled: {logger.path}")
                        logger.close()
                        logger = None
                elif action == "config_changed":
                    print(
                        "[LIVE] "
                        f"alpha={cfg.alpha:.2f} eps={cfg.cluster_eps_m:.2f} "
                        f"min_snr_raw={cfg.min_snr_raw} on/off={cfg.front_on_thresh:.2f}/{cfg.front_off_thresh:.2f}"
                    )

            if not viz.paused:
                decoded = parser.read_decoded_frame()
                if decoded is not None:
                    last_output = pipeline.process_frame(decoded)
                    if logger is not None:
                        logger.write(last_output)

            viz.draw(last_output)
    finally:
        parser.close()
        if logger is not None:
            logger.close()
        viz.close()


if __name__ == "__main__":
    main()
