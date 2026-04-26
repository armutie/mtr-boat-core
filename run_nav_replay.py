import argparse
import time

from radar_nav import NavConfig, RadarNavPipeline
from radar_nav.pygame_viz import RadarPygameViz
from radar_nav.replay import iter_log_records, record_to_decoded_frame


def main() -> None:
    ap = argparse.ArgumentParser(description="Replay a radar navigation JSONL log")
    ap.add_argument("--log", required=True, help="JSONL log created by run_nav_live.py")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier")
    args = ap.parse_args()

    records = list(iter_log_records(args.log))
    if not records:
        raise SystemExit("Replay log is empty.")

    cfg = NavConfig()
    pipeline = RadarNavPipeline(cfg)
    viz = RadarPygameViz(cfg, width=args.width, height=args.height)
    index = 0
    last_output = None
    last_timestamp = None
    next_due = time.time()

    print("[REPLAY] Keys: Q/ESC quit, P pause, Space step, R restart, arrows/[ ]/- = tune.")
    try:
        running = True
        while running:
            running, actions = viz.handle_events()
            if "reset" in actions:
                pipeline.reset()
                index = 0
                last_timestamp = None
                next_due = time.time()
                print("[REPLAY] Restarted.")

            step_requested = "step" in actions
            if index >= len(records):
                viz.paused = True

            should_advance = step_requested
            now = time.time()
            if not viz.paused and index < len(records) and now >= next_due:
                should_advance = True

            if should_advance and index < len(records):
                record = records[index]
                decoded = record_to_decoded_frame(record)
                record_time = record.get("timestamp")
                process_time = record_time if isinstance(record_time, (int, float)) else None
                last_output = pipeline.process_frame(decoded, now=process_time)
                index += 1

                if index < len(records):
                    next_time = records[index].get("timestamp")
                    if isinstance(record_time, (int, float)) and isinstance(next_time, (int, float)):
                        delay = max(0.0, (next_time - record_time) / max(args.speed, 0.01))
                    else:
                        delay = 0.05 / max(args.speed, 0.01)
                    next_due = time.time() + delay
                last_timestamp = record_time
            elif last_timestamp is None:
                last_timestamp = time.time()

            viz.draw(last_output)
    finally:
        viz.close()


if __name__ == "__main__":
    main()
