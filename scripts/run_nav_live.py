import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from mmwave_uart import MmwaveUartParser, send_cfg
from radar_nav import NavConfig, NavOutput, RadarCluster, RadarNavPipeline
from radar_nav.logging import JsonlNavLogger
from radar_nav.pygame_viz import RadarPygameViz


def output_from_record(record: dict) -> NavOutput:
    current = record.get("current", {})
    scores = record.get("scores", {})
    control = record.get("control", {})
    return NavOutput(
        timestamp=record.get("timestamp", 0.0),
        frame_number=record.get("frame_number"),
        raw_points=record.get("raw_points", []),
        filtered_points=record.get("filtered_points", []),
        clusters=[RadarCluster(**cluster) for cluster in record.get("clusters", [])],
        current_left=current.get("left", 0.0),
        current_front=current.get("front", 0.0),
        current_right=current.get("right", 0.0),
        left_score=scores.get("left", 0.0),
        front_score=scores.get("front", 0.0),
        right_score=scores.get("right", 0.0),
        front_blocked=record.get("front_blocked", False),
        target_throttle=control.get("target_throttle", 0.0),
        target_steering=control.get("target_steering", 0.0),
        throttle=control.get("throttle", 0.0),
        steering=control.get("steering", 0.0),
        command=record.get("command", "stop"),
        desired_command=record.get("desired_command", record.get("command", "stop")),
        reason=record.get("reason", ""),
        metadata=record.get("metadata", {}),
    )


class RosNavSubscriber:
    def __init__(self, topic: str):
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import String
        except ImportError as exc:
            raise RuntimeError(
                "ROS mode requires ROS2 Python packages. Run this from a sourced ROS2 environment."
            ) from exc

        self.rclpy = rclpy
        self.latest_output = None
        self.node = Node("radar_nav_live_viewer")
        self.node.create_subscription(String, topic, self._on_nav_state, 10)
        self.node.get_logger().info(f"Subscribed to {topic}")

    def _on_nav_state(self, msg) -> None:
        try:
            self.latest_output = output_from_record(json.loads(msg.data))
        except Exception as exc:
            self.node.get_logger().warning(f"Failed to decode nav state: {exc}")

    def spin_once(self) -> NavOutput | None:
        self.rclpy.spin_once(self.node, timeout_sec=0.0)
        return self.latest_output

    def close(self) -> None:
        self.node.destroy_node()


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
        throttle_down_alpha=args.throttle_down_alpha,
        throttle_up_alpha=args.throttle_up_alpha,
        steering_alpha=args.steering_alpha,
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
    ap.add_argument("--throttle-down-alpha", type=float, default=0.18)
    ap.add_argument("--throttle-up-alpha", type=float, default=0.06)
    ap.add_argument("--steering-alpha", type=float, default=0.12)


def main() -> None:
    ap = argparse.ArgumentParser(description="Live AWR1843 radar navigation visualization")
    ap.add_argument("--ros", action="store_true", help="Subscribe to ROS2 nav_state_json instead of reading UART")
    ap.add_argument("--nav-state-topic", default="/radar/nav_state_json", help="ROS2 std_msgs/String nav state topic")
    ap.add_argument("--cfg-port", help="COM port for CFG / CLI UART, e.g. COM6")
    ap.add_argument("--cfg-file", help="Path to TI .cfg file")
    ap.add_argument("--data-port", help="COM port for DATA UART, e.g. COM5")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--log", action="store_true", help="Start with JSONL logging enabled")
    ap.add_argument("--log-path", help="Optional JSONL output path")
    add_nav_args(ap)
    args = ap.parse_args()

    if args.ros:
        run_ros_viewer(args)
        return

    if not args.data_port:
        ap.error("--data-port is required unless --ros is set")

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
                        f"min_snr_raw={cfg.min_snr_raw} block={cfg.front_on_thresh:.2f}/{cfg.front_off_thresh:.2f}"
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


def run_ros_viewer(args) -> None:
    try:
        import rclpy
    except ImportError as exc:
        raise RuntimeError("ROS mode requires ROS2 Python packages. Run from a sourced ROS2 environment.") from exc

    rclpy.init()
    cfg = build_config(args)
    cfg.clamp_values()
    viz = RadarPygameViz(cfg, width=args.width, height=args.height)
    subscriber = RosNavSubscriber(args.nav_state_topic)
    logger = JsonlNavLogger(args.log_path) if args.log else None
    last_output = None

    print("[LIVE:ROS] Keys: Q/ESC quit, P pause, L toggle log, G raw, S singletons.")
    print("[LIVE:ROS] Tuning keys only affect local visualization labels; the publishing node owns filtering.")
    try:
        running = True
        while running:
            running, actions = viz.handle_events()
            for action in actions:
                if action == "toggle_logging":
                    if logger is None:
                        logger = JsonlNavLogger(args.log_path)
                        print(f"[LIVE:ROS] Logging enabled: {logger.path}")
                    else:
                        print(f"[LIVE:ROS] Logging disabled: {logger.path}")
                        logger.close()
                        logger = None
                elif action == "reset":
                    print("[LIVE:ROS] Reset is ignored in subscriber mode; restart or reset the publisher node.")
                elif action == "config_changed":
                    print("[LIVE:ROS] Config changes are local only in subscriber mode.")

            if not viz.paused:
                last_output = subscriber.spin_once() or last_output
                if logger is not None and last_output is not None:
                    logger.write(last_output)

            viz.draw(last_output)
    finally:
        subscriber.close()
        if logger is not None:
            logger.close()
        viz.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
