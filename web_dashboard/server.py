from __future__ import annotations

import argparse
import json
import math
import random
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse


ROOT = Path(__file__).resolve().parent
HOST = "0.0.0.0"
PORT = 8080


class DemoSensorState:
    def __init__(self) -> None:
        self.started_at = time.time()
        self.rng = random.Random(42)
        self.lat = 43.6532
        self.lon = -79.3832

    def snapshot(self) -> dict:
        now = time.time()
        elapsed = now - self.started_at
        heading = (elapsed * 12.0) % 360.0
        speed = 0.65 + 0.18 * math.sin(elapsed / 5.0)
        self.lat += math.cos(elapsed / 13.0) * 0.000002
        self.lon += math.sin(elapsed / 11.0) * 0.000002

        radar_points = []
        for index in range(34):
            y = 0.25 + self.rng.random() * 2.55
            side_bias = math.sin(elapsed / 3.0) * 0.28
            x = self.rng.gauss(side_bias, 0.42)
            if -1.35 <= x <= 1.35:
                radar_points.append(
                    {
                        "x": round(x, 3),
                        "y": round(y, 3),
                        "snr": round(80 + self.rng.random() * 230),
                        "doppler": round(self.rng.uniform(-0.8, 0.8), 2),
                    }
                )

        front = 0.24 + 0.48 * (0.5 + 0.5 * math.sin(elapsed / 4.0))
        left = 0.20 + 0.28 * (0.5 + 0.5 * math.sin(elapsed / 5.6 + 1.4))
        right = 0.18 + 0.34 * (0.5 + 0.5 * math.sin(elapsed / 4.8 + 2.0))
        sonar = {
            "front": round(1.8 - front, 2),
            "left": round(2.3 - left, 2),
            "right": round(2.2 - right, 2),
            "rear": round(1.7 + 0.18 * math.sin(elapsed / 3.7), 2),
        }
        ultrasonic = {
            "bow": round(46 + 18 * math.sin(elapsed / 2.5), 1),
            "port": round(72 + 16 * math.sin(elapsed / 3.0 + 1.0), 1),
            "starboard": round(68 + 19 * math.sin(elapsed / 2.8 + 2.2), 1),
        }

        return {
            "timestamp": now,
            "mode": "demo",
            "health": {
                "mmwave": "live",
                "gnss": "live",
                "sonar": "live",
                "ultrasonic": "live",
            },
            "mmwave": {
                "points": radar_points,
                "zones": {
                    "left": round(left, 2),
                    "front": round(front, 2),
                    "right": round(right, 2),
                },
                "command": "turn_left" if right > left + 0.12 else "turn_right" if left > right + 0.12 else "forward",
            },
            "gnss": {
                "lat": round(self.lat, 7),
                "lon": round(self.lon, 7),
                "speed_mps": round(speed, 2),
                "heading_deg": round(heading, 1),
                "fix": "3D",
                "satellites": 14 + int(2 * math.sin(elapsed / 9.0)),
                "hdop": round(0.76 + 0.08 * math.sin(elapsed / 7.0), 2),
            },
            "sonar": sonar,
            "ultrasonic": ultrasonic,
        }


class UnavailableSensorState:
    def snapshot(self) -> dict:
        now = time.time()
        return {
            "timestamp": now,
            "mode": "waiting",
            "health": {
                "mmwave": "unavailable",
                "gnss": "unavailable",
                "sonar": "unavailable",
                "ultrasonic": "unavailable",
            },
            "mmwave": {
                "points": [],
                "raw_points": [],
                "filtered_points": [],
                "clusters": [],
                "zones": {"left": 0.0, "front": 0.0, "right": 0.0},
                "scores": {"left": 0.0, "front": 0.0, "right": 0.0},
                "command": "unavailable",
                "reason": "No mmWave feed connected.",
                "control": {
                    "throttle": 0.0,
                    "target_throttle": 0.0,
                    "steering": 0.0,
                    "target_steering": 0.0,
                },
            },
            "gnss": {
                "lat": None,
                "lon": None,
                "speed_mps": None,
                "heading_deg": None,
                "fix": "unavailable",
                "satellites": 0,
                "hdop": None,
            },
            "sonar": {},
            "ultrasonic": {},
        }


class RosSensorState:
    def __init__(self, nav_state_topic: str = "radar/nav_state_json", stale_after_s: float = 2.0) -> None:
        self.nav_state_topic = nav_state_topic
        self.stale_after_s = stale_after_s
        self._lock = threading.Lock()
        self._mmwave_record: dict | None = None
        self._last_mmwave_at: float | None = None
        self._ros_error: str | None = None
        self._thread = threading.Thread(target=self._spin_ros, daemon=True)
        self._thread.start()

    def snapshot(self) -> dict:
        now = time.time()
        with self._lock:
            record = self._mmwave_record
            last_at = self._last_mmwave_at
            ros_error = self._ros_error

        base = UnavailableSensorState().snapshot()
        base["mode"] = "ros"

        if ros_error:
            base["health"]["mmwave"] = "ros unavailable"
            base["mmwave"]["reason"] = ros_error
            return base

        if record is None:
            base["health"]["mmwave"] = "waiting"
            base["mmwave"]["reason"] = f"Waiting for ROS2 topic {self.nav_state_topic}."
            return base

        stale = last_at is None or now - last_at > self.stale_after_s
        base["health"]["mmwave"] = "stale" if stale else "live"
        base["mmwave"] = self._mmwave_from_record(record, stale=stale, now=now, last_at=last_at)
        return base

    def _spin_ros(self) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import String
        except ImportError as exc:
            with self._lock:
                self._ros_error = f"ROS2 Python packages unavailable: {exc}"
            return

        class DashboardRosNode(Node):
            def __init__(node_self, outer: RosSensorState) -> None:
                super().__init__("mtr_sensor_dashboard")
                node_self.create_subscription(String, outer.nav_state_topic, outer._on_nav_state, 10)
                node_self.get_logger().info(f"Dashboard subscribed to {outer.nav_state_topic}")

        try:
            rclpy.init(args=None)
            node = DashboardRosNode(self)
            try:
                rclpy.spin(node)
            finally:
                node.destroy_node()
                rclpy.shutdown()
        except Exception as exc:
            with self._lock:
                self._ros_error = f"ROS2 dashboard subscriber failed: {exc}"

    def _on_nav_state(self, msg) -> None:
        try:
            record = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._mmwave_record = record
            self._last_mmwave_at = time.time()
            self._ros_error = None

    def _mmwave_from_record(self, record: dict, stale: bool, now: float, last_at: float | None) -> dict:
        current = record.get("current", {})
        scores = record.get("scores", {})
        control = record.get("control", {})
        raw_points = record.get("raw_points", [])
        filtered_points = record.get("filtered_points", [])
        clusters = record.get("clusters", [])
        age = None if last_at is None else round(now - last_at, 2)
        reason = record.get("reason", "")
        if stale:
            reason = f"Last mmWave message is stale ({age}s old)."

        return {
            "points": filtered_points,
            "raw_points": raw_points,
            "filtered_points": filtered_points,
            "clusters": clusters,
            "zones": {
                "left": float(current.get("left", 0.0)),
                "front": float(current.get("front", 0.0)),
                "right": float(current.get("right", 0.0)),
            },
            "scores": {
                "left": float(scores.get("left", 0.0)),
                "front": float(scores.get("front", 0.0)),
                "right": float(scores.get("right", 0.0)),
            },
            "command": "stale" if stale else record.get("command", "unavailable"),
            "reason": reason,
            "front_blocked": bool(record.get("front_blocked", False)),
            "frame_number": record.get("frame_number"),
            "point_count": {
                "raw": len(raw_points),
                "filtered": len(filtered_points),
                "clusters": len(clusters),
            },
            "control": {
                "throttle": float(control.get("throttle", 0.0)),
                "target_throttle": float(control.get("target_throttle", 0.0)),
                "steering": float(control.get("steering", 0.0)),
                "target_steering": float(control.get("target_steering", 0.0)),
            },
            "age_s": age,
            "metadata": record.get("metadata", {}),
        }


STATE = UnavailableSensorState()


class DashboardHandler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path == "/events":
            self.stream_events()
            return
        if path == "/api/snapshot":
            self.write_json(STATE.snapshot())
            return
        if path == "/":
            path = "/index.html"

        file_path = (ROOT / path.lstrip("/")).resolve()
        if ROOT not in file_path.parents and file_path != ROOT:
            self.send_error(403)
            return
        if not file_path.exists() or not file_path.is_file():
            self.send_error(404)
            return

        content_type = "text/html"
        if file_path.suffix == ".css":
            content_type = "text/css"
        elif file_path.suffix == ".js":
            content_type = "application/javascript"
        self.send_response(200)
        self.send_header("Content-Type", f"{content_type}; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(file_path.read_bytes())

    def write_json(self, payload: dict) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def stream_events(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        while True:
            payload = json.dumps(STATE.snapshot(), separators=(",", ":"))
            try:
                self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                break
            time.sleep(0.25)

    def log_message(self, fmt: str, *args) -> None:
        return


def main() -> None:
    global STATE
    parser = argparse.ArgumentParser(description="MTR realtime sensor dashboard")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--demo", action="store_true", help="Use simulated live sensor data")
    parser.add_argument("--ros", action="store_true", help="Subscribe to ROS2 sensor topics")
    parser.add_argument("--mmwave-topic", default="radar/nav_state_json", help="ROS2 std_msgs/String mmWave nav topic")
    parser.add_argument("--stale-after-s", type=float, default=2.0, help="Seconds before a ROS feed is marked stale")
    args = parser.parse_args()
    if args.demo and args.ros:
        parser.error("--demo and --ros are mutually exclusive")
    if args.demo:
        STATE = DemoSensorState()
    elif args.ros:
        STATE = RosSensorState(nav_state_topic=args.mmwave_topic, stale_after_s=args.stale_after_s)

    server = ThreadingHTTPServer((args.host, args.port), DashboardHandler)
    print(f"Radar sensor dashboard: http://localhost:{args.port}")
    print(f"LAN dashboard: http://<orange-pi-or-laptop-ip>:{args.port}")
    if args.demo:
        print("Sensor mode: demo")
    elif args.ros:
        print(f"Sensor mode: ROS2, mmWave topic {args.mmwave_topic}")
    else:
        print("Sensor mode: waiting for real feeds")
    server.serve_forever()


if __name__ == "__main__":
    main()
