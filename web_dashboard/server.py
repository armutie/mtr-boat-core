from __future__ import annotations

import argparse
import json
import math
import random
import sys
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse


ROOT = Path(__file__).resolve().parent
REPO_ROOT = ROOT.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

HOST = "0.0.0.0"
PORT = 8080
LINUX_TTY_CANDIDATES = [f"/dev/ttyACM{i}" for i in range(3)] + [f"/dev/ttyUSB{i}" for i in range(3)]
WINDOWS_COM_CANDIDATES = [f"COM{i}" for i in range(1, 13)]


def default_mmwave() -> dict:
    return {
        "points": [],
        "raw_points": [],
        "filtered_points": [],
        "clusters": [],
        "zones": {"left": 0.0, "front": 0.0, "right": 0.0},
        "scores": {"left": 0.0, "front": 0.0, "right": 0.0},
        "command": "unavailable",
        "reason": "No mmWave feed connected.",
        "front_blocked": False,
        "frame_number": None,
        "point_count": {"raw": 0, "filtered": 0, "clusters": 0},
        "control": {
            "throttle": 0.0,
            "target_throttle": 0.0,
            "steering": 0.0,
            "target_steering": 0.0,
        },
        "age_s": None,
        "metadata": {},
    }


def default_gnss() -> dict:
    return {
        "lat": None,
        "lon": None,
        "speed_mps": None,
        "heading_deg": None,
        "fix": "unavailable",
        "satellites": 0,
        "hdop": None,
        "age_s": None,
        "source": None,
    }


def default_imu() -> dict:
    return {
        "accel_x_g": None,
        "accel_y_g": None,
        "accel_z_g": None,
        "accel_mag_g": None,
        "gyro_x_dps": None,
        "gyro_y_dps": None,
        "gyro_z_dps": None,
        "yaw_relative_deg": None,
        "dt_s": None,
        "age_s": None,
        "source": None,
        "bias": {"x_dps": 0.0, "y_dps": 0.0, "z_dps": 0.0},
    }


def base_snapshot(mode: str, started_at: float, log_paths: dict[str, str]) -> dict:
    now = time.time()
    return {
        "timestamp": now,
        "mode": mode,
        "session": {
            "started_at": started_at,
            "uptime_s": round(now - started_at, 2),
            "logging": bool(log_paths),
            "log_paths": log_paths,
        },
        "health": {
            "mmwave": "unavailable",
            "gnss": "unavailable",
            "imu": "unavailable",
        },
        "mmwave": default_mmwave(),
        "gnss": default_gnss(),
        "imu": default_imu(),
    }


class JsonlLog:
    def __init__(self, path: Path):
        self.path = path
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("a", encoding="utf-8")
        self._lock = threading.Lock()

    def write(self, record: dict) -> None:
        with self._lock:
            self._file.write(json.dumps(record, separators=(",", ":")) + "\n")
            self._file.flush()

    def close(self) -> None:
        with self._lock:
            self._file.close()


class DemoSensorState:
    def __init__(self, started_at: float):
        self.started_at = started_at
        self.rng = random.Random(42)
        self.lat = 43.47468
        self.lon = -80.53913

    def snapshot(self) -> dict:
        now = time.time()
        elapsed = now - self.started_at
        base = base_snapshot("demo", self.started_at, {})
        heading = (elapsed * 13.0) % 360.0
        speed = 0.75 + 0.25 * math.sin(elapsed / 5.0)
        self.lat += math.cos(elapsed / 13.0) * 0.0000015
        self.lon += math.sin(elapsed / 11.0) * 0.0000015

        raw_points = []
        filtered_points = []
        for _ in range(34):
            y = 0.25 + self.rng.random() * 2.55
            side_bias = math.sin(elapsed / 3.0) * 0.28
            x = self.rng.gauss(side_bias, 0.42)
            point = {
                "x": round(x, 3),
                "y": round(y, 3),
                "snr": round(80 + self.rng.random() * 230),
                "doppler": round(self.rng.uniform(-0.8, 0.8), 2),
            }
            raw_points.append(point)
            if -1.35 <= x <= 1.35:
                filtered_points.append(point)

        front = 0.24 + 0.48 * (0.5 + 0.5 * math.sin(elapsed / 4.0))
        left = 0.20 + 0.28 * (0.5 + 0.5 * math.sin(elapsed / 5.6 + 1.4))
        right = 0.18 + 0.34 * (0.5 + 0.5 * math.sin(elapsed / 4.8 + 2.0))
        steering = max(-1.0, min(1.0, left - right))
        throttle = 0.34 if front < 0.58 else 0.18
        command = "turn_left" if right > left + 0.12 else "turn_right" if left > right + 0.12 else "forward"

        base["health"] = {"mmwave": "live", "gnss": "live", "imu": "live"}
        base["mmwave"] = {
            **default_mmwave(),
            "points": filtered_points,
            "raw_points": raw_points,
            "filtered_points": filtered_points,
            "zones": {"left": round(left, 2), "front": round(front, 2), "right": round(right, 2)},
            "scores": {"left": round(left, 2), "front": round(front, 2), "right": round(right, 2)},
            "command": command,
            "reason": "Demo obstacle field.",
            "frame_number": int(elapsed * 8),
            "point_count": {"raw": len(raw_points), "filtered": len(filtered_points), "clusters": 0},
            "control": {
                "throttle": round(throttle, 3),
                "target_throttle": round(throttle, 3),
                "steering": round(steering, 3),
                "target_steering": round(steering, 3),
            },
        }
        base["gnss"] = {
            **default_gnss(),
            "lat": round(self.lat, 7),
            "lon": round(self.lon, 7),
            "speed_mps": round(speed, 2),
            "heading_deg": round(heading, 1),
            "fix": "fix",
            "satellites": 12,
            "hdop": round(0.74 + 0.08 * math.sin(elapsed / 7.0), 2),
            "age_s": 0.0,
            "source": "demo",
        }
        base["imu"] = {
            **default_imu(),
            "accel_x_g": round(0.18 * math.sin(elapsed / 2.0), 3),
            "accel_y_g": round(0.12 * math.cos(elapsed / 2.7), 3),
            "accel_z_g": round(0.98 + 0.03 * math.sin(elapsed / 1.8), 3),
            "accel_mag_g": round(1.0 + 0.03 * math.sin(elapsed / 1.8), 3),
            "gyro_x_dps": round(4.0 * math.sin(elapsed / 3.0), 2),
            "gyro_y_dps": round(3.0 * math.cos(elapsed / 4.0), 2),
            "gyro_z_dps": round(18.0 * math.sin(elapsed / 5.0), 2),
            "yaw_relative_deg": round(22.0 * math.sin(elapsed / 6.0), 1),
            "dt_s": 0.03,
            "age_s": 0.0,
            "source": "demo",
        }
        return base


class RosMmwaveState:
    def __init__(self, started_at: float, nav_state_topic: str = "radar/nav_state_json", stale_after_s: float = 2.0) -> None:
        self.started_at = started_at
        self.nav_state_topic = nav_state_topic
        self.stale_after_s = stale_after_s
        self._lock = threading.Lock()
        self._record: dict | None = None
        self._last_at: float | None = None
        self._error: str | None = None
        self._thread = threading.Thread(target=self._spin_ros, daemon=True)
        self._thread.start()

    def snapshot(self) -> dict:
        now = time.time()
        base = base_snapshot("ros", self.started_at, {})
        with self._lock:
            record = self._record
            last_at = self._last_at
            error = self._error

        if error:
            base["health"]["mmwave"] = "error"
            base["mmwave"]["reason"] = error
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
                self._error = f"ROS2 Python packages unavailable: {exc}"
            return

        class DashboardRosNode(Node):
            def __init__(node_self, outer: RosMmwaveState) -> None:
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
                self._error = f"ROS2 dashboard subscriber failed: {exc}"

    def _on_nav_state(self, msg) -> None:
        try:
            record = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._record = record
            self._last_at = time.time()
            self._error = None

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
            **default_mmwave(),
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
            "point_count": {"raw": len(raw_points), "filtered": len(filtered_points), "clusters": len(clusters)},
            "control": {
                "throttle": float(control.get("throttle", 0.0)),
                "target_throttle": float(control.get("target_throttle", 0.0)),
                "steering": float(control.get("steering", 0.0)),
                "target_steering": float(control.get("target_steering", 0.0)),
            },
            "age_s": age,
            "metadata": record.get("metadata", {}),
        }


class DirectMmwaveState:
    def __init__(
        self,
        started_at: float,
        cfg_port: str | None,
        cfg_file: str | None,
        data_port: str,
        baud: int,
        stale_after_s: float = 2.0,
        log=None,
    ) -> None:
        self.started_at = started_at
        self.cfg_port = cfg_port
        self.cfg_file = cfg_file
        self.data_port = data_port
        self.baud = baud
        self.stale_after_s = stale_after_s
        self.log = log
        usb_candidates = _platform_candidates([f"/dev/ttyUSB{i}" for i in range(3)])
        self.cfg_candidates = _serial_candidates(cfg_port, usb_candidates)
        self.data_candidates = _serial_candidates(data_port, usb_candidates)
        self._lock = threading.Lock()
        self._record: dict | None = None
        self._last_at: float | None = None
        self._error: str | None = None
        self._status = "starting"
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def snapshot(self) -> dict:
        now = time.time()
        base = base_snapshot("direct-mmwave", self.started_at, {})
        with self._lock:
            record = self._record
            last_at = self._last_at
            error = self._error
            status = self._status

        if error:
            base["health"]["mmwave"] = "error"
            base["mmwave"]["reason"] = error
            base["mmwave"]["metadata"] = self._source_metadata(status)
            return base
        if record is None:
            base["health"]["mmwave"] = "waiting"
            base["mmwave"]["reason"] = f"Waiting for radar frames on {self.data_port}."
            base["mmwave"]["metadata"] = self._source_metadata(status)
            return base

        stale = last_at is None or now - last_at > self.stale_after_s
        base["health"]["mmwave"] = "stale" if stale else "live"
        base["mmwave"] = self._mmwave_from_record(record, stale=stale, now=now, last_at=last_at)
        return base

    def _run(self) -> None:
        try:
            from mmwave_uart import MmwaveUartParser, send_cfg
            from radar_nav import RadarNavPipeline
            from radar_nav.logging import output_to_record
        except Exception as exc:
            with self._lock:
                self._error = f"Direct mmWave imports failed: {exc}"
            return

        parser = None
        try:
            if self.cfg_port and self.cfg_file:
                cfg_errors = []
                for candidate in self.cfg_candidates:
                    try:
                        with self._lock:
                            self._status = f"sending config on {candidate}"
                        send_cfg(candidate, self.cfg_file)
                        self.cfg_port = candidate
                        break
                    except Exception as exc:
                        cfg_errors.append(f"{candidate}: {exc}")
                else:
                    raise RuntimeError("Unable to send mmWave config on any CFG port: " + "; ".join(cfg_errors))
            elif self.cfg_port or self.cfg_file:
                with self._lock:
                    self._status = "config not sent"

            pipeline = RadarNavPipeline()
            data_errors = []
            for candidate in self.data_candidates:
                try:
                    parser = MmwaveUartParser(candidate, baud=self.baud)
                    self.data_port = candidate
                    break
                except Exception as exc:
                    data_errors.append(f"{candidate}: {exc}")
            if parser is None:
                raise RuntimeError("Unable to open mmWave DATA port: " + "; ".join(data_errors))
            with self._lock:
                self._status = f"reading {self.data_port} @ {self.baud}"
                self._error = None

            while True:
                decoded = parser.read_decoded_frame()
                if decoded is None:
                    continue
                output = pipeline.process_frame(decoded)
                record = output_to_record(output)
                if self.log is not None:
                    self.log.write(record)
                metadata = dict(record.get("metadata", {}))
                metadata.update(self._source_metadata("live"))
                record["metadata"] = metadata
                with self._lock:
                    self._record = record
                    self._last_at = time.time()
                    self._status = "live"
                    self._error = None
        except Exception as exc:
            with self._lock:
                self._error = f"Direct mmWave failed: {exc}"
                self._status = "error"
        finally:
            if parser is not None:
                parser.close()

    def _source_metadata(self, status: str) -> dict:
        return {
            "source": "direct-mmwave",
            "status": status,
            "cfg_port": self.cfg_port,
            "cfg_file": self.cfg_file,
            "data_port": self.data_port,
            "cfg_candidates": self.cfg_candidates,
            "data_candidates": self.data_candidates,
            "baud": self.baud,
        }

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
            reason = f"Last direct mmWave frame is stale ({age}s old)."

        return {
            **default_mmwave(),
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
            "point_count": {"raw": len(raw_points), "filtered": len(filtered_points), "clusters": len(clusters)},
            "control": {
                "throttle": float(control.get("throttle", 0.0)),
                "target_throttle": float(control.get("target_throttle", 0.0)),
                "steering": float(control.get("steering", 0.0)),
                "target_steering": float(control.get("target_steering", 0.0)),
            },
            "age_s": age,
            "metadata": record.get("metadata", {}),
        }


class UnavailableMmwaveState:
    def __init__(self, started_at: float):
        self.started_at = started_at

    def snapshot(self) -> dict:
        return base_snapshot("waiting", self.started_at, {})


class LiveGnssReader:
    def __init__(self, port: str, baud: int, log: JsonlLog | None):
        self.port = port
        self.baud = baud
        self.log = log
        self.candidates = _serial_candidates(port, _platform_candidates(LINUX_TTY_CANDIDATES))
        self._lock = threading.Lock()
        self._latest = default_gnss()
        self._last_at: float | None = None
        self._health = "waiting"
        self._error: str | None = None
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def snapshot(self) -> tuple[str, dict]:
        now = time.time()
        with self._lock:
            latest = dict(self._latest)
            health = self._health
            error = self._error
            last_at = self._last_at
        if error:
            latest["fix"] = "error"
            latest["source"] = self.port
            return "error", latest
        if last_at is None:
            return "waiting", latest
        latest["age_s"] = round(now - last_at, 2)
        if now - last_at > 3.0:
            return "stale", latest
        return health, latest

    def _run(self) -> None:
        from gnss import NmeaReader

        try:
            errors = []
            reader = None
            for candidate in self.candidates:
                try:
                    reader = NmeaReader(candidate, baud=self.baud)
                    self.port = candidate
                    break
                except Exception as exc:
                    errors.append(f"{candidate}: {exc}")
            if reader is None:
                raise RuntimeError("Unable to open GNSS port: " + "; ".join(errors))
            with reader:
                with self._lock:
                    self._health = "waiting"
                while True:
                    fix = reader.read_fix()
                    if fix is None:
                        continue
                    record = fix.to_record()
                    if self.log is not None:
                        self.log.write(record)
                    self._apply_fix(record)
        except Exception as exc:
            with self._lock:
                self._health = "error"
                self._error = str(exc)

    def _apply_fix(self, record: dict) -> None:
        with self._lock:
            latest = dict(self._latest)
            for key in ("lat", "lon", "speed_mps", "heading_deg", "altitude_m", "satellites", "hdop"):
                value = record.get(key)
                if value is not None:
                    if key in ("lat", "lon") and record.get("fix") == "none" and value == 0:
                        continue
                    latest[key] = value
            latest["fix"] = record.get("fix") or latest.get("fix") or "none"
            latest["source"] = record.get("source") or self.port
            latest["age_s"] = 0.0
            self._latest = latest
            self._last_at = time.time()
            self._health = "live" if latest["fix"] in ("fix", "estimated") else "waiting"


class LiveImuReader:
    def __init__(self, bus: int, address: int, log: JsonlLog | None, calibration_samples: int = 200):
        self.bus = bus
        self.address = address
        self.log = log
        self.calibration_samples = calibration_samples
        self._lock = threading.Lock()
        self._latest = default_imu()
        self._last_at: float | None = None
        self._health = "waiting"
        self._error: str | None = None
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def snapshot(self) -> tuple[str, dict]:
        now = time.time()
        with self._lock:
            latest = dict(self._latest)
            health = self._health
            error = self._error
            last_at = self._last_at
        if error:
            latest["source"] = f"i2c-{self.bus}:0x{self.address:02x}"
            latest["error"] = error
            return "error", latest
        if last_at is None:
            return "waiting", latest
        latest["age_s"] = round(now - last_at, 2)
        if now - last_at > 2.0:
            return "stale", latest
        return health, latest

    def _run(self) -> None:
        from imu import GyroBias, Mpu6050, RelativeYawTracker

        try:
            with Mpu6050(bus=self.bus, address=self.address) as imu:
                bias = imu.calibrate_gyro(samples=self.calibration_samples) if self.calibration_samples > 0 else GyroBias()
                yaw = RelativeYawTracker()
                with self._lock:
                    self._health = "live"
                while True:
                    sample = imu.read_sample(bias=bias)
                    yaw_relative_deg, dt_s = yaw.update(sample)
                    record = sample.to_record()
                    record["yaw_relative_deg"] = yaw_relative_deg
                    record["dt_s"] = dt_s
                    if self.log is not None:
                        self.log.write(record)
                    latest = {
                        **default_imu(),
                        **record,
                        "age_s": 0.0,
                        "bias": {"x_dps": bias.x_dps, "y_dps": bias.y_dps, "z_dps": bias.z_dps},
                    }
                    with self._lock:
                        self._latest = latest
                        self._last_at = time.time()
                        self._health = "live"
                    time.sleep(1 / 30)
        except Exception as exc:
            with self._lock:
                self._health = "error"
                self._error = str(exc)


class DashboardState:
    def __init__(self, mmwave_state, gnss_reader: LiveGnssReader | None, imu_reader: LiveImuReader | None, log_paths: dict[str, str]):
        self.mmwave_state = mmwave_state
        self.gnss_reader = gnss_reader
        self.imu_reader = imu_reader
        self.log_paths = log_paths

    def snapshot(self) -> dict:
        snapshot = self.mmwave_state.snapshot()
        snapshot["session"]["log_paths"] = self.log_paths
        snapshot["session"]["logging"] = bool(self.log_paths)
        if self.gnss_reader is not None:
            health, gnss = self.gnss_reader.snapshot()
            snapshot["health"]["gnss"] = health
            snapshot["gnss"] = gnss
        if self.imu_reader is not None:
            health, imu = self.imu_reader.snapshot()
            snapshot["health"]["imu"] = health
            snapshot["imu"] = imu
        return snapshot


STATE = DashboardState(UnavailableMmwaveState(time.time()), None, None, {})


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


def _parse_address(value: str | int) -> int:
    return value if isinstance(value, int) else int(value, 0)


def _serial_candidates(preferred: str | None, fallback: list[str] | None = None) -> list[str]:
    if fallback is None:
        fallback = WINDOWS_COM_CANDIDATES if sys.platform.startswith("win") else LINUX_TTY_CANDIDATES
    seen = set()
    candidates = []
    for port in [preferred, *fallback]:
        if port and port not in seen:
            seen.add(port)
            candidates.append(port)
    return candidates


def _platform_candidates(linux_candidates: list[str]) -> list[str]:
    return WINDOWS_COM_CANDIDATES if sys.platform.startswith("win") else linux_candidates


def _serial_device_present(preferred: str | None, fallback: list[str] | None = None) -> bool:
    if sys.platform.startswith("win"):
        return preferred is not None
    return any(Path(port).exists() for port in _serial_candidates(preferred, fallback))


def _i2c_device_present(bus: int) -> bool:
    if sys.platform.startswith("win"):
        return False
    return Path(f"/dev/i2c-{bus}").exists()


def _log_path(name: str) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return REPO_ROOT / "logs" / f"dashboard_{name}_{stamp}.jsonl"


def main() -> None:
    global STATE
    from boat_core.config import choose, load_boat_config, section

    started_at = time.time()
    parser = argparse.ArgumentParser(description="MTR realtime sensor dashboard")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--config", default="config/boat.local.json")
    parser.add_argument("--demo", action="store_true", help="Use simulated live sensor data")
    parser.add_argument("--ros", action="store_true", help="Subscribe to ROS2 mmWave topic")
    parser.add_argument("--direct-mmwave", action="store_true", help="Force direct mmWave UART mode")
    parser.add_argument("--no-mmwave", action="store_true", help="Do not auto-start direct mmWave UART mode")
    parser.add_argument("--mmwave-topic", default="radar/nav_state_json", help="ROS2 std_msgs/String mmWave nav topic")
    parser.add_argument("--cfg-port", help="mmWave CFG / CLI UART port, e.g. COM6 or /dev/ttyUSB0")
    parser.add_argument("--cfg-file", help="Path to TI mmWave .cfg file")
    parser.add_argument("--data-port", help="mmWave DATA UART port, e.g. COM5 or /dev/ttyUSB1")
    parser.add_argument("--baud", type=int, help="mmWave DATA UART baud")
    parser.add_argument("--stale-after-s", type=float, default=2.0, help="Seconds before a ROS feed is marked stale")
    parser.add_argument("--gnss", action="store_true", help="Force GNSS NMEA serial mode")
    parser.add_argument("--no-gnss", action="store_true", help="Do not auto-start GNSS serial mode")
    parser.add_argument("--gnss-port", help="GNSS serial port")
    parser.add_argument("--gnss-baud", type=int, help="GNSS serial baud")
    parser.add_argument("--imu", action="store_true", help="Force MPU-6050 I2C mode")
    parser.add_argument("--no-imu", action="store_true", help="Do not auto-start MPU-6050 I2C mode")
    parser.add_argument("--imu-bus", type=int, help="IMU I2C bus")
    parser.add_argument("--imu-address", help="IMU I2C address")
    parser.add_argument("--imu-calibration-samples", type=int, default=200)
    parser.add_argument("--log", action="store_true", help="Write live GNSS/IMU JSONL logs")
    args = parser.parse_args()

    config = load_boat_config(args.config)
    radar_config = section(config, "radar")
    gnss_config = section(config, "gnss")
    imu_config = section(config, "imu")
    cfg_port = choose(args.cfg_port, radar_config, "cfg_port")
    cfg_file = choose(args.cfg_file, radar_config, "cfg_file")
    data_port = choose(args.data_port, radar_config, "data_port")
    data_baud = choose(args.baud, radar_config, "baud", 921600)
    gnss_port = choose(args.gnss_port, gnss_config, "port")
    gnss_baud = choose(args.gnss_baud, gnss_config, "baud", 38400)
    imu_bus = choose(args.imu_bus, imu_config, "bus", 2)
    imu_address = _parse_address(choose(args.imu_address, imu_config, "address", "0x68"))
    auto_direct_mmwave = (
        not args.demo
        and not args.ros
        and not args.no_mmwave
        and bool(data_port)
        and _serial_device_present(data_port, _platform_candidates([f"/dev/ttyUSB{i}" for i in range(3)]))
    )
    use_direct_mmwave = args.direct_mmwave or auto_direct_mmwave
    auto_gnss = (
        not args.demo
        and not args.no_gnss
        and bool(gnss_port)
        and _serial_device_present(gnss_port, _platform_candidates(LINUX_TTY_CANDIDATES))
    )
    use_gnss = args.gnss or bool(args.gnss_port) or auto_gnss
    auto_imu = not args.demo and not args.no_imu
    use_imu = args.imu or auto_imu

    modes = [args.demo, args.ros, use_direct_mmwave]
    if sum(1 for enabled in modes if enabled) > 1:
        parser.error("--demo, --ros, and direct mmWave mode are mutually exclusive")

    if args.demo:
        mmwave_state = DemoSensorState(started_at)
    elif args.ros:
        mmwave_state = RosMmwaveState(started_at, nav_state_topic=args.mmwave_topic, stale_after_s=args.stale_after_s)
    elif use_direct_mmwave:
        if not data_port:
            parser.error("--data-port is required unless set in config")
        if bool(cfg_port) != bool(cfg_file):
            parser.error("--cfg-port and --cfg-file must be provided together")
        mmwave_log = JsonlLog(_log_path("mmwave")) if args.log else None
        log_paths: dict[str, str] = {}
        if mmwave_log is not None:
            log_paths["mmwave"] = str(mmwave_log.path)
        mmwave_state = DirectMmwaveState(
            started_at,
            cfg_port=cfg_port,
            cfg_file=cfg_file,
            data_port=data_port,
            baud=data_baud,
            stale_after_s=args.stale_after_s,
            log=mmwave_log,
        )
    else:
        mmwave_state = UnavailableMmwaveState(started_at)

    if not use_direct_mmwave:
        log_paths = {}
    gnss_reader = None
    imu_reader = None
    if use_gnss:
        if not gnss_port:
            parser.error("--gnss-port is required unless set in config")
        gnss_log = JsonlLog(_log_path("gnss")) if args.log else None
        if gnss_log is not None:
            log_paths["gnss"] = str(gnss_log.path)
        gnss_reader = LiveGnssReader(gnss_port, gnss_baud, gnss_log)
    if use_imu:
        imu_log = JsonlLog(_log_path("imu")) if args.log else None
        if imu_log is not None:
            log_paths["imu"] = str(imu_log.path)
        imu_reader = LiveImuReader(imu_bus, imu_address, imu_log, calibration_samples=args.imu_calibration_samples)

    STATE = DashboardState(mmwave_state, gnss_reader, imu_reader, log_paths)

    server = ThreadingHTTPServer((args.host, args.port), DashboardHandler)
    print(f"MTR dashboard: http://localhost:{args.port}")
    print(f"LAN dashboard: http://<orange-pi-or-laptop-ip>:{args.port}")
    if args.demo:
        print("Mode: demo")
    elif args.ros:
        print(f"Mode: ROS2 mmWave topic {args.mmwave_topic}")
    elif use_direct_mmwave:
        print(f"Mode: direct mmWave {data_port} @ {data_baud}")
        print(f"mmWave DATA candidates: {mmwave_state.data_candidates}")
        if cfg_port and cfg_file:
            print(f"mmWave config: {cfg_file} via {cfg_port}")
            print(f"mmWave CFG candidates: {mmwave_state.cfg_candidates}")
        if auto_direct_mmwave and not args.direct_mmwave:
            print("mmWave auto-started from detected serial device")
    else:
        print("Mode: waiting for live feeds")
    if gnss_reader is not None:
        print(f"GNSS: {gnss_port} @ {gnss_baud}")
        print(f"GNSS candidates: {gnss_reader.candidates}")
        if auto_gnss and not args.gnss and not args.gnss_port:
            print("GNSS auto-started from detected serial device")
    if imu_reader is not None:
        print(f"IMU: I2C bus {imu_bus}, address 0x{imu_address:02x}")
        if auto_imu and not args.imu:
            print("IMU auto-started from detected I2C device")
    if log_paths:
        print(f"Logging: {log_paths}")
    server.serve_forever()


if __name__ == "__main__":
    main()
