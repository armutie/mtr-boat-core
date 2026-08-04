from __future__ import annotations

import argparse
import json
import math
import random
import socket
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

from web_dashboard.gnss_ros import RosGnssReader
from web_dashboard.imu_ros import RosImuReader, empty_imu_record


HOST = "0.0.0.0"
PORT = 8080
DASHBOARD_BUILD = "auto-navigator-v1"
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
    return empty_imu_record()


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
            "build": DASHBOARD_BUILD,
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
            "accel_x_mps2": round(1.76 * math.sin(elapsed / 2.0), 3),
            "accel_y_mps2": round(1.18 * math.cos(elapsed / 2.7), 3),
            "accel_z_mps2": round(9.61 + 0.29 * math.sin(elapsed / 1.8), 3),
            "accel_mag_mps2": round(9.81 + 0.29 * math.sin(elapsed / 1.8), 3),
            "linear_accel_x_mps2": round(0.3 * math.sin(elapsed / 2.0), 3),
            "linear_accel_y_mps2": round(0.2 * math.cos(elapsed / 2.7), 3),
            "linear_accel_z_mps2": round(0.08 * math.sin(elapsed / 1.8), 3),
            "gravity_x_mps2": 0.0,
            "gravity_y_mps2": 0.0,
            "gravity_z_mps2": 9.807,
            "gyro_x_rad_s": round(0.07 * math.sin(elapsed / 3.0), 3),
            "gyro_y_rad_s": round(0.05 * math.cos(elapsed / 4.0), 3),
            "gyro_z_rad_s": round(0.31 * math.sin(elapsed / 5.0), 3),
            "roll_deg": round(7.0 * math.sin(elapsed / 4.0), 1),
            "pitch_deg": round(10.0 * math.sin(elapsed / 5.0), 1),
            "yaw_deg": round((elapsed * 8.0) % 360.0, 1),
            "orientation_available": True,
            "mag_x_ut": round(18.0 + 2.0 * math.sin(elapsed / 4.0), 1),
            "mag_y_ut": round(-4.0 + math.cos(elapsed / 3.0), 1),
            "mag_z_ut": round(42.0 + 1.5 * math.sin(elapsed / 6.0), 1),
            "mag_strength_ut": 45.9,
            "temperature_c": 27.0,
            "calibration": {
                "system": 3,
                "gyroscope": 3,
                "accelerometer": 3,
                "magnetometer": 3,
                "status": "ready",
                "message": "fully calibrated",
            },
            "frame_id": "imu_link",
            "accel_x_g": round(0.18 * math.sin(elapsed / 2.0), 3),
            "accel_y_g": round(0.12 * math.cos(elapsed / 2.7), 3),
            "accel_z_g": round(0.98 + 0.03 * math.sin(elapsed / 1.8), 3),
            "accel_mag_g": round(1.0 + 0.03 * math.sin(elapsed / 1.8), 3),
            "gyro_x_dps": round(4.0 * math.sin(elapsed / 3.0), 2),
            "gyro_y_dps": round(3.0 * math.cos(elapsed / 4.0), 2),
            "gyro_z_dps": round(18.0 * math.sin(elapsed / 5.0), 2),
            "yaw_relative_deg": round((elapsed * 8.0) % 360.0, 1),
            "dt_s": 0.03,
            "age_s": 0.0,
            "source": "demo BNO055",
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
        self._stop_event = threading.Event()
        self._context = None
        self._thread = threading.Thread(target=self._spin_ros, daemon=True)
        self._thread.start()

    def shutdown(self) -> None:
        self._stop_event.set()
        context = self._context
        if context is not None and context.ok():
            context.shutdown()
        self._thread.join(timeout=2.0)

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
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from std_msgs.msg import String
        except ImportError as exc:
            with self._lock:
                self._error = f"ROS2 Python packages unavailable: {exc}"
            return

        class DashboardRosNode(Node):
            def __init__(
                node_self,
                outer: RosMmwaveState,
                **kwargs,
            ) -> None:
                super().__init__("mtr_sensor_dashboard", **kwargs)
                node_self.create_subscription(String, outer.nav_state_topic, outer._on_nav_state, 10)
                node_self.get_logger().info(f"Dashboard subscribed to {outer.nav_state_topic}")

        context = rclpy.context.Context()
        self._context = context
        node = None
        executor = None
        try:
            rclpy.init(context=context)
            if not self._stop_event.is_set():
                node = DashboardRosNode(self, context=context)
                executor = SingleThreadedExecutor(context=context)
                rclpy.spin(node, executor=executor)
        except Exception as exc:
            if not self._stop_event.is_set():
                with self._lock:
                    self._error = f"ROS2 dashboard subscriber failed: {exc}"
        finally:
            if executor is not None:
                executor.shutdown()
            if node is not None:
                node.destroy_node()
            if context.ok():
                rclpy.shutdown(context=context)
            self._context = None

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
                    acceleration = tuple(
                        float(record[key])
                        for key in ("accel_x_g", "accel_y_g", "accel_z_g")
                    )
                    angular = tuple(
                        float(record[key])
                        for key in (
                            "gyro_x_dps",
                            "gyro_y_dps",
                            "gyro_z_dps",
                        )
                    )
                    record.update(
                        {
                            "accel_x_mps2": acceleration[0] * 9.80665,
                            "accel_y_mps2": acceleration[1] * 9.80665,
                            "accel_z_mps2": acceleration[2] * 9.80665,
                            "accel_mag_mps2": math.sqrt(
                                sum(value * value for value in acceleration)
                            )
                            * 9.80665,
                            "gyro_x_rad_s": math.radians(angular[0]),
                            "gyro_y_rad_s": math.radians(angular[1]),
                            "gyro_z_rad_s": math.radians(angular[2]),
                            "source": (
                                f"legacy MPU-6050 i2c-{self.bus}:"
                                f"0x{self.address:02x}"
                            ),
                        }
                    )
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
    def __init__(
        self,
        mmwave_state,
        gnss_reader: "LiveGnssReader | RosGnssReader | None",
        imu_reader: "LiveImuReader | RosImuReader | None",
        log_paths: dict[str, str],
        manual_slew_per_s: float = 0.0,
        pivot_turn_start: float = 0.75,
        pivot_reverse_ratio: float = 1.0,
    ):
        self.mmwave_state = mmwave_state
        self.gnss_reader = gnss_reader
        self.imu_reader = imu_reader
        self.log_paths = log_paths
        self.control = ControlState(
            manual_slew_per_s=manual_slew_per_s,
            pivot_turn_start=pivot_turn_start,
            pivot_reverse_ratio=pivot_reverse_ratio,
        )
        self.actuator = None

    def snapshot(self) -> dict:
        snapshot = self.mmwave_state.snapshot()
        snapshot["session"]["log_paths"] = self.log_paths
        snapshot["session"]["logging"] = bool(self.log_paths)
        snapshot["control"] = self.control.snapshot()
        if self.actuator is not None:
            snapshot["control"]["actuator"] = self.actuator.status_label()
            snapshot["control"]["effective"] = self.actuator.effective_output()
        else:
            snapshot["control"]["actuator"] = "dry-run"
            snapshot["control"]["effective"] = None
        if self.gnss_reader is not None:
            health, gnss = self.gnss_reader.snapshot()
            snapshot["health"]["gnss"] = health
            snapshot["gnss"] = gnss
        if self.imu_reader is not None:
            health, imu = self.imu_reader.snapshot()
            snapshot["health"]["imu"] = health
            snapshot["imu"] = imu
        return snapshot


class SessionLogger:
    """Fixed-rate unified dashboard logger.

    Each row is the same high-level snapshot shape the browser receives:
    health, GNSS, IMU, mmWave, control intent, and effective
    actuator output. Raw per-sensor logs can still be enabled separately with
    --log when deeper parser debugging is needed.
    """

    def __init__(self, state: DashboardState, path: Path, hz: float = 10.0):
        self.state = state
        self.hz = max(1.0, hz)
        self._log = JsonlLog(path)
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name="session-logger")

    @property
    def path(self) -> Path:
        return self._log.path

    def start(self) -> None:
        self._thread.start()

    def shutdown(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=1.0)
        try:
            self._log.close()
        except Exception:
            pass

    def _run(self) -> None:
        period = 1.0 / self.hz
        while not self._stop_event.is_set():
            started = time.monotonic()
            try:
                snapshot = self.state.snapshot()
                snapshot["schema"] = "dashboard_session_v1"
                self._log.write(snapshot)
            except Exception as exc:
                self._log.write({
                    "schema": "dashboard_session_v1",
                    "timestamp": time.time(),
                    "error": f"session logger failed: {exc}",
                })
            elapsed = time.monotonic() - started
            if self._stop_event.wait(max(0.0, period - elapsed)):
                break


VALID_MODES = ("off", "manual", "auto")
AUTO_ARM_CHECK = None


class ControlInputError(ValueError):
    pass


class ControlState:
    """Tracks the operator intent: drive mode and (when in manual) the latest
    throttle/steering payload from the browser.

    Acts as the single source of truth for the actuator bridge to read from.
    Has no knowledge of the actuator itself - it just records intent and a
    timestamp. Staleness, slew, and PWM mapping happen downstream.
    """

    def __init__(
        self,
        stale_after_s: float = 0.45,
        manual_slew_per_s: float = 0.0,
        pivot_turn_start: float = 0.75,
        pivot_reverse_ratio: float = 1.0,
    ) -> None:
        self.stale_after_s = stale_after_s
        self.manual_slew_per_s = max(0.0, manual_slew_per_s)
        self.pivot_turn_start = max(0.0, min(0.99, pivot_turn_start))
        self.pivot_reverse_ratio = max(
            0.0,
            min(1.0, pivot_reverse_ratio),
        )
        self._lock = threading.Lock()
        self.mode = "off"
        self.throttle = 0.0
        self.steering = 0.0
        self.auto_left_us: int | None = None
        self.auto_right_us: int | None = None
        self.last_update_at: float | None = None
        self.reason = "off"
        self.auto_status = {"state": "idle", "reason": "auto not armed"}

    @staticmethod
    def _coerce_float(value, name: str, default: float) -> float:
        if value is None:
            return default
        try:
            return float(value)
        except (TypeError, ValueError) as exc:
            raise ControlInputError(f"{name} must be a number") from exc

    def apply(self, payload: dict) -> dict:
        if not isinstance(payload, dict):
            raise ControlInputError("payload must be a JSON object")

        with self._lock:
            current_mode = self.mode
            current_enabled = current_mode == "manual"
            current_throttle = self.throttle
            current_steering = self.steering

        if "enabled" in payload:
            try:
                enabled = bool(payload["enabled"])
            except (TypeError, ValueError) as exc:
                raise ControlInputError("enabled must be boolean") from exc
        else:
            enabled = current_enabled

        throttle = self._coerce_float(payload.get("throttle"), "throttle", current_throttle)
        steering = self._coerce_float(payload.get("steering"), "steering", current_steering)

        throttle = max(0.0, min(1.0, throttle))
        steering = max(-1.0, min(1.0, steering))

        with self._lock:
            touched = True
            # /api/control/manual is throttle/steering-only. It cannot enter
            # manual mode (use /api/control/mode for that) so a stale heartbeat
            # racing after set_mode("off") can never re-arm the boat. It can
            # still cooperatively disarm by sending enabled=false. When auto is
            # armed, browser manual heartbeats are ignored entirely.
            if not enabled and self.mode == "manual":
                self.mode = "off"
                self.throttle = 0.0
                self.steering = 0.0
                self.reason = "manual disabled"
            elif self.mode == "manual" and enabled:
                self.throttle = throttle
                self.steering = steering
                self.reason = "manual command"
            else:
                self.throttle = 0.0
                self.steering = 0.0
                if self.mode == "off":
                    self.reason = "off"
                elif self.mode == "auto":
                    touched = False
            if touched:
                self.last_update_at = time.time()
        return self.snapshot()

    def set_mode(self, mode: str) -> dict:
        if mode not in VALID_MODES:
            raise ControlInputError(f"mode must be one of {VALID_MODES}")
        if mode == "auto" and AUTO_ARM_CHECK is not None:
            ok, reason = AUTO_ARM_CHECK()
            if not ok:
                raise ControlInputError(reason)
        with self._lock:
            self.mode = mode
            if mode != "manual":
                self.throttle = 0.0
                self.steering = 0.0
            if mode != "auto":
                self.auto_left_us = None
                self.auto_right_us = None
            self.last_update_at = time.time()
            if mode == "off":
                self.reason = "off"
            elif mode == "manual":
                self.reason = "manual armed"
            else:
                self.reason = "auto armed"
        return self.snapshot()

    def apply_auto_pwm(self, left_us: int, right_us: int, reason: str, status: dict | None = None) -> dict:
        with self._lock:
            if self.mode == "auto":
                self.throttle = 0.0
                self.steering = 0.0
                self.auto_left_us = int(left_us)
                self.auto_right_us = int(right_us)
                self.last_update_at = time.time()
                self.reason = reason
            if status is not None:
                self.auto_status = dict(status)
        return self.snapshot()

    def set_auto_status(self, status: dict) -> None:
        with self._lock:
            self.auto_status = dict(status)

    def stop(self) -> dict:
        with self._lock:
            self.mode = "off"
            self.throttle = 0.0
            self.steering = 0.0
            self.auto_left_us = None
            self.auto_right_us = None
            self.last_update_at = time.time()
            self.reason = "operator stop"
        return self.snapshot()

    def command_snapshot(self) -> dict:
        """Return the actuator-bound intent as a plain dict.

        Used by the actuator bridge on its own clock; does not include UI
        chrome. ``stale`` is True when manual/auto mode has not received a
        fresh command within ``stale_after_s``.
        """

        now = time.time()
        with self._lock:
            mode = self.mode
            throttle = self.throttle
            steering = self.steering
            auto_left_us = self.auto_left_us
            auto_right_us = self.auto_right_us
            last_update_at = self.last_update_at
            reason = self.reason

        if mode == "manual":
            stale = last_update_at is None or now - last_update_at > self.stale_after_s
            if stale:
                return {
                    "mode": mode,
                    "source": "manual",
                    "stale": True,
                    "throttle": 0.0,
                    "steering": 0.0,
                    "reason": "manual stale; neutral output",
                }
            return {
                "mode": mode,
                "source": "manual",
                "stale": False,
                "throttle": throttle,
                "steering": steering,
                "reason": reason,
            }
        if mode == "auto":
            stale = last_update_at is None or now - last_update_at > self.stale_after_s
            if stale:
                return {
                    "mode": mode,
                    "source": "auto",
                    "stale": True,
                    "throttle": 0.0,
                    "steering": 0.0,
                    "left_us": None,
                    "right_us": None,
                    "reason": "auto stale; neutral output",
                }
            return {
                "mode": mode,
                "source": "auto",
                "stale": False,
                "throttle": throttle,
                "steering": steering,
                "left_us": auto_left_us,
                "right_us": auto_right_us,
                "reason": reason,
            }
        return {
            "mode": mode,
            "source": "off",
            "stale": False,
            "throttle": 0.0,
            "steering": 0.0,
            "reason": reason or "off",
        }

    def snapshot(self) -> dict:
        now = time.time()
        with self._lock:
            mode = self.mode
            throttle = self.throttle
            steering = self.steering
            auto_left_us = self.auto_left_us
            auto_right_us = self.auto_right_us
            last_update_at = self.last_update_at
            reason = self.reason
            stale_after_s = self.stale_after_s
            manual_slew_per_s = self.manual_slew_per_s
            pivot_turn_start = self.pivot_turn_start
            pivot_reverse_ratio = self.pivot_reverse_ratio
            auto_status = dict(self.auto_status)

        age_s = None if last_update_at is None else round(now - last_update_at, 3)
        stale = mode in ("manual", "auto") and (last_update_at is None or now - last_update_at > stale_after_s)
        enabled = mode in ("manual", "auto")
        effective_throttle = 0.0 if stale or not enabled else throttle
        effective_steering = 0.0 if stale or not enabled else steering
        steering_magnitude = abs(effective_steering)
        outer = min(
            1.0,
            effective_throttle * (1.0 + steering_magnitude),
        )
        inner = effective_throttle * (1.0 - steering_magnitude)
        if (
            pivot_reverse_ratio > 0.0
            and steering_magnitude > pivot_turn_start
        ):
            blend = (
                (steering_magnitude - pivot_turn_start)
                / (1.0 - pivot_turn_start)
            )
            inner_at_start = effective_throttle * (
                1.0 - pivot_turn_start
            )
            inner_at_full = (
                -effective_throttle * pivot_reverse_ratio
            )
            inner = (
                inner_at_start * (1.0 - blend)
                + inner_at_full * blend
            )
        if effective_steering >= 0.0:
            left, right = outer, inner
        else:
            left, right = inner, outer

        if stale:
            reason = f"{mode} command stale; neutral output"

        return {
            "enabled": enabled,
            "mode": mode,
            "stale": stale,
            "age_s": age_s,
            "input": {"throttle": throttle, "steering": steering},
            "output": {
                "throttle": effective_throttle,
                "steering": effective_steering,
                "left_thruster": left,
                "right_thruster": right,
                "left_us": None if stale or mode != "auto" else auto_left_us,
                "right_us": None if stale or mode != "auto" else auto_right_us,
            },
            "limits": {
                "manual_slew_per_s": manual_slew_per_s,
                "stale_after_s": stale_after_s,
                "pivot_turn_start": pivot_turn_start,
                "pivot_reverse_ratio": pivot_reverse_ratio,
            },
            "auto_status": auto_status,
            "reason": reason,
        }


# Backwards-compatibility alias for any external imports / tooling.
ManualControlState = ControlState


class ActuatorBridge:
    """Fixed-rate actuator writer. Reads operator intent from a ControlState,
    applies slew, maps to a left/right PWM pair, and writes to the ESP32 (or
    prints a [DRY] line in dry-run mode). Has its own staleness watchdog.

    Always logs issued commands to a JSONL file: one record per change, plus
    a 1 Hz heartbeat so the disk file always has recent activity.
    """

    def __init__(
        self,
        control_state: ControlState,
        mapping,
        send_hz: float = 20.0,
        slew_per_s: float = 1.5,
        log_path=None,
        serial_writer=None,
        dry_run: bool = True,
    ) -> None:
        from thruster_control import manual_to_pair as _map

        self._map = _map
        self.control_state = control_state
        self.mapping = mapping
        self.send_hz = max(1.0, send_hz)
        self.slew_per_s = max(0.0, slew_per_s)
        self.serial_writer = serial_writer
        self.dry_run = dry_run
        self._log: JsonlLog | None = None
        if log_path is not None:
            try:
                self._log = JsonlLog(log_path)
            except Exception:
                self._log = None
        self._lock = threading.Lock()
        self._last_throttle = 0.0
        self._last_steering = 0.0
        self._last_pair = (mapping.neutral_us, mapping.neutral_us)
        self._last_send_at: float | None = None
        self._last_log_at = 0.0
        self._actuator_label = "dry-run" if dry_run or serial_writer is None else "live"
        self._error: str | None = None
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name="actuator-bridge")

    @property
    def log_path(self) -> str | None:
        return str(self._log.path) if self._log is not None else None

    def start(self) -> None:
        self._thread.start()

    def shutdown(self) -> None:
        self._stop_event.set()
        if self.serial_writer is not None and not self.dry_run:
            try:
                self.serial_writer.stop()
                self.serial_writer.close()
            except Exception:
                pass
        if self._log is not None:
            try:
                self._log.close()
            except Exception:
                pass

    def status_label(self) -> str:
        with self._lock:
            return self._actuator_label

    def effective_output(self) -> dict:
        with self._lock:
            return {
                "throttle": round(self._last_throttle, 4),
                "steering": round(self._last_steering, 4),
                "left_us": int(self._last_pair[0]),
                "right_us": int(self._last_pair[1]),
                "last_send_age_s": None if self._last_send_at is None else round(time.time() - self._last_send_at, 3),
                "send_hz": self.send_hz,
                "slew_per_s": self.slew_per_s,
                "error": self._error,
            }

    def _slew_throttle(self, current: float, target: float, dt: float) -> float:
        """Asymmetric slew: ramp up gently, snap down for safety."""
        if self.slew_per_s <= 0 or target <= current:
            return target
        return min(target, current + self.slew_per_s * dt)

    def _emit(self, intent: dict, throttle: float, steering: float, pair_us: tuple[int, int], sent: bool) -> None:
        now = time.time()
        with self._lock:
            changed = (
                pair_us != self._last_pair
                or abs(throttle - self._last_throttle) > 0.01
                or abs(steering - self._last_steering) > 0.01
            )
            self._last_throttle = throttle
            self._last_steering = steering
            self._last_pair = pair_us
            if sent:
                self._last_send_at = now
            heartbeat_due = now - self._last_log_at >= 1.0

        if self._log is not None and (changed or heartbeat_due):
            self._log.write({
                "ts": round(now, 3),
                "mode": intent.get("mode"),
                "source": intent.get("source"),
                "stale": intent.get("stale", False),
                "intent_throttle": round(float(intent.get("throttle", 0.0)), 4),
                "intent_steering": round(float(intent.get("steering", 0.0)), 4),
                "throttle": round(throttle, 4),
                "steering": round(steering, 4),
                "left_us": int(pair_us[0]),
                "right_us": int(pair_us[1]),
                "actuator": self._actuator_label,
                "reason": intent.get("reason", ""),
            })
            with self._lock:
                self._last_log_at = now

        if self.dry_run and changed:
            print(
                f"[DRY] PWM L{pair_us[0]} R{pair_us[1]} "
                f"throttle={throttle:.2f} steering={steering:+.2f} "
                f"mode={intent.get('mode')} src={intent.get('source')}"
            )

    def _run(self) -> None:
        period = 1.0 / self.send_hz
        last_tick = time.monotonic()

        while not self._stop_event.is_set():
            tick_start = time.monotonic()
            dt = max(0.0, tick_start - last_tick)
            last_tick = tick_start

            intent = self.control_state.command_snapshot()
            target_throttle = float(intent.get("throttle", 0.0))
            target_steering = float(intent.get("steering", 0.0))
            direct_left_us = intent.get("left_us")
            direct_right_us = intent.get("right_us")

            # When the upstream ControlState already reports stale, command_snapshot
            # already returns neutral. We additionally guard against a wedged
            # apply() loop: if the wall-clock age of the last apply is past
            # 5x stale_after_s, force neutral regardless of mode.
            with self.control_state._lock:  # noqa: SLF001 - intentional fast read
                last_update = self.control_state.last_update_at
                stale_after = self.control_state.stale_after_s
            if last_update is not None and intent.get("mode") in ("manual", "auto"):
                wall_age = time.time() - last_update
                if wall_age > stale_after * 5:
                    target_throttle = 0.0
                    target_steering = 0.0

            direct_auto = (
                intent.get("mode") == "auto"
                and not intent.get("stale", False)
                and direct_left_us is not None
                and direct_right_us is not None
            )
            if direct_auto:
                pair_us = (
                    self.mapping.clamp_pwm(float(direct_left_us)),
                    self.mapping.clamp_pwm(float(direct_right_us)),
                )
                new_throttle = 0.0
                new_steering = 0.0
            else:
                with self._lock:
                    last_throttle = self._last_throttle
                new_throttle = self._slew_throttle(last_throttle, target_throttle, dt)
                new_steering = target_steering

                pair = self._map(
                    new_throttle,
                    new_steering,
                    self.mapping,
                    enabled=intent.get("mode") == "manual" and not intent.get("stale", False),
                )
                pair_us = (pair.left_us, pair.right_us)

            sent = False
            if self.dry_run or self.serial_writer is None:
                sent = True
                with self._lock:
                    self._actuator_label = "dry-run"
                    self._error = None
            else:
                try:
                    self.serial_writer.send_pwm_pair(pair_us[0], pair_us[1])
                    sent = True
                    with self._lock:
                        self._actuator_label = "live"
                        self._error = None
                except Exception as exc:
                    with self._lock:
                        self._actuator_label = "error"
                        self._error = str(exc)

            self._emit(intent, new_throttle, new_steering, pair_us, sent)

            elapsed = time.monotonic() - tick_start
            sleep_for = max(0.0, period - elapsed)
            if self._stop_event.wait(sleep_for):
                break


STATE = DashboardState(UnavailableMmwaveState(time.time()), None, None, {})
AUTO_CONTROLLER = None


SNAPSHOT_HZ = 15.0


class DashboardHandler(BaseHTTPRequestHandler):
    # Use HTTP/1.1 so fetch() and EventSource can keep the TCP connection alive
    # across many heartbeats. Without this we'd burn a fresh socket per POST,
    # exhaust ephemeral ports on Windows, and pile up multi-second backpressure.
    protocol_version = "HTTP/1.1"

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
        body = file_path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", f"{content_type}; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            length = 0
        raw = self.rfile.read(length) if length > 0 else b"{}"
        try:
            payload = json.loads(raw.decode("utf-8") or "{}")
        except json.JSONDecodeError:
            self.write_json_error(400, "invalid JSON body")
            return

        try:
            if path == "/api/control/manual":
                self.write_json(STATE.control.apply(payload))
                return
            if path == "/api/control/stop":
                self.write_json(STATE.control.stop())
                return
            if path == "/api/control/mode":
                mode = payload.get("mode") if isinstance(payload, dict) else None
                if not isinstance(mode, str):
                    raise ControlInputError("mode must be a string")
                self.write_json(STATE.control.set_mode(mode))
                return
            if path == "/api/control/waypoints":
                if AUTO_CONTROLLER is None:
                    raise ControlInputError("auto controller unavailable")
                waypoints = payload.get("waypoints") if isinstance(payload, dict) else None
                if not isinstance(waypoints, list):
                    raise ControlInputError("waypoints must be a list")
                self.write_json(AUTO_CONTROLLER.set_waypoints(waypoints))
                return
            if path == "/api/control/relearn-heading":
                if AUTO_CONTROLLER is None:
                    raise ControlInputError("auto controller unavailable")
                self.write_json(AUTO_CONTROLLER.relearn_heading())
                return
            if path == "/api/control/steering-takeover":
                if AUTO_CONTROLLER is None or not hasattr(
                    AUTO_CONTROLLER,
                    "set_steering_takeover",
                ):
                    raise ControlInputError(
                        "steering takeover requires ROS control"
                    )
                if not isinstance(payload, dict):
                    raise ControlInputError("invalid steering takeover")
                self.write_json(
                    AUTO_CONTROLLER.set_steering_takeover(
                        bool(payload.get("active", False)),
                        float(payload.get("steering", 0.0)),
                    )
                )
                return
        except ValueError as exc:
            self.write_json_error(400, str(exc))
            return
        except ControlInputError as exc:
            self.write_json_error(400, str(exc))
            return
        except Exception as exc:  # pragma: no cover - defensive
            self.write_json_error(500, f"control error: {exc}")
            return
        self.send_error(404)

    def write_json(self, payload: dict) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def write_json_error(self, status: int, message: str) -> None:
        body = json.dumps({"error": message}).encode("utf-8")
        self.send_response(status)
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
        period = 1.0 / max(1.0, SNAPSHOT_HZ)
        while True:
            payload = json.dumps(STATE.snapshot(), separators=(",", ":"))
            try:
                self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                break
            time.sleep(period)

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


def _log_path(name: str) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return REPO_ROOT / "logs" / f"dashboard_{name}_{stamp}.jsonl"


def main() -> None:
    global AUTO_ARM_CHECK, AUTO_CONTROLLER, STATE, SNAPSHOT_HZ
    from boat_core.config import choose, load_boat_config, section
    from radar_nav.waypoint import WaypointNavConfig
    from thruster_control import Esp32ThrusterSerial, ThrusterMapping
    from web_dashboard.auto_controller import AutoConfig, AutoController

    started_at = time.time()
    parser = argparse.ArgumentParser(description="MTR realtime sensor dashboard")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--config", default="config/boat.local.json")
    parser.add_argument("--demo", action="store_true", help="Use simulated live sensor data")
    parser.add_argument("--ros", action="store_true", help="Use the default ROS2 mmWave topic (legacy alias)")
    parser.add_argument("--direct-mmwave", action="store_true", help="Force direct mmWave UART mode")
    parser.add_argument("--no-mmwave", action="store_true", help="Disable the mmWave feed")
    parser.add_argument("--mmwave-topic", default="radar/nav_state_json", help="ROS2 std_msgs/String mmWave nav topic")
    parser.add_argument("--cfg-port", help="mmWave CFG / CLI UART port, e.g. COM6 or /dev/ttyUSB0")
    parser.add_argument("--cfg-file", help="Path to TI mmWave .cfg file")
    parser.add_argument("--data-port", help="mmWave DATA UART port, e.g. COM5 or /dev/ttyUSB1")
    parser.add_argument("--baud", type=int, help="mmWave DATA UART baud")
    parser.add_argument("--stale-after-s", type=float, default=2.0, help="Seconds before a ROS feed is marked stale")
    parser.add_argument("--gnss", action="store_true", help="Use direct GNSS serial mode (legacy alias)")
    parser.add_argument("--direct-gnss", action="store_true", help="Use the legacy direct GNSS serial reader")
    parser.add_argument("--no-gnss", action="store_true", help="Disable the GNSS feed")
    parser.add_argument("--gnss-port", help="GNSS serial port")
    parser.add_argument("--gnss-baud", type=int, help="GNSS serial baud")
    parser.add_argument("--imu", action="store_true", help="Use the legacy direct MPU-6050 reader")
    parser.add_argument("--no-imu", action="store_true", help="Disable the ROS IMU feed")
    parser.add_argument("--imu-bus", type=int, help="Legacy MPU-6050 I2C bus")
    parser.add_argument("--imu-address", help="Legacy MPU-6050 I2C address")
    parser.add_argument("--imu-calibration-samples", type=int, default=200, help="Legacy MPU-6050 gyro calibration samples")
    parser.add_argument("--log", action="store_true", help="Write raw live GNSS/IMU/mmWave JSONL logs in addition to the session log")
    parser.add_argument("--no-session-log", action="store_true", help="Disable the unified dashboard session JSONL log")
    parser.add_argument("--session-log-hz", type=float, help="Unified session log rate (Hz, default 10)")
    parser.add_argument("--esp32-port", help="ESP32 thruster serial port (e.g. COM3 or /dev/ttyACM0)")
    parser.add_argument("--esp32-baud", type=int, help="ESP32 serial baud")
    parser.add_argument("--actuator-dry-run", action="store_true", help="Do not write motor commands to the ESP32")
    parser.add_argument("--ros-control", action="store_true", help="Use ROS 2 control (now the default)")
    parser.add_argument("--direct-control", action="store_true", help="Use the legacy direct ESP32 control path")
    parser.add_argument("--snapshot-hz", type=float, help="SSE telemetry rate (Hz, default 15)")
    parser.add_argument("--send-hz", type=float, help="Actuator command send rate (Hz, default 20)")
    parser.add_argument("--manual-slew-per-s", type=float, help="Per-axis slew limit (units/s, default 4.0)")
    args = parser.parse_args()

    config = load_boat_config(args.config)
    radar_config = section(config, "radar")
    gnss_config = section(config, "gnss")
    imu_config = section(config, "imu")
    esp32_config = section(config, "esp32")
    thruster_config = section(config, "thruster")
    runtime_config = section(config, "runtime")
    ros_control_config = section(config, "ros_control")
    auto_config = section(config, "auto")
    cfg_port = choose(args.cfg_port, radar_config, "cfg_port")
    cfg_file = choose(args.cfg_file, radar_config, "cfg_file")
    data_port = choose(args.data_port, radar_config, "data_port")
    data_baud = choose(args.baud, radar_config, "baud", 921600)
    gnss_port = choose(args.gnss_port, gnss_config, "port")
    gnss_baud = choose(args.gnss_baud, gnss_config, "baud", 38400)
    imu_bus = choose(args.imu_bus, imu_config, "bus", 2)
    imu_address = _parse_address(choose(args.imu_address, imu_config, "address", "0x68"))
    esp32_port = choose(args.esp32_port, esp32_config, "port")
    esp32_baud = choose(args.esp32_baud, esp32_config, "baud", 115200)

    snapshot_hz = float(choose(args.snapshot_hz, runtime_config, "snapshot_hz", 30.0))
    send_hz = float(choose(args.send_hz, runtime_config, "send_hz", 20.0))
    session_log_hz = float(choose(args.session_log_hz, runtime_config, "session_log_hz", 10.0))
    manual_slew_per_s = float(choose(args.manual_slew_per_s, runtime_config, "manual_slew_per_s", 4.0))
    waypoint_cfg = WaypointNavConfig(
        reach_radius_m=float(auto_config.get("reach_radius_m", 2.0)),
        approach_slow_radius_m=float(auto_config.get("approach_slow_radius_m", 8.0)),
    )
    auto_runtime_cfg = AutoConfig(
        controller=str(auto_config.get("controller", "smooth_pd_v1")),
        control_hz=float(auto_config.get("control_hz", 10.0)),
        min_speed_for_course_mps=float(auto_config.get("min_speed_for_course_mps", 0.08)),
        gnss_reanchor_speed_mps=float(auto_config.get("gnss_reanchor_speed_mps", 0.3)),
        gnss_heading_blend=float(auto_config.get("gnss_heading_blend", 0.02)),
        gnss_stale_s=float(auto_config.get("gnss_stale_s", 3.0)),
        route_match_tolerance_m=float(auto_config.get("route_match_tolerance_m", 1.0)),
        imu_stale_s=float(auto_config.get("imu_stale_s", 2.0)),
        heading_command_stale_s=float(auto_config.get("heading_command_stale_s", 0.75)),
        heading_learn_min_forward_us=int(auto_config.get("heading_learn_min_forward_us", 1565)),
        heading_learn_max_thruster_delta_us=int(
            auto_config.get("heading_learn_max_thruster_delta_us", 20)
        ),
        heading_learn_duration_s=float(auto_config.get("heading_learn_duration_s", 1.0)),
        heading_learn_min_samples=int(auto_config.get("heading_learn_min_samples", 5)),
        heading_course_stability_deg=float(
            auto_config.get("heading_course_stability_deg", 10.0)
        ),
        heading_course_agreement_deg=float(
            auto_config.get("heading_course_agreement_deg", 45.0)
        ),
        heading_deadband_deg=float(auto_config.get("heading_deadband_deg", 8.0)),
        yaw_rate_deadband_dps=float(auto_config.get("yaw_rate_deadband_dps", 2.0)),
        yaw_lookahead_s=float(auto_config.get("yaw_lookahead_s", 2.0)),
        pulse_turn_enter_deg=float(auto_config.get("pulse_turn_enter_deg", 18.0)),
        pulse_turn_exit_deg=float(auto_config.get("pulse_turn_exit_deg", 6.0)),
        pulse_reverse_deg=float(auto_config.get("pulse_reverse_deg", 38.0)),
        pulse_duration_s=float(auto_config.get("pulse_duration_s", 0.25)),
        pulse_observe_s=float(auto_config.get("pulse_observe_s", 0.35)),
        smooth_kp=float(auto_config.get("smooth_kp", 0.025)),
        smooth_kd=float(auto_config.get("smooth_kd", 0.035)),
        smooth_turn_deadband=float(auto_config.get("smooth_turn_deadband", 0.08)),
        smooth_pwm_slew_us_per_s=float(auto_config.get("smooth_pwm_slew_us_per_s", 300.0)),
        behind_enter_deg=float(auto_config.get("behind_enter_deg", 125.0)),
        behind_exit_deg=float(auto_config.get("behind_exit_deg", 70.0)),
        neutral_us=int(thruster_config.get("neutral_us", 1500)),
        level1_us=int(auto_config.get("level1_us", thruster_config.get("forward_min_us", 1565))),
        level2_us=int(auto_config.get("level2_us", 1575)),
        level3_us=int(auto_config.get("level3_us", thruster_config.get("forward_max_us", 1650))),
        reverse_level1_us=int(
            auto_config.get(
                "reverse_level1_us",
                thruster_config.get("reverse_level1_us", 1460),
            )
        ),
        reverse_level2_us=int(
            auto_config.get(
                "reverse_level2_us",
                thruster_config.get("reverse_level2_us", 1445),
            )
        ),
        reverse_level3_us=int(
            auto_config.get(
                "reverse_level3_us",
                thruster_config.get("reverse_level3_us", 1425),
            )
        ),
        waypoint=waypoint_cfg,
    )

    SNAPSHOT_HZ = max(1.0, snapshot_hz)

    if args.ros and args.direct_mmwave:
        parser.error("--ros and --direct-mmwave are mutually exclusive")
    if args.ros_control and args.direct_control:
        parser.error("--ros-control and --direct-control are mutually exclusive")

    use_direct_mmwave = args.direct_mmwave and not args.no_mmwave
    use_ros_mmwave = (
        not args.demo
        and not args.no_mmwave
        and not use_direct_mmwave
    )
    use_direct_gnss = (
        not args.demo
        and not args.no_gnss
        and (args.direct_gnss or args.gnss or bool(args.gnss_port))
    )
    use_ros_gnss = (
        not args.demo
        and not args.no_gnss
        and not use_direct_gnss
    )
    use_direct_imu = args.imu and not args.no_imu
    use_ros_imu = not args.demo and not args.no_imu and not use_direct_imu
    use_ros_control = (
        not args.demo
        and not args.direct_control
    )

    if args.demo:
        mmwave_state = DemoSensorState(started_at)
    elif use_ros_mmwave:
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
    if use_direct_gnss:
        if not gnss_port:
            parser.error("--gnss-port is required unless set in config")
        gnss_log = JsonlLog(_log_path("gnss")) if args.log else None
        if gnss_log is not None:
            log_paths["gnss"] = str(gnss_log.path)
        gnss_reader = LiveGnssReader(gnss_port, gnss_baud, gnss_log)
    elif use_ros_gnss:
        gnss_log = JsonlLog(_log_path("gnss")) if args.log else None
        if gnss_log is not None:
            log_paths["gnss"] = str(gnss_log.path)
        gnss_reader = RosGnssReader(
            stale_after_s=float(auto_config.get("gnss_stale_s", 3.0)),
            log=gnss_log,
        )
    if use_direct_imu or use_ros_imu:
        imu_log = JsonlLog(_log_path("imu")) if args.log else None
        if imu_log is not None:
            log_paths["imu"] = str(imu_log.path)
        if use_direct_imu:
            imu_reader = LiveImuReader(
                imu_bus,
                imu_address,
                imu_log,
                calibration_samples=args.imu_calibration_samples,
            )
        else:
            imu_reader = RosImuReader(log=imu_log)

    STATE = DashboardState(
        mmwave_state,
        gnss_reader,
        imu_reader,
        log_paths,
        manual_slew_per_s=manual_slew_per_s,
        pivot_turn_start=float(
            thruster_config.get("pivot_turn_start", 0.75)
        ),
        pivot_reverse_ratio=float(
            thruster_config.get("pivot_reverse_ratio", 1.0)
        ),
    )
    mapping = ThrusterMapping(
        neutral_us=int(thruster_config.get("neutral_us", 1500)),
        forward_min_us=int(thruster_config.get("forward_min_us", 1520)),
        forward_max_us=int(thruster_config.get("forward_max_us", 1600)),
        reverse_level1_us=int(
            thruster_config.get("reverse_level1_us", 1460)
        ),
        reverse_level2_us=int(
            thruster_config.get("reverse_level2_us", 1445)
        ),
        reverse_level3_us=int(
            thruster_config.get("reverse_level3_us", 1425)
        ),
        hard_min_us=int(thruster_config.get("hard_min_us", 1350)),
        hard_max_us=int(thruster_config.get("hard_max_us", 2000)),
        steering_slowdown=float(thruster_config.get("steering_slowdown", 0.35)),
        pivot_turn_start=float(
            thruster_config.get("pivot_turn_start", 0.75)
        ),
        pivot_reverse_ratio=float(
            thruster_config.get("pivot_reverse_ratio", 1.0)
        ),
    )

    local_auto_controller = None
    if use_ros_control:
        from web_dashboard.ros_control import RosCommandBridge

        actuator_bridge = RosCommandBridge(
            STATE.control,
            mapping=mapping,
            send_hz=send_hz,
            max_linear_mps=float(
                ros_control_config.get("max_linear_mps", 1.0)
            ),
            max_angular_rps=float(
                ros_control_config.get("max_angular_rps", 1.0)
            ),
            operator_topic=str(
                ros_control_config.get(
                    "operator_topic",
                    "cmd_vel/operator",
                )
            ),
            steering_takeover_topic=str(
                ros_control_config.get(
                    "steering_takeover_topic",
                    "control/steering_takeover",
                )
            ),
            mode_topic=str(
                ros_control_config.get(
                    "mode_request_topic",
                    "control/mode_request",
                )
            ),
            route_topic=str(
                ros_control_config.get(
                    "route_topic",
                    "autonomy/route",
                )
            ),
            heading_reset_topic=str(
                ros_control_config.get(
                    "heading_reset_topic",
                    "autonomy/relearn_heading",
                )
            ),
            autonomy_status_topic=str(
                ros_control_config.get(
                    "autonomy_status_topic",
                    "autonomy/status",
                )
            ),
            thruster_command_topic=str(
                ros_control_config.get(
                    "thruster_command_topic",
                    "thrusters/command",
                )
            ),
        )
        AUTO_CONTROLLER = actuator_bridge
        actuator_status_msg = "ROS 2 control topics"
    else:
        local_auto_controller = AutoController(
            STATE.control,
            gnss_reader,
            imu_reader,
            auto_runtime_cfg,
        )
        local_auto_controller.start()
        AUTO_CONTROLLER = local_auto_controller
        esp32_serial = None
        actuator_dry_run = True
        actuator_status_msg = "dry-run (no ESP32 port configured)"
        want_live = not args.actuator_dry_run and bool(esp32_port)
        if want_live:
            try:
                esp32_serial = Esp32ThrusterSerial(
                    esp32_port,
                    baud=esp32_baud,
                )
                actuator_dry_run = False
                actuator_status_msg = (
                    f"live ({esp32_port} @ {esp32_baud})"
                )
            except Exception as exc:
                actuator_status_msg = (
                    f"dry-run (ESP32 open failed: {exc})"
                )
                print(f"[ACTUATOR] {actuator_status_msg}")

        actuator_bridge = ActuatorBridge(
            STATE.control,
            mapping=mapping,
            send_hz=send_hz,
            slew_per_s=manual_slew_per_s,
            log_path=_log_path("actuator"),
            serial_writer=esp32_serial,
            dry_run=actuator_dry_run,
        )
    AUTO_ARM_CHECK = AUTO_CONTROLLER.can_arm
    STATE.actuator = actuator_bridge
    actuator_bridge.start()
    if actuator_bridge.log_path is not None:
        log_paths["actuator"] = actuator_bridge.log_path
    session_logger = None
    if not args.no_session_log:
        session_logger = SessionLogger(STATE, _log_path("session"), hz=session_log_hz)
        log_paths["session"] = str(session_logger.path)
        session_logger.start()

    class _LowLatencyServer(ThreadingHTTPServer):
        # Disable Nagle on each accepted socket so 50-byte heartbeat POSTs
        # round-trip in microseconds instead of being held up to 200ms.
        daemon_threads = True

        def process_request(self, request, client_address):  # type: ignore[override]
            try:
                request.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except OSError:
                pass
            super().process_request(request, client_address)

    server = _LowLatencyServer((args.host, args.port), DashboardHandler)
    print(f"Dashboard build: {DASHBOARD_BUILD}")
    print(f"MTR dashboard: http://localhost:{args.port}")
    print(f"LAN dashboard: http://<orange-pi-or-laptop-ip>:{args.port}")
    if args.demo:
        print("Mode: demo")
    elif use_ros_mmwave:
        print(f"Mode: ROS2 mmWave topic {args.mmwave_topic}")
    elif use_direct_mmwave:
        print(f"Mode: direct mmWave {data_port} @ {data_baud}")
        print(f"mmWave DATA candidates: {mmwave_state.data_candidates}")
        if cfg_port and cfg_file:
            print(f"mmWave config: {cfg_file} via {cfg_port}")
            print(f"mmWave CFG candidates: {mmwave_state.cfg_candidates}")
    else:
        print("Mode: waiting for live feeds")
    if use_direct_gnss and gnss_reader is not None:
        print(f"GNSS: {gnss_port} @ {gnss_baud}")
        print(f"GNSS candidates: {gnss_reader.candidates}")
    elif use_ros_gnss:
        print("GNSS: ROS2 topics gnss/fix and gnss/velocity")
    if imu_reader is not None:
        if use_direct_imu:
            print(
                f"IMU: legacy MPU-6050 on I2C bus {imu_bus}, "
                f"address 0x{imu_address:02x}"
            )
        else:
            print("IMU: ROS2 BNO055 topics under /imu")
    print(
        f"Telemetry: snapshot {SNAPSHOT_HZ:.0f} Hz / send {send_hz:.0f} Hz / "
        f"slew {manual_slew_per_s:.2f}/s"
    )
    print(f"Actuator: {actuator_status_msg}")
    if log_paths:
        print(f"Logging: {log_paths}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        if local_auto_controller is not None:
            local_auto_controller.shutdown()
        if session_logger is not None:
            session_logger.shutdown()
        actuator_bridge.shutdown()
        for reader in (mmwave_state, gnss_reader, imu_reader):
            shutdown = getattr(reader, "shutdown", None)
            if shutdown is not None:
                shutdown()


if __name__ == "__main__":
    main()
