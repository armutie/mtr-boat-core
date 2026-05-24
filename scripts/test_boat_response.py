from __future__ import annotations

import argparse
import json
import statistics
import sys
import threading
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from boat_core.config import choose, load_boat_config, section
from gnss import NmeaReader
from gnss.geo import distance_m, normalize_angle_deg
from imu import GyroBias, Mpu6050, RelativeYawTracker
from thruster_control import Esp32ThrusterSerial


@dataclass(frozen=True)
class Phase:
    name: str
    duration_s: float
    left_us: int
    right_us: int
    note: str


def parse_address(value: int | str) -> int:
    return value if isinstance(value, int) else int(value, 0)


def default_log_path() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("logs") / f"boat_response_{stamp}.jsonl"


def parse_levels(raw: str) -> list[int]:
    levels = []
    for item in raw.split(","):
        item = item.strip()
        if item:
            levels.append(int(item))
    if not levels:
        raise argparse.ArgumentTypeError("at least one PWM level is required")
    return levels


def apply_config(args) -> None:
    config = load_boat_config(args.config)
    esp32 = section(config, "esp32")
    gnss = section(config, "gnss")
    imu = section(config, "imu")
    thruster = section(config, "thruster")

    args.esp32_port = choose(args.esp32_port, esp32, "port")
    args.esp32_baud = choose(args.esp32_baud, esp32, "baud", 115200)
    args.gnss_port = choose(args.gnss_port, gnss, "port")
    args.gnss_baud = choose(args.gnss_baud, gnss, "baud", 38400)
    args.imu_bus = choose(args.imu_bus, imu, "bus", 2)
    args.imu_address = parse_address(choose(args.imu_address, imu, "address", "0x68"))
    args.neutral_us = choose(args.neutral_us, thruster, "neutral_us", 1500)
    forward_min = choose(None, thruster, "forward_min_us", 1565)
    forward_mid = min(choose(None, thruster, "forward_max_us", 1650), max(int(forward_min) + 35, int(forward_min)))
    if args.levels is None:
        args.levels = [int(forward_min), int(forward_mid)]


class GnssThread:
    def __init__(self, port: str, baud: int):
        self.port = port
        self.baud = baud
        self.latest: dict | None = None
        self.error: str | None = None
        self.count = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)

    def snapshot(self) -> dict:
        with self._lock:
            latest = dict(self.latest) if self.latest is not None else None
            error = self.error
            count = self.count
        now = time.time()
        if latest is None:
            return {"health": "waiting", "error": error, "count": count, "age_s": None}
        latest["health"] = "live"
        latest["error"] = error
        latest["count"] = count
        latest["age_s"] = round(now - float(latest["timestamp"]), 3)
        return latest

    def _run(self) -> None:
        try:
            with NmeaReader(self.port, baud=self.baud) as reader:
                while not self._stop.is_set():
                    fix = reader.read_fix()
                    if fix is None:
                        continue
                    with self._lock:
                        self.latest = fix.to_record()
                        self.count += 1
                        self.error = None
        except Exception as exc:
            with self._lock:
                self.error = str(exc)


def build_phases(args) -> list[Phase]:
    phases: list[Phase] = [
        Phase("still_baseline", args.baseline_s, args.neutral_us, args.neutral_us, "neutral; measure drift/jitter"),
    ]
    pulse_durations = [args.flash_s, args.medium_s]
    long_durations = [args.long_s]
    if args.extra_long_s > 0:
        long_durations.append(args.extra_long_s)

    for level in args.levels:
        for duration in pulse_durations + long_durations:
            label = f"{duration:g}s"
            phases.extend(
                [
                    Phase(f"both_{level}_{label}", duration, level, level, "both motors forward"),
                    Phase(f"coast_after_both_{level}_{label}", args.coast_s, args.neutral_us, args.neutral_us, "neutral coast"),
                    Phase(f"left_only_{level}_{label}", duration, level, args.neutral_us, "left motor forward, right neutral"),
                    Phase(f"coast_after_left_{level}_{label}", args.coast_s, args.neutral_us, args.neutral_us, "neutral coast"),
                    Phase(f"right_only_{level}_{label}", duration, args.neutral_us, level, "right motor forward, left neutral"),
                    Phase(f"coast_after_right_{level}_{label}", args.coast_s, args.neutral_us, args.neutral_us, "neutral coast"),
                ]
            )
    phases.append(Phase("still_end", args.baseline_s, args.neutral_us, args.neutral_us, "neutral; end drift/jitter"))
    return phases


def send_pair(writer: Esp32ThrusterSerial | None, left_us: int, right_us: int, dry_run: bool) -> str | None:
    if dry_run:
        return None
    assert writer is not None
    try:
        writer.send_pwm_pair(left_us, right_us)
        return None
    except Exception as exc:
        return str(exc)


def run_test(args) -> None:
    apply_config(args)
    if not args.gnss_port:
        raise SystemExit("--gnss-port is required unless set in config")
    if not args.esp32_port and not args.dry_run:
        raise SystemExit("--esp32-port is required unless set in config; use --dry-run to log without motors")

    log_path = Path(args.log_path) if args.log_path else default_log_path()
    log_path.parent.mkdir(parents=True, exist_ok=True)
    phases = build_phases(args)
    sample_period = 1.0 / max(args.rate_hz, 1.0)
    send_period = 1.0 / max(args.send_hz, 1.0)

    print(f"[BOAT RESPONSE] Log: {log_path}")
    print(f"[BOAT RESPONSE] GNSS {args.gnss_port} @ {args.gnss_baud}")
    print(f"[BOAT RESPONSE] IMU I2C bus {args.imu_bus}, address 0x{args.imu_address:02x}")
    print(f"[BOAT RESPONSE] ESP32 {'dry-run' if args.dry_run else f'{args.esp32_port} @ {args.esp32_baud}'}")
    print(f"[BOAT RESPONSE] Levels: {', '.join(str(x) for x in args.levels)} us")
    print(f"[BOAT RESPONSE] Phases: {len(phases)}")
    print(f"[BOAT RESPONSE] Starting in {args.start_delay_s:.1f}s...")
    time.sleep(max(0.0, args.start_delay_s))

    writer = None if args.dry_run else Esp32ThrusterSerial(args.esp32_port, args.esp32_baud)
    gnss = GnssThread(args.gnss_port, args.gnss_baud)
    gnss.start()
    command_error: str | None = None
    try:
        with Mpu6050(bus=args.imu_bus, address=args.imu_address) as imu, log_path.open("a", encoding="utf-8") as log:
            print("[BOAT RESPONSE] Keep boat still for gyro calibration.")
            bias = imu.calibrate_gyro(samples=args.calibration_samples) if args.calibration_samples > 0 else GyroBias()
            yaw = RelativeYawTracker()
            print(f"[BOAT RESPONSE] Bias gx={bias.x_dps:+.3f}, gy={bias.y_dps:+.3f}, gz={bias.z_dps:+.3f} dps")

            started = time.time()
            for phase_index, phase in enumerate(phases, start=1):
                phase_started = time.time()
                next_send = 0.0
                print(
                    f"\n[BOAT RESPONSE] {phase_index}/{len(phases)} {phase.name} "
                    f"({phase.duration_s:.2f}s) L{phase.left_us} R{phase.right_us}: {phase.note}"
                )
                while time.time() - phase_started < phase.duration_s:
                    now = time.time()
                    if now >= next_send:
                        command_error = send_pair(writer, phase.left_us, phase.right_us, args.dry_run)
                        next_send = now + send_period

                    sample = imu.read_sample(bias=bias)
                    yaw_deg, dt_s = yaw.update(sample)
                    record = {
                        "timestamp": now,
                        "elapsed_s": round(now - started, 3),
                        "phase": phase.name,
                        "phase_elapsed_s": round(now - phase_started, 3),
                        "command": {
                            "left_us": phase.left_us,
                            "right_us": phase.right_us,
                            "neutral_us": args.neutral_us,
                            "dry_run": args.dry_run,
                            "error": command_error,
                        },
                        "imu": {
                            **sample.to_record(),
                            "yaw_relative_deg": yaw_deg,
                            "dt_s": dt_s,
                            "bias": asdict(bias),
                        },
                        "gnss": gnss.snapshot(),
                    }
                    log.write(json.dumps(record, separators=(",", ":")) + "\n")
                    log.flush()

                    if int((now - phase_started) * 2) != int((now - phase_started - sample_period) * 2):
                        g = record["gnss"]
                        print(
                            f"[BOAT RESPONSE] t={record['elapsed_s']:6.1f}s "
                            f"yaw={yaw_deg:+7.2f}deg gz={sample.gyro_z_dps:+6.2f}dps "
                            f"speed={g.get('speed_mps') if g.get('speed_mps') is not None else '--'} "
                            f"course={g.get('heading_deg') if g.get('heading_deg') is not None else '--'}"
                        )
                    time.sleep(sample_period)
    finally:
        try:
            if writer is not None:
                writer.stop()
                writer.close()
        finally:
            gnss.stop()

    print(f"\n[BOAT RESPONSE] Done. Analyze with:")
    print(f"python3 scripts/test_boat_response.py analyze {log_path}")


def positioned(rows: list[dict]) -> list[dict]:
    return [r for r in rows if r.get("gnss", {}).get("lat") is not None and r.get("gnss", {}).get("lon") is not None]


def path_distance(rows: list[dict]) -> float:
    pts = positioned(rows)
    total = 0.0
    prev = None
    for row in pts:
        g = row["gnss"]
        if prev is not None:
            p = prev["gnss"]
            total += distance_m(p["lat"], p["lon"], g["lat"], g["lon"])
        prev = row
    return total


def analyze_phase(rows: list[dict]) -> dict:
    first = rows[0]
    last = rows[-1]
    yaw_delta = normalize_angle_deg(last["imu"]["yaw_relative_deg"] - first["imu"]["yaw_relative_deg"])
    gyro_z = [float(r["imu"]["gyro_z_dps"]) for r in rows if r.get("imu", {}).get("gyro_z_dps") is not None]
    speeds = [float(r["gnss"]["speed_mps"]) for r in rows if r.get("gnss", {}).get("speed_mps") is not None]
    headings = [float(r["gnss"]["heading_deg"]) for r in rows if r.get("gnss", {}).get("heading_deg") is not None]
    duration = max(0.001, last["elapsed_s"] - first["elapsed_s"])
    distance = path_distance(rows)
    yaw_rate_avg = yaw_delta / duration
    speed_avg = statistics.mean(speeds) if speeds else None
    turn_radius = None
    if speed_avg is not None and abs(yaw_rate_avg) > 0.5:
        turn_radius = speed_avg / abs(yaw_rate_avg * 3.141592653589793 / 180.0)
    return {
        "phase": first["phase"],
        "duration_s": duration,
        "left_us": first["command"]["left_us"],
        "right_us": first["command"]["right_us"],
        "yaw_delta_deg": yaw_delta,
        "yaw_rate_avg_dps": yaw_rate_avg,
        "gyro_z_peak_abs_dps": max((abs(x) for x in gyro_z), default=None),
        "distance_m": distance,
        "speed_avg_mps": speed_avg,
        "speed_max_mps": max(speeds) if speeds else None,
        "gnss_heading_delta_deg": normalize_angle_deg(headings[-1] - headings[0]) if len(headings) >= 2 else None,
        "estimated_turn_radius_m": turn_radius,
    }


def fmt(value, digits: int = 2) -> str:
    return "--" if value is None else f"{value:.{digits}f}"


def run_analyze(args) -> None:
    rows = [json.loads(line) for line in Path(args.log_path).read_text(encoding="utf-8-sig").splitlines() if line.strip()]
    if not rows:
        raise SystemExit("log is empty")
    phases: dict[str, list[dict]] = {}
    for row in rows:
        phases.setdefault(row["phase"], []).append(row)

    print(f"[ANALYZE] {args.log_path}")
    print("phase,left_us,right_us,duration_s,yaw_delta_deg,yaw_rate_avg_dps,gyro_z_peak_abs_dps,distance_m,speed_avg_mps,speed_max_mps,gnss_heading_delta_deg,estimated_turn_radius_m")
    for phase_rows in phases.values():
        s = analyze_phase(phase_rows)
        print(
            f"{s['phase']},{s['left_us']},{s['right_us']},{s['duration_s']:.2f},"
            f"{s['yaw_delta_deg']:.2f},{s['yaw_rate_avg_dps']:.2f},{fmt(s['gyro_z_peak_abs_dps'])},"
            f"{s['distance_m']:.2f},{fmt(s['speed_avg_mps'])},{fmt(s['speed_max_mps'])},"
            f"{fmt(s['gnss_heading_delta_deg'])},{fmt(s['estimated_turn_radius_m'])}"
        )


def main() -> None:
    if len(sys.argv) > 1 and sys.argv[1] == "analyze":
        parser = argparse.ArgumentParser(
            prog=f"{Path(sys.argv[0]).name} analyze",
            description="Analyze a boat response JSONL log.",
        )
        parser.add_argument("log_path")
        run_analyze(parser.parse_args(sys.argv[2:]))
        return

    parser = argparse.ArgumentParser(
        description="Run a GNSS+IMU+thruster boat response characterization sequence."
    )
    parser.add_argument("--config", default="config/boat.local.json")
    parser.add_argument("--esp32-port")
    parser.add_argument("--esp32-baud", type=int)
    parser.add_argument("--gnss-port")
    parser.add_argument("--gnss-baud", type=int)
    parser.add_argument("--imu-bus", type=int)
    parser.add_argument("--imu-address")
    parser.add_argument("--neutral-us", type=int)
    parser.add_argument("--levels", type=parse_levels, help="Comma-separated PWM levels, default from config")
    parser.add_argument("--flash-s", type=float, default=0.25)
    parser.add_argument("--medium-s", type=float, default=1.0)
    parser.add_argument("--long-s", type=float, default=2.0)
    parser.add_argument("--extra-long-s", type=float, default=4.0)
    parser.add_argument("--coast-s", type=float, default=4.0)
    parser.add_argument("--baseline-s", type=float, default=5.0)
    parser.add_argument("--start-delay-s", type=float, default=5.0)
    parser.add_argument("--calibration-samples", type=int, default=400)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--send-hz", type=float, default=20.0)
    parser.add_argument("--log-path")
    parser.add_argument("--dry-run", action="store_true")
    run_test(parser.parse_args())


if __name__ == "__main__":
    main()
