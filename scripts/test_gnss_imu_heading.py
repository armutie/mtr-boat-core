from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import threading
import time
from collections import defaultdict
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from boat_core.config import choose, load_boat_config, section
from gnss import NmeaReader
from gnss.geo import bearing_deg, distance_m, normalize_angle_deg
from imu import GyroBias, Mpu6050, RelativeYawTracker


DEFAULT_PHASES = (
    ("still_start", 45.0, "Keep the package still."),
    ("straight_walk_1", 45.0, "Walk straight at a normal pace."),
    ("rotate_in_place", 25.0, "Stop and rotate the package about 90-180 degrees in place."),
    ("straight_walk_2", 45.0, "Walk straight again in the new facing direction."),
    ("side_step", 30.0, "Keep the package facing the same way and side-step if you can."),
    ("slow_shuffle", 30.0, "Move slowly; this should stress GNSS course confidence."),
    ("still_end", 45.0, "Set it still again."),
)


def parse_address(value: int | str) -> int:
    return value if isinstance(value, int) else int(value, 0)


def default_log_path() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("logs") / f"gnss_imu_heading_{stamp}.jsonl"


def load_config(args) -> None:
    config = load_boat_config(args.config)
    gnss = section(config, "gnss")
    imu = section(config, "imu")
    args.gnss_port = choose(args.gnss_port, gnss, "port")
    args.gnss_baud = choose(args.gnss_baud, gnss, "baud", 38400)
    args.imu_bus = choose(args.imu_bus, imu, "bus", 2)
    args.imu_address = parse_address(choose(args.imu_address, imu, "address", "0x68"))


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


def run_record(args) -> None:
    load_config(args)
    if not args.gnss_port:
        raise SystemExit("--gnss-port is required unless set in config")

    log_path = Path(args.log_path) if args.log_path else default_log_path()
    log_path.parent.mkdir(parents=True, exist_ok=True)
    delay_s = 1.0 / max(args.rate_hz, 1.0)
    phases = DEFAULT_PHASES if not args.quick else tuple((name, min(duration, 15.0), prompt) for name, duration, prompt in DEFAULT_PHASES)

    print(f"[HEADING TEST] GNSS {args.gnss_port} @ {args.gnss_baud}")
    print(f"[HEADING TEST] IMU I2C bus {args.imu_bus}, address 0x{args.imu_address:02x}")
    print(f"[HEADING TEST] Logging to {log_path}")
    print("[HEADING TEST] Keep the package still during gyro calibration.")

    gnss = GnssThread(args.gnss_port, args.gnss_baud)
    gnss.start()
    try:
        with Mpu6050(bus=args.imu_bus, address=args.imu_address) as imu, log_path.open("a", encoding="utf-8") as log:
            bias = imu.calibrate_gyro(samples=args.calibration_samples) if args.calibration_samples > 0 else GyroBias()
            yaw = RelativeYawTracker()
            print(f"[HEADING TEST] Bias gx={bias.x_dps:+.3f}, gy={bias.y_dps:+.3f}, gz={bias.z_dps:+.3f} dps")

            started = time.time()
            for phase, duration_s, prompt in phases:
                phase_started = time.time()
                print(f"\n[HEADING TEST] Phase: {phase} ({duration_s:.0f}s)")
                print(f"[HEADING TEST] {prompt}")
                while time.time() - phase_started < duration_s:
                    now = time.time()
                    sample = imu.read_sample(bias=bias)
                    yaw_deg, dt_s = yaw.update(sample)
                    record = {
                        "timestamp": now,
                        "elapsed_s": round(now - started, 3),
                        "phase": phase,
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

                    if int((now - phase_started) * 2) != int((now - phase_started - delay_s) * 2):
                        g = record["gnss"]
                        lat = g.get("lat")
                        lon = g.get("lon")
                        speed = g.get("speed_mps")
                        heading = g.get("heading_deg")
                        print(
                            f"[HEADING TEST] t={record['elapsed_s']:6.1f}s phase={phase:16s} "
                            f"yaw={yaw_deg:+7.2f} deg "
                            f"gnss=({lat if lat is not None else '--'}, {lon if lon is not None else '--'}) "
                            f"speed={speed if speed is not None else '--'} heading={heading if heading is not None else '--'}"
                        )
                    time.sleep(delay_s)
    finally:
        gnss.stop()

    print(f"\n[HEADING TEST] Done. Analyze with:")
    print(f"python3 scripts/test_gnss_imu_heading.py analyze {log_path}")


def unique_gnss_samples(rows: list[dict]) -> list[dict]:
    samples = []
    seen = set()
    for row in rows:
        g = row.get("gnss") or {}
        ts = g.get("timestamp")
        if ts is None or ts in seen:
            continue
        seen.add(ts)
        samples.append(g)
    return samples


def circular_mean_deg(values: list[float]) -> float | None:
    if not values:
        return None
    x = sum(math.cos(math.radians(v)) for v in values)
    y = sum(math.sin(math.radians(v)) for v in values)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def percentile(values: list[float], pct: float) -> float | None:
    if not values:
        return None
    values = sorted(values)
    idx = min(len(values) - 1, max(0, int(round((pct / 100.0) * (len(values) - 1)))))
    return values[idx]


def heading_spread_deg(values: list[float]) -> tuple[float | None, float | None]:
    mean = circular_mean_deg(values)
    if mean is None:
        return None, None
    errors = [abs(normalize_angle_deg(value - mean)) for value in values]
    return percentile(errors, 90), max(errors) if errors else None


def positioned(samples: list[dict]) -> list[dict]:
    return [s for s in samples if s.get("lat") is not None and s.get("lon") is not None]


def path_distance(samples: list[dict]) -> float:
    total = 0.0
    prev = None
    for item in positioned(samples):
        if prev is not None:
            total += distance_m(prev["lat"], prev["lon"], item["lat"], item["lon"])
        prev = item
    return total


def mean_position_radius(samples: list[dict]) -> float | None:
    pts = positioned(samples)
    if not pts:
        return None
    lat = statistics.mean(p["lat"] for p in pts)
    lon = statistics.mean(p["lon"] for p in pts)
    return max(distance_m(lat, lon, p["lat"], p["lon"]) for p in pts)


def analyze_phase(name: str, rows: list[dict], speed_threshold: float, stable_heading_deg: float) -> dict:
    samples = unique_gnss_samples(rows)
    pos = positioned(samples)
    headings = [float(s["heading_deg"]) for s in samples if s.get("heading_deg") is not None]
    speeds = [float(s["speed_mps"]) for s in samples if s.get("speed_mps") is not None]
    p90_spread, max_spread = heading_spread_deg(headings)
    yaw0 = rows[0]["imu"]["yaw_relative_deg"]
    yaw1 = rows[-1]["imu"]["yaw_relative_deg"]
    yaw_delta = normalize_angle_deg(yaw1 - yaw0)
    first_heading = headings[0] if headings else None
    last_heading = headings[-1] if headings else None
    gnss_heading_delta = None if first_heading is None or last_heading is None else normalize_angle_deg(last_heading - first_heading)
    disagreement = None if gnss_heading_delta is None else normalize_angle_deg(gnss_heading_delta - yaw_delta)
    confident = [
        s for s in samples
        if s.get("fix") in ("fix", "estimated")
        and s.get("heading_deg") is not None
        and s.get("speed_mps") is not None
        and float(s["speed_mps"]) >= speed_threshold
    ]

    verdict = []
    if speeds:
        if max(speeds) < speed_threshold:
            verdict.append("GNSS course should not be trusted here: speed stayed below threshold.")
        elif p90_spread is not None and p90_spread <= stable_heading_deg:
            verdict.append("GNSS course looks stable enough for anchoring while moving.")
        else:
            verdict.append("GNSS course is moving but noisy; anchor only after a longer stable window.")
    else:
        verdict.append("No GNSS speed samples.")
    if disagreement is not None and abs(disagreement) > 35.0:
        verdict.append("GNSS course change and IMU yaw disagree strongly; reject re-anchor in this phase.")
    if name.startswith("still") and mean_position_radius(samples) is not None:
        verdict.append(f"Stationary GNSS jitter radius was about {mean_position_radius(samples):.1f} m.")

    return {
        "phase": name,
        "duration_s": rows[-1]["elapsed_s"] - rows[0]["elapsed_s"],
        "rows": len(rows),
        "gnss_samples": len(samples),
        "positioned_samples": len(pos),
        "path_distance_m": path_distance(samples),
        "stationary_jitter_radius_m": mean_position_radius(samples) if name.startswith("still") else None,
        "speed_avg_mps": statistics.mean(speeds) if speeds else None,
        "speed_max_mps": max(speeds) if speeds else None,
        "heading_p90_spread_deg": p90_spread,
        "heading_max_spread_deg": max_spread,
        "imu_yaw_delta_deg": yaw_delta,
        "gnss_heading_delta_deg": gnss_heading_delta,
        "gnss_minus_imu_delta_deg": disagreement,
        "confident_gnss_samples": len(confident),
        "verdict": verdict,
    }


def fmt(value, digits: int = 2) -> str:
    return "--" if value is None else f"{value:.{digits}f}"


def run_analyze(args) -> None:
    rows = [json.loads(line) for line in Path(args.log_path).read_text(encoding="utf-8-sig").splitlines() if line.strip()]
    if not rows:
        raise SystemExit("log is empty")
    phases: dict[str, list[dict]] = defaultdict(list)
    for row in rows:
        phases[row.get("phase", "unknown")].append(row)

    print(f"[ANALYZE] {args.log_path}")
    print(f"[ANALYZE] rows={len(rows)} duration={rows[-1]['elapsed_s']:.1f}s")
    print(f"[ANALYZE] thresholds: speed>={args.speed_threshold_mps:.2f} m/s, heading p90 spread<={args.stable_heading_deg:.1f} deg")

    for name, phase_rows in phases.items():
        summary = analyze_phase(name, phase_rows, args.speed_threshold_mps, args.stable_heading_deg)
        print(f"\n## {name}")
        print(
            f"duration={summary['duration_s']:.1f}s gnss_samples={summary['gnss_samples']} "
            f"path={summary['path_distance_m']:.1f}m speed_avg={fmt(summary['speed_avg_mps'])} "
            f"speed_max={fmt(summary['speed_max_mps'])}"
        )
        print(
            f"heading_spread_p90={fmt(summary['heading_p90_spread_deg'],1)}deg "
            f"imu_yaw_delta={fmt(summary['imu_yaw_delta_deg'],1)}deg "
            f"gnss_heading_delta={fmt(summary['gnss_heading_delta_deg'],1)}deg "
            f"disagreement={fmt(summary['gnss_minus_imu_delta_deg'],1)}deg"
        )
        if summary["stationary_jitter_radius_m"] is not None:
            print(f"stationary_jitter_radius={summary['stationary_jitter_radius_m']:.1f}m")
        print(f"confident_gnss_samples={summary['confident_gnss_samples']}")
        for item in summary["verdict"]:
            print(f"- {item}")

    print("\n[ANALYZE] Practical interpretation:")
    print("- Use phases with stable moving GNSS and low GNSS-vs-IMU disagreement as anchor candidates.")
    print("- Treat still/slow phases as IMU-only carry periods, not GNSS re-anchor periods.")
    print("- If stationary jitter is several meters, waypoint reach radius must be larger than that jitter.")


def main() -> None:
    if len(sys.argv) > 1 and sys.argv[1] == "analyze":
        parser = argparse.ArgumentParser(
            prog=f"{Path(sys.argv[0]).name} analyze",
            description="Analyze a recorded GNSS+IMU heading log.",
        )
        parser.add_argument("log_path")
        parser.add_argument("--speed-threshold-mps", type=float, default=0.4)
        parser.add_argument("--stable-heading-deg", type=float, default=20.0)
        run_analyze(parser.parse_args(sys.argv[2:]))
        return

    parser = argparse.ArgumentParser(
        description="Record synchronized GNSS+IMU data. Use 'analyze <log>' afterward to analyze a log."
    )
    parser.add_argument("--config", default="config/boat.local.json")
    parser.add_argument("--gnss-port")
    parser.add_argument("--gnss-baud", type=int)
    parser.add_argument("--imu-bus", type=int)
    parser.add_argument("--imu-address")
    parser.add_argument("--calibration-samples", type=int, default=400)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--log-path")
    parser.add_argument("--quick", action="store_true", help="Use 15s phases for a short smoke test.")
    run_record(parser.parse_args())


if __name__ == "__main__":
    main()
