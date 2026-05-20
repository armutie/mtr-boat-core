from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from gnss.replay import load_gnss_log, summarize_gnss_log
from gnss.track_export import write_map_html, write_track_html


def fmt(value, digits: int = 2, suffix: str = "") -> str:
    if value is None:
        return "--"
    return f"{value:.{digits}f}{suffix}"


def fmt_coord(lat, lon) -> str:
    if lat is None or lon is None:
        return "--"
    return f"{lat:.7f}, {lon:.7f}"


def print_summary(path: str, summary) -> None:
    bounds = summary.bounds
    fix_counts = ", ".join(f"{name}={count}" for name, count in sorted(summary.fix_counts.items())) or "--"
    print(f"[GNSS replay] {path}")
    print(
        f"[GNSS replay] samples={summary.sample_count} positioned={summary.positioned_count} "
        f"duration={summary.duration_s:.2f}s avg_rate={summary.average_hz:.2f} Hz"
    )
    print(f"[GNSS replay] fixes={fix_counts}")
    print(f"[GNSS replay] first={fmt_coord(summary.first_lat, summary.first_lon)}")
    print(f"[GNSS replay] last={fmt_coord(summary.last_lat, summary.last_lon)}")
    print(
        "[GNSS replay] bounds="
        f"lat {fmt(bounds.min_lat, 7)}..{fmt(bounds.max_lat, 7)}, "
        f"lon {fmt(bounds.min_lon, 7)}..{fmt(bounds.max_lon, 7)}"
    )
    print(f"[GNSS replay] rough_distance={summary.distance_m:.1f} m")
    print(
        "[GNSS replay] speed="
        f"{fmt(summary.speed_min_mps)}..{fmt(summary.speed_max_mps)} m/s "
        f"avg={fmt(summary.speed_avg_mps)} m/s"
    )
    print(f"[GNSS replay] heading={fmt(summary.heading_min_deg, 1)}..{fmt(summary.heading_max_deg, 1)} deg")
    print(f"[GNSS replay] satellites={summary.satellites_min or '--'}..{summary.satellites_max or '--'}")
    print(f"[GNSS replay] hdop={fmt(summary.hdop_min)}..{fmt(summary.hdop_max)}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Replay a GNSS JSONL log.")
    parser.add_argument("log_path", help="Path to logs/gnss_*.jsonl")
    parser.add_argument("--html", action="store_true", help="Write a standalone HTML track visualization")
    parser.add_argument("--html-path", help="Custom HTML output path")
    parser.add_argument("--map", action="store_true", help="Write a Leaflet/OpenStreetMap HTML track view")
    parser.add_argument("--map-path", help="Custom map HTML output path")
    parser.add_argument("--speed", action="store_true", help="Color exported track segments by reported GNSS speed")
    args = parser.parse_args()

    fixes = load_gnss_log(args.log_path)
    summary = summarize_gnss_log(fixes)
    print_summary(args.log_path, summary)
    if args.html or args.html_path:
        path = write_track_html(args.log_path, fixes, summary, args.html_path, color_by_speed=args.speed)
        print(f"[GNSS replay] wrote {path}")
    if args.map or args.map_path:
        path = write_map_html(args.log_path, fixes, summary, args.map_path, color_by_speed=args.speed)
        print(f"[GNSS replay] wrote {path}")


if __name__ == "__main__":
    main()
