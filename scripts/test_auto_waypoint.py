from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from gnss.geo import bearing_deg, distance_m, heading_error_deg
from gnss.replay import load_gnss_log
from web_dashboard.auto_controller import AutoConfig, AutoController


class _ControlState:
    pass


def main() -> None:
    parser = argparse.ArgumentParser(description="Offline waypoint-controller smoke test from a GNSS JSONL log.")
    parser.add_argument("log_path")
    parser.add_argument("--target-lat", type=float, required=True)
    parser.add_argument("--target-lon", type=float, required=True)
    parser.add_argument("--controller", default="smooth_pd_v1", choices=("guesstimate_rate_v1", "pulse_yaw_v1", "smooth_pd_v1"))
    parser.add_argument("--max-rows", type=int, default=40)
    args = parser.parse_args()

    fixes = [fix for fix in load_gnss_log(args.log_path) if fix.lat is not None and fix.lon is not None]
    controller = AutoController(_ControlState(), None, None, AutoConfig(controller=args.controller))
    print("idx,distance_m,bearing_deg,heading_deg,speed_mps,error_deg,left_us,right_us,action,reached")
    emitted = 0
    for index, fix in enumerate(fixes):
        if emitted >= args.max_rows:
            break
        if fix.heading_deg is None or fix.speed_mps is None or fix.speed_mps < controller.config.min_speed_for_course_mps:
            continue
        dist = distance_m(fix.lat, fix.lon, args.target_lat, args.target_lon)
        bearing = bearing_deg(fix.lat, fix.lon, args.target_lat, args.target_lon)
        error = heading_error_deg(fix.heading_deg, bearing)
        control = controller._compute_waypoint_control(  # noqa: SLF001 - smoke test intentionally probes live math
            dist,
            error,
            "unavailable",
            {},
        )
        print(
            f"{index},{dist:.2f},{bearing:.1f},{fix.heading_deg:.1f},{fix.speed_mps:.2f},{error:.1f},"
            f"{control.left_us},{control.right_us},{control.action},{control.reached}"
        )
        emitted += 1


if __name__ == "__main__":
    main()
