from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from web_dashboard.auto_controller import AutoConfig, AutoController


class _ControlState:
    pass


def main() -> None:
    parser = argparse.ArgumentParser(description="Inspect the v1 auto waypoint guesstimate controller.")
    parser.add_argument("--controller", default="pulse_yaw_v1", choices=("guesstimate_rate_v1", "pulse_yaw_v1"))
    parser.add_argument("--distance-m", type=float, default=20.0)
    parser.add_argument("--heading-error-deg", type=float, required=True)
    parser.add_argument("--yaw-rate-dps", type=float, default=0.0)
    parser.add_argument("--steps", type=int, default=1)
    parser.add_argument("--step-s", type=float, default=0.1)
    args = parser.parse_args()

    controller = AutoController(_ControlState(), None, None, AutoConfig(controller=args.controller))
    print(f"distance_m={args.distance_m:.2f}")
    print(f"controller={args.controller}")
    print(f"heading_error_deg={args.heading_error_deg:.2f}")
    print(f"yaw_rate_dps={args.yaw_rate_dps:.2f}")
    for index in range(args.steps):
        control = controller._compute_waypoint_control(  # noqa: SLF001 - script intentionally probes controller math
            args.distance_m,
            args.heading_error_deg,
            "live",
            {"gyro_z_dps": args.yaw_rate_dps, "age_s": 0.0},
        )
        print(
            f"step={index} left_us={control.left_us} right_us={control.right_us} "
            f"action={control.action} metadata={control.metadata}"
        )
        if index + 1 < args.steps:
            time.sleep(args.step_s)


if __name__ == "__main__":
    main()
