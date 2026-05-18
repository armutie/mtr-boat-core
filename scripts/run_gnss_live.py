from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from boat_core.config import choose, load_boat_config, section
from gnss import NmeaReader


def apply_config(args) -> None:
    config = load_boat_config(args.config)
    gnss = section(config, "gnss")
    args.port = choose(args.port, gnss, "port")
    args.baud = choose(args.baud, gnss, "baud", 9600)


def default_log_path() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("logs") / f"gnss_{stamp}.jsonl"


def main() -> None:
    ap = argparse.ArgumentParser(description="Read GNSS NMEA sentences from a serial receiver.")
    ap.add_argument("--config", default="config/boat.local.json", help="Boat config JSON path")
    ap.add_argument("--port", help="GNSS serial port, e.g. /dev/ttyACM0")
    ap.add_argument("--baud", type=int, help="GNSS serial baud, often 9600")
    ap.add_argument("--log", action="store_true", help="Write parsed fixes to logs/ as JSONL")
    ap.add_argument("--log-path", help="Custom JSONL log path")
    args = ap.parse_args()
    apply_config(args)

    if not args.port:
        ap.error("--port is required unless set in --config")

    log_file = None
    if args.log or args.log_path:
        path = Path(args.log_path) if args.log_path else default_log_path()
        path.parent.mkdir(parents=True, exist_ok=True)
        log_file = path.open("a", encoding="utf-8")
        print(f"[GNSS] Logging to {path}")

    print(f"[GNSS] Opening {args.port} @ {args.baud}. Ctrl+C to exit.")
    try:
        with NmeaReader(args.port, baud=args.baud) as reader:
            while True:
                fix = reader.read_fix()
                if fix is None:
                    continue
                record = fix.to_record()
                if log_file is not None:
                    log_file.write(json.dumps(record, separators=(",", ":")) + "\n")
                    log_file.flush()
                lat = "--" if fix.lat is None else f"{fix.lat:.7f}"
                lon = "--" if fix.lon is None else f"{fix.lon:.7f}"
                speed = "--" if fix.speed_mps is None else f"{fix.speed_mps:.2f} m/s"
                heading = "--" if fix.heading_deg is None else f"{fix.heading_deg:.1f} deg"
                sats = "--" if fix.satellites is None else str(fix.satellites)
                print(f"[GNSS] fix={fix.fix} lat={lat} lon={lon} speed={speed} heading={heading} sats={sats}")
    except KeyboardInterrupt:
        print("\n[GNSS] Stopped.")
    finally:
        if log_file is not None:
            log_file.close()


if __name__ == "__main__":
    main()
