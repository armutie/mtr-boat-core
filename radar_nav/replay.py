from __future__ import annotations

import json
from pathlib import Path


def iter_log_records(path: str | Path):
    with Path(path).open("r", encoding="utf-8") as f:
        for line_number, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            record = json.loads(line)
            record["_line_number"] = line_number
            yield record


def record_to_decoded_frame(record: dict) -> dict:
    return {
        "header": {
            "frame_number": record.get("frame_number"),
            "num_detected_obj": len(record.get("raw_points", [])),
        },
        "combined_points": record.get("raw_points", []),
        "tlv_length_mode": record.get("metadata", {}).get("tlv_length_mode"),
        "trailing_padding": record.get("metadata", {}).get("trailing_padding"),
    }
