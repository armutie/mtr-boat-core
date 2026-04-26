from __future__ import annotations

import json
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

from .models import NavOutput


def output_to_record(output: NavOutput) -> dict:
    return {
        "timestamp": output.timestamp,
        "frame_number": output.frame_number,
        "raw_points": output.raw_points,
        "filtered_points": output.filtered_points,
        "clusters": [asdict(cluster) for cluster in output.clusters],
        "current": {
            "left": output.current_left,
            "front": output.current_front,
            "right": output.current_right,
        },
        "scores": {
            "left": output.left_score,
            "front": output.front_score,
            "right": output.right_score,
        },
        "control": {
            "target_throttle": output.target_throttle,
            "target_steering": output.target_steering,
            "throttle": output.throttle,
            "steering": output.steering,
        },
        "front_blocked": output.front_blocked,
        "command": output.command,
        "desired_command": output.desired_command,
        "reason": output.reason,
        "metadata": output.metadata,
    }


class JsonlNavLogger:
    def __init__(self, path: str | Path | None = None):
        if path is None:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = Path("logs") / f"radar_nav_{stamp}.jsonl"
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("a", encoding="utf-8")

    def write(self, output: NavOutput) -> None:
        self._file.write(json.dumps(output_to_record(output), separators=(",", ":")) + "\n")
        self._file.flush()

    def close(self) -> None:
        self._file.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
