from __future__ import annotations

import time

from .clustering import cluster_points, clusters_to_emergency_evidence, clusters_to_evidence
from .config import NavConfig
from .decision import apply_command_lock, choose_desired_command, update_emergency_stop, update_front_blocked, update_scores
from .filtering import filter_points
from .models import NavOutput, NavState


class RadarNavPipeline:
    def __init__(self, cfg: NavConfig | None = None):
        self.cfg = cfg or NavConfig()
        self.state = NavState()

    def reset(self) -> None:
        self.state.reset()

    def process_points(
        self,
        points: list[dict],
        frame_number: int | None = None,
        now: float | None = None,
        metadata: dict | None = None,
    ) -> NavOutput:
        timestamp = time.time() if now is None else now
        filtered = filter_points(points, self.cfg)
        clusters = cluster_points(filtered, self.cfg)
        current_left, current_front, current_right = clusters_to_evidence(clusters, self.cfg)
        current_emergency = clusters_to_emergency_evidence(clusters, self.cfg)

        update_scores(self.state, current_left, current_front, current_right, current_emergency, self.cfg)
        update_front_blocked(self.state, self.cfg)
        update_emergency_stop(self.state, self.cfg)
        desired, reason = choose_desired_command(self.state, self.cfg)
        command = apply_command_lock(self.state, desired, timestamp, self.cfg)
        if command != desired:
            reason = f"command lock holding {command}; desired {desired}"

        return NavOutput(
            timestamp=timestamp,
            frame_number=frame_number,
            raw_points=list(points),
            filtered_points=filtered,
            clusters=clusters,
            current_left=current_left,
            current_front=current_front,
            current_right=current_right,
            current_emergency=current_emergency,
            left_score=self.state.left_score,
            front_score=self.state.front_score,
            right_score=self.state.right_score,
            emergency_score=self.state.emergency_score,
            front_blocked=self.state.front_blocked,
            emergency_stop=self.state.emergency_stop,
            command=command,
            desired_command=desired,
            reason=reason,
            metadata=metadata or {},
        )

    def process_frame(self, decoded_frame: dict, now: float | None = None) -> NavOutput:
        header = decoded_frame.get("header", {})
        return self.process_points(
            decoded_frame.get("combined_points", []),
            frame_number=header.get("frame_number"),
            now=now,
            metadata={
                "tlv_length_mode": decoded_frame.get("tlv_length_mode"),
                "trailing_padding": decoded_frame.get("trailing_padding"),
                "num_detected_obj": header.get("num_detected_obj"),
            },
        )
