from dataclasses import dataclass, field
from typing import Literal


Zone = Literal["left", "front", "right", "unknown"]
Command = Literal["forward", "turn_left", "turn_right", "stop"]


@dataclass
class RadarCluster:
    points: list[dict]
    cx: float
    cy: float
    cz: float
    mean_doppler: float
    mean_snr_raw: float | None
    count: int
    is_singleton: bool
    confidence: float
    zone: Zone


@dataclass
class NavState:
    left_score: float = 0.0
    front_score: float = 0.0
    right_score: float = 0.0
    front_blocked: bool = False
    target_throttle: float = 1.0
    target_steering: float = 0.0
    throttle: float = 0.0
    steering: float = 0.0
    command: Command = "stop"
    last_command_time: float = 0.0

    def reset(self) -> None:
        self.left_score = 0.0
        self.front_score = 0.0
        self.right_score = 0.0
        self.front_blocked = False
        self.target_throttle = 1.0
        self.target_steering = 0.0
        self.throttle = 0.0
        self.steering = 0.0
        self.command = "stop"
        self.last_command_time = 0.0


@dataclass
class NavOutput:
    timestamp: float
    frame_number: int | None
    raw_points: list[dict]
    filtered_points: list[dict]
    clusters: list[RadarCluster]
    current_left: float
    current_front: float
    current_right: float
    left_score: float
    front_score: float
    right_score: float
    front_blocked: bool
    target_throttle: float
    target_steering: float
    throttle: float
    steering: float
    command: Command
    desired_command: Command
    reason: str
    metadata: dict = field(default_factory=dict)
