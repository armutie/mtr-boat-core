from dataclasses import dataclass


@dataclass
class NavConfig:
    min_y: float = 0.15
    max_y: float = 2.5
    lateral_limit: float = 1.2
    min_snr_raw: int | None = 120

    cluster_eps_m: float = 0.35
    cluster_min_points: int = 2
    keep_singletons: bool = True

    front_half_width: float = 0.25
    left_right_deadband: float = 0.10

    danger_near_y: float = 0.25
    danger_far_y: float = 2.5
    singleton_weight: float = 0.25
    cluster_weight: float = 1.0

    alpha: float = 0.10

    front_on_thresh: float = 0.70
    front_off_thresh: float = 0.40
    side_margin: float = 0.15

    command_lock_s: float = 0.35
    emergency_stop_thresh: float = 0.90

    viz_x_min: float = -1.5
    viz_x_max: float = 1.5
    viz_y_min: float = 0.0
    viz_y_max: float = 3.0

    def clamp_values(self) -> None:
        self.alpha = min(max(self.alpha, 0.01), 1.0)
        self.cluster_eps_m = max(self.cluster_eps_m, 0.05)
        if self.min_snr_raw is not None:
            self.min_snr_raw = max(self.min_snr_raw, 0)
        self.front_on_thresh = min(max(self.front_on_thresh, 0.05), 1.0)
        self.front_off_thresh = min(max(self.front_off_thresh, 0.0), self.front_on_thresh)
