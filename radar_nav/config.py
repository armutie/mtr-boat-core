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
    throttle_down_alpha: float = 0.18
    throttle_up_alpha: float = 0.06
    steering_alpha: float = 0.12
    front_throttle_weight: float = 1.0
    side_throttle_weight: float = 0.50
    steering_deadband: float = 0.08
    steering_command_thresh: float = 0.25
    stop_throttle_thresh: float = 0.18
    free_steering_scale: float = 0.35

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
        self.throttle_down_alpha = min(max(self.throttle_down_alpha, 0.01), 1.0)
        self.throttle_up_alpha = min(max(self.throttle_up_alpha, 0.01), 1.0)
        self.steering_alpha = min(max(self.steering_alpha, 0.01), 1.0)
        self.side_throttle_weight = min(max(self.side_throttle_weight, 0.0), 1.0)
        self.free_steering_scale = min(max(self.free_steering_scale, 0.0), 1.0)
        self.steering_command_thresh = min(max(self.steering_command_thresh, 0.0), 1.0)
        self.stop_throttle_thresh = min(max(self.stop_throttle_thresh, 0.0), 1.0)
