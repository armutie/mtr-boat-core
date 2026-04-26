from .clustering import clamp
from .config import NavConfig
from .models import Command, NavState


def update_scores(
    state: NavState,
    current_left: float,
    current_front: float,
    current_right: float,
    cfg: NavConfig,
) -> None:
    alpha = cfg.alpha
    state.left_score = clamp((1.0 - alpha) * state.left_score + alpha * current_left)
    state.front_score = clamp((1.0 - alpha) * state.front_score + alpha * current_front)
    state.right_score = clamp((1.0 - alpha) * state.right_score + alpha * current_right)


def update_front_blocked(state: NavState, cfg: NavConfig) -> None:
    if state.front_score > cfg.front_on_thresh:
        state.front_blocked = True
    elif state.front_score < cfg.front_off_thresh:
        state.front_blocked = False


def compute_target_control(state: NavState, cfg: NavConfig) -> tuple[float, float, str]:
    front_pressure = clamp(state.front_score * cfg.front_throttle_weight)
    shared_side_pressure = min(state.left_score, state.right_score)

    target_throttle = 1.0 - front_pressure
    target_throttle *= 1.0 - cfg.side_throttle_weight * shared_side_pressure
    target_throttle = clamp(target_throttle)

    target_steering = clamp(state.left_score - state.right_score, -1.0, 1.0)
    if abs(target_steering) < cfg.steering_deadband:
        target_steering = 0.0
    if not state.front_blocked:
        target_steering *= cfg.free_steering_scale

    if state.front_blocked and shared_side_pressure > 0.65 and abs(target_steering) < cfg.side_margin:
        target_throttle *= 0.35
        reason = "blocked on front and both sides; bleeding throttle"
    elif state.front_blocked:
        direction = "left" if target_steering < 0 else "right" if target_steering > 0 else "straight"
        reason = f"front blocked; steering {direction} while reducing throttle"
    else:
        reason = "front corridor clear; recovering throttle"

    return target_throttle, target_steering, reason


def update_control(state: NavState, target_throttle: float, target_steering: float, cfg: NavConfig) -> None:
    throttle_alpha = cfg.throttle_down_alpha if target_throttle < state.throttle else cfg.throttle_up_alpha
    state.target_throttle = target_throttle
    state.target_steering = target_steering
    state.throttle = clamp((1.0 - throttle_alpha) * state.throttle + throttle_alpha * target_throttle)
    state.steering = clamp((1.0 - cfg.steering_alpha) * state.steering + cfg.steering_alpha * target_steering, -1.0, 1.0)


def command_from_control(state: NavState, cfg: NavConfig) -> Command:
    if state.throttle < cfg.stop_throttle_thresh:
        return "stop"
    if state.steering < -cfg.steering_command_thresh:
        return "turn_left"
    if state.steering > cfg.steering_command_thresh:
        return "turn_right"
    return "forward"
