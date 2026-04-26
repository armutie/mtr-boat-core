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


def choose_desired_command(state: NavState, cfg: NavConfig) -> tuple[Command, str]:
    if state.front_score > cfg.emergency_stop_thresh:
        return "stop", "front score above emergency stop threshold"

    if state.front_blocked:
        if state.left_score < state.right_score - cfg.side_margin:
            return "turn_left", "front blocked; left side has lower danger"
        if state.right_score < state.left_score - cfg.side_margin:
            return "turn_right", "front blocked; right side has lower danger"
        if state.command in ("turn_left", "turn_right"):
            return state.command, "front blocked; side scores tied, holding previous turn"
        return "stop", "front blocked; no clear safer side"

    return "forward", "front corridor below blocked threshold"


def apply_command_lock(state: NavState, desired: Command, now: float, cfg: NavConfig) -> Command:
    emergency = desired == "stop" and state.front_score > cfg.emergency_stop_thresh
    locked = now - state.last_command_time < cfg.command_lock_s

    if desired == state.command:
        return state.command
    if locked and not emergency:
        return state.command

    state.command = desired
    state.last_command_time = now
    return state.command
