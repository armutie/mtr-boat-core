
import argparse
from collections import Counter, deque
import math

from mmwave_uart import count_points_in_box
from mmwave_uart import MmwaveUartParser
from mmwave_uart import merge_nav_config
from mmwave_uart import send_cfg


DEFAULT_ROBOT_NAV_CONFIG = merge_nav_config({
    "filter_min_y": 0.10,
    "filter_max_y": 1.20,
    "filter_lateral_limit": 0.70,
    "filter_min_snr_raw": 200,
    "ahead_min_y": 0.20,
    "ahead_lateral_limit": 0.30,
    "danger_box": {
        "x_min": -0.25,
        "x_max": 0.25,
        "y_min": 0.20,
        "y_max": 0.70,
    },
    "density_y_min": 0.20,
    "density_y_max": 1.20,
})

DEFAULT_DECISION_CONFIG = {
    "front_box": {
        "x_min": -0.22,
        "x_max": 0.22,
        "y_min": 0.20,
        "y_max": 0.75,
    },
    "left_box": {
        "x_min": -0.60,
        "x_max": -0.12,
        "y_min": 0.20,
        "y_max": 0.90,
    },
    "right_box": {
        "x_min": 0.12,
        "x_max": 0.60,
        "y_min": 0.20,
        "y_max": 0.90,
    },
    "front_min_points": 5,
    "side_min_points": 5,
    "stop_distance_y": 0.30,
    "emergency_distance_y": 0.22,
    "side_bias_threshold": 2,
    "front_score_threshold": 10.0,
    "front_clear_score_threshold": 6.0,
    "side_score_blocked_threshold": 7.0,
    "side_score_margin": 1.6,
    "center_close_min_points": 2,
    "emergency_min_points": 2,
    "score_distance_floor": 0.18,
    "score_snr_floor": 180,
    "switch_confirmation_frames": 3,
    "turn_to_forward_confirmation_frames": 2,
    "stop_confirmation_frames": 2,
}


def merge_decision_config(overrides=None):
    config = {
        "front_box": dict(DEFAULT_DECISION_CONFIG["front_box"]),
        "left_box": dict(DEFAULT_DECISION_CONFIG["left_box"]),
        "right_box": dict(DEFAULT_DECISION_CONFIG["right_box"]),
    }
    for key, value in DEFAULT_DECISION_CONFIG.items():
        if key not in {"front_box", "left_box", "right_box"}:
            config[key] = value

    if not overrides:
        return config

    for key, value in overrides.items():
        if key in {"front_box", "left_box", "right_box"} and isinstance(value, dict):
            config[key].update(value)
        else:
            config[key] = value
    return config


def point_distance(point):
    return math.sqrt(point["x"] ** 2 + point["y"] ** 2 + point["z"] ** 2)


def point_in_box(point, box):
    return (
        box["x_min"] <= point["x"] <= box["x_max"]
        and box["y_min"] <= point["y"] <= box["y_max"]
    )


def weighted_point_score(point, config):
    y = max(config["score_distance_floor"], point["y"])
    closeness = 1.0 / y
    snr_raw = point.get("snr_raw", config["score_snr_floor"])
    snr_bonus = 1.0 + max(0.0, (snr_raw - config["score_snr_floor"]) / 180.0)
    return closeness * min(1.6, snr_bonus)


def weighted_box_score(points, box, config):
    score = 0.0
    count = 0
    for point in points:
        if point_in_box(point, box):
            score += weighted_point_score(point, config)
            count += 1
    return score, count


def count_center_close_points(points, y_limit, lateral_limit):
    return sum(
        1
        for point in points
        if abs(point["x"]) <= lateral_limit and 0.0 <= point["y"] <= y_limit
    )


def weighted_left_right_density(points, y_min=0.0, y_max=1.5):
    left = 0.0
    right = 0.0
    for point in points:
        if not (y_min <= point["y"] <= y_max):
            continue
        weight = 1.0 / max(0.18, point["y"])
        if point["x"] < 0.0:
            left += weight
        elif point["x"] > 0.0:
            right += weight
    return {"left": left, "right": right}


def choose_turn_hint(weighted_density, bias_threshold: float = 1.5):
    left = weighted_density["left"]
    right = weighted_density["right"]
    delta = left - right

    if abs(delta) < bias_threshold:
        return "straight"
    if delta > 0:
        return "right"
    return "left"


def classify_obstacle_zones(filtered_points, nearest_ahead, decision_config=None):
    config = merge_decision_config(decision_config)
    front_score, front_count = weighted_box_score(filtered_points, config["front_box"], config)
    left_score, left_count = weighted_box_score(filtered_points, config["left_box"], config)
    right_score, right_count = weighted_box_score(filtered_points, config["right_box"], config)
    nearest_y = nearest_ahead["y"] if nearest_ahead else None

    center_close_count = count_center_close_points(
        filtered_points,
        y_limit=config["stop_distance_y"],
        lateral_limit=max(abs(config["front_box"]["x_min"]), abs(config["front_box"]["x_max"])),
    )
    emergency_count = count_center_close_points(
        filtered_points,
        y_limit=config["emergency_distance_y"],
        lateral_limit=max(abs(config["front_box"]["x_min"]), abs(config["front_box"]["x_max"])),
    )

    front_blocked = (
        front_count >= config["front_min_points"]
        or front_score >= config["front_score_threshold"]
        or center_close_count >= config["center_close_min_points"]
    )
    front_preferred_clear = (
        front_count < max(1, config["front_min_points"] - 1)
        and front_score < config["front_clear_score_threshold"]
        and center_close_count < config["center_close_min_points"]
    )
    left_blocked = (
        left_count >= config["side_min_points"]
        or left_score >= config["side_score_blocked_threshold"]
    )
    right_blocked = (
        right_count >= config["side_min_points"]
        or right_score >= config["side_score_blocked_threshold"]
    )
    emergency_stop = (
        emergency_count >= config["emergency_min_points"]
        or (
            nearest_y is not None
            and nearest_y <= config["emergency_distance_y"]
            and front_count >= config["emergency_min_points"]
        )
    )

    return {
        "config": config,
        "front_count": front_count,
        "left_count": left_count,
        "right_count": right_count,
        "front_score": front_score,
        "left_score": left_score,
        "right_score": right_score,
        "front_blocked": front_blocked,
        "front_preferred_clear": front_preferred_clear,
        "left_blocked": left_blocked,
        "right_blocked": right_blocked,
        "nearest_y": nearest_y,
        "center_close_count": center_close_count,
        "emergency_count": emergency_count,
        "emergency_stop": emergency_stop,
    }


def decide_motion(zones, turn_hint: str):
    front_count = zones["front_count"]
    left_count = zones["left_count"]
    right_count = zones["right_count"]
    front_score = zones["front_score"]
    left_score = zones["left_score"]
    right_score = zones["right_score"]
    front_blocked = zones["front_blocked"]
    front_preferred_clear = zones["front_preferred_clear"]
    left_blocked = zones["left_blocked"]
    right_blocked = zones["right_blocked"]
    nearest_y = zones["nearest_y"]
    emergency_stop = zones["emergency_stop"]
    center_close_count = zones["center_close_count"]
    bias_threshold = zones["config"]["side_bias_threshold"]
    side_score_margin = zones["config"]["side_score_margin"]

    if emergency_stop and left_blocked and right_blocked:
        return {
            "command": "STOP",
            "reason": (
                f"emergency stop; front has {zones['emergency_count']} very close points and both sides look occupied"
            ),
        }

    if (not front_blocked) and front_preferred_clear:
        if nearest_y is None:
            reason = f"front clear; front_score={front_score:.1f}, front_count={front_count}"
        else:
            reason = (
                f"front clear; nearest centered obstacle at y={nearest_y:.3f} m, "
                f"front_score={front_score:.1f}"
            )
        return {"command": "FORWARD", "reason": reason}

    if not left_blocked and right_blocked:
        return {
            "command": "LEFT",
            "reason": (
                f"front constrained; right side looks busier "
                f"(right_score={right_score:.1f}, left_score={left_score:.1f})"
            ),
        }

    if not right_blocked and left_blocked:
        return {
            "command": "RIGHT",
            "reason": (
                f"front constrained; left side looks busier "
                f"(left_score={left_score:.1f}, right_score={right_score:.1f})"
            ),
        }

    if left_blocked and right_blocked:
        score_delta = left_score - right_score
        if abs(score_delta) >= side_score_margin:
            if score_delta < 0:
                return {
                    "command": "LEFT",
                    "reason": (
                        f"front blocked; both sides occupied, but left score is lower "
                        f"({left_score:.1f}<{right_score:.1f})"
                    ),
                }
            return {
                "command": "RIGHT",
                "reason": (
                    f"front blocked; both sides occupied, but right score is lower "
                    f"({right_score:.1f}<{left_score:.1f})"
                ),
            }
        return {
            "command": "STOP",
            "reason": (
                f"front blocked and both sides look similarly occupied "
                f"(left_score={left_score:.1f}, right_score={right_score:.1f})"
            ),
        }

    if left_score + side_score_margin < right_score:
        return {
            "command": "LEFT",
            "reason": (
                f"front constrained; left side score is meaningfully lower "
                f"({left_score:.1f}<{right_score:.1f})"
            ),
        }
    if right_score + side_score_margin < left_score:
        return {
            "command": "RIGHT",
            "reason": (
                f"front constrained; right side score is meaningfully lower "
                f"({right_score:.1f}<{left_score:.1f})"
            ),
        }

    if abs(left_count - right_count) >= bias_threshold:
        if left_count < right_count:
            return {
                "command": "LEFT",
                "reason": f"front constrained; left count is lower ({left_count}<{right_count})",
            }
        return {
            "command": "RIGHT",
            "reason": f"front constrained; right count is lower ({right_count}<{left_count})",
        }

    if turn_hint == "left":
        return {
            "command": "LEFT",
            "reason": (
                f"front constrained; near-field counts tie, so using weighted density hint "
                f"with center_close_count={center_close_count}"
            ),
        }
    if turn_hint == "right":
        return {
            "command": "RIGHT",
            "reason": (
                f"front constrained; near-field counts tie, so using weighted density hint "
                f"with center_close_count={center_close_count}"
            ),
        }

    if nearest_y is not None and nearest_y <= zones["config"]["stop_distance_y"]:
        return {
            "command": "STOP",
            "reason": (
                f"front constrained with a centered obstacle at y={nearest_y:.3f} m and no clear turn preference"
            ),
        }

    return {
        "command": "FORWARD",
        "reason": (
            f"front is only weakly constrained; keeping forward until a turn is consistently better "
            f"(front_score={front_score:.1f})"
        ),
    }


def build_robot_nav_state(
    decoded_frame,
    stop_danger_count: int = 5,
    turn_bias_threshold: int = 2,
    decision_config=None,
):
    nav = decoded_frame["navigation"]
    nearest = nav["nearest_ahead"]
    weighted_density = weighted_left_right_density(
        nav["filtered_points"],
        y_min=0.20,
        y_max=1.20,
    )
    turn_hint = choose_turn_hint(weighted_density, bias_threshold=1.5)
    zones = classify_obstacle_zones(nav["filtered_points"], nearest, decision_config=decision_config)
    decision = decide_motion(zones, turn_hint=turn_hint)

    return {
        "frame_number": decoded_frame["header"]["frame_number"],
        "points_total": len(decoded_frame["combined_points"]),
        "points_filtered": nav["filtered_count"],
        "nearest_ahead": nearest,
        "nearest_ahead_distance": point_distance(nearest) if nearest else None,
        "danger_box_count": nav["danger_box_count"],
        "left_density": nav["left_right_density"]["left"],
        "right_density": nav["left_right_density"]["right"],
        "weighted_left_density": weighted_density["left"],
        "weighted_right_density": weighted_density["right"],
        "turn_hint": turn_hint,
        "stop": nav["danger_box_count"] >= stop_danger_count,
        "zones": zones,
        "decision": decision,
        "command": decision["command"],
        "reason": decision["reason"],
        "raw_frame": decoded_frame,
    }


class CommandVoteSmoother:
    def __init__(
        self,
        window_size: int = 9,
        switch_confirmation_frames: int = 3,
        turn_to_forward_confirmation_frames: int = 2,
        stop_confirmation_frames: int = 2,
    ):
        self.window = deque(maxlen=max(1, window_size))
        self.last_output = None
        self.candidate_command = None
        self.candidate_count = 0
        self.switch_confirmation_frames = max(1, switch_confirmation_frames)
        self.turn_to_forward_confirmation_frames = max(1, turn_to_forward_confirmation_frames)
        self.stop_confirmation_frames = max(1, stop_confirmation_frames)

    def _required_confirmations(self, current, proposed, state):
        if state["zones"]["emergency_stop"] and proposed == "STOP":
            return 1
        if proposed == "STOP":
            return self.stop_confirmation_frames
        if current in {"LEFT", "RIGHT"} and proposed == "FORWARD":
            return self.turn_to_forward_confirmation_frames
        if current in {"LEFT", "RIGHT"} and proposed in {"LEFT", "RIGHT"} and proposed != current:
            return self.switch_confirmation_frames + 1
        return self.switch_confirmation_frames

    def update(self, state):
        raw_command = state["decision"]["command"]
        raw_reason = state["decision"]["reason"]
        self.window.append(raw_command)
        counts = Counter(self.window)

        if self.last_output is None:
            voted_command = raw_command
            self.last_output = voted_command
            self.candidate_command = None
            self.candidate_count = 0
        elif state["zones"]["emergency_stop"] and raw_command == "STOP":
            voted_command = "STOP"
            self.last_output = voted_command
            self.candidate_command = None
            self.candidate_count = 0
        elif raw_command == self.last_output:
            voted_command = self.last_output
            self.candidate_command = None
            self.candidate_count = 0
        else:
            if raw_command == self.candidate_command:
                self.candidate_count += 1
            else:
                self.candidate_command = raw_command
                self.candidate_count = 1

            required = self._required_confirmations(self.last_output, raw_command, state)
            if self.candidate_count >= required:
                voted_command = raw_command
                self.last_output = voted_command
                self.candidate_command = None
                self.candidate_count = 0
            else:
                voted_command = self.last_output

        if voted_command == raw_command:
            stable_reason = raw_reason
        else:
            stable_reason = (
                f"sticky smoother kept {voted_command} over raw {raw_command} "
                f"(candidate={self.candidate_command}, count={self.candidate_count}, window={list(self.window)})"
            )

        updated_state = dict(state)
        updated_state["decision"] = dict(state["decision"])
        updated_state["raw_command"] = raw_command
        updated_state["raw_reason"] = raw_reason
        updated_state["smoothed_command"] = voted_command
        updated_state["smoothed_reason"] = stable_reason
        updated_state["command_vote"] = {
            "window_size": len(self.window),
            "history": list(self.window),
            "counts": dict(counts),
            "candidate_command": self.candidate_command,
            "candidate_count": self.candidate_count,
        }
        updated_state["command"] = voted_command
        updated_state["reason"] = stable_reason
        updated_state["decision"]["raw_command"] = raw_command
        updated_state["decision"]["raw_reason"] = raw_reason
        updated_state["decision"]["smoothed_command"] = voted_command
        updated_state["decision"]["smoothed_reason"] = stable_reason
        return updated_state


class RobotRadarNavigator:
    def __init__(
        self,
        data_port: str,
        cfg_port: str = None,
        cfg_file: str = None,
        data_baud: int = 921600,
        nav_config=None,
        decision_config=None,
        stop_danger_count: int = 5,
        turn_bias_threshold: int = 2,
        command_vote_window: int = 9,
    ):
        self.data_port = data_port
        self.cfg_port = cfg_port
        self.cfg_file = cfg_file
        self.data_baud = data_baud
        self.nav_config = merge_nav_config(nav_config or DEFAULT_ROBOT_NAV_CONFIG)
        self.decision_config = merge_decision_config(decision_config)
        self.stop_danger_count = stop_danger_count
        self.turn_bias_threshold = turn_bias_threshold
        self.command_smoother = CommandVoteSmoother(
            window_size=command_vote_window,
            switch_confirmation_frames=self.decision_config["switch_confirmation_frames"],
            turn_to_forward_confirmation_frames=self.decision_config["turn_to_forward_confirmation_frames"],
            stop_confirmation_frames=self.decision_config["stop_confirmation_frames"],
        )
        self.reader = None

    def start(self):
        if self.cfg_port and self.cfg_file:
            send_cfg(self.cfg_port, self.cfg_file)
        self.reader = MmwaveUartParser(self.data_port, baud=self.data_baud)
        return self

    def close(self):
        if self.reader is not None:
            self.reader.close()
            self.reader = None

    def read(self):
        if self.reader is None:
            raise RuntimeError("RobotRadarNavigator.start() must be called before read()")

        decoded = self.reader.read_decoded_frame(nav_config=self.nav_config)
        if decoded is None:
            return None
        state = build_robot_nav_state(
            decoded,
            stop_danger_count=self.stop_danger_count,
            turn_bias_threshold=self.turn_bias_threshold,
            decision_config=self.decision_config,
        )
        return self.command_smoother.update(state)

    def iter_reads(self, frames=None):
        count = 0
        while frames is None or count < frames:
            state = self.read()
            if state is not None:
                yield state
                count += 1


def _fake_decoded_frame(filtered_points, nearest_ahead, frame_number=1):
    return {
        "header": {"frame_number": frame_number},
        "combined_points": filtered_points,
        "navigation": {
            "filtered_points": filtered_points,
            "filtered_count": len(filtered_points),
            "nearest_ahead": nearest_ahead,
            "danger_box_count": 0,
            "left_right_density": {
                "left": sum(1 for point in filtered_points if point["x"] < 0.0),
                "right": sum(1 for point in filtered_points if point["x"] > 0.0),
            },
        },
    }


def run_self_test():
    clear_points = [
        {"x": -0.45, "y": 0.55, "z": 0.0, "snr_raw": 240},
        {"x": 0.42, "y": 0.62, "z": 0.0, "snr_raw": 240},
    ]
    clear_frame = _fake_decoded_frame(clear_points, nearest_ahead=None, frame_number=1)
    clear_state = build_robot_nav_state(clear_frame)
    assert clear_state["command"] == "FORWARD"

    left_turn_points = [
        {"x": 0.00, "y": 0.24, "z": 0.0, "snr_raw": 245},
        {"x": 0.05, "y": 0.28, "z": 0.0, "snr_raw": 245},
        {"x": -0.45, "y": 0.55, "z": 0.0, "snr_raw": 240},
        {"x": 0.28, "y": 0.36, "z": 0.0, "snr_raw": 245},
        {"x": 0.34, "y": 0.48, "z": 0.0, "snr_raw": 245},
        {"x": 0.30, "y": 0.62, "z": 0.0, "snr_raw": 245},
        {"x": 0.26, "y": 0.73, "z": 0.0, "snr_raw": 245},
    ]
    left_turn_frame = _fake_decoded_frame(
        left_turn_points,
        nearest_ahead={"x": 0.00, "y": 0.24, "z": 0.0, "snr_raw": 245},
        frame_number=2,
    )
    left_turn_state = build_robot_nav_state(left_turn_frame)
    assert left_turn_state["command"] == "LEFT"

    stop_points = [
        {"x": -0.02, "y": 0.22, "z": 0.0, "snr_raw": 250},
        {"x": 0.04, "y": 0.24, "z": 0.0, "snr_raw": 250},
        {"x": -0.16, "y": 0.31, "z": 0.0, "snr_raw": 245},
        {"x": -0.22, "y": 0.45, "z": 0.0, "snr_raw": 240},
        {"x": -0.28, "y": 0.58, "z": 0.0, "snr_raw": 240},
        {"x": -0.34, "y": 0.70, "z": 0.0, "snr_raw": 240},
        {"x": 0.18, "y": 0.33, "z": 0.0, "snr_raw": 245},
        {"x": 0.24, "y": 0.46, "z": 0.0, "snr_raw": 240},
        {"x": 0.31, "y": 0.59, "z": 0.0, "snr_raw": 240},
        {"x": 0.38, "y": 0.71, "z": 0.0, "snr_raw": 240},
    ]
    stop_frame = _fake_decoded_frame(
        stop_points,
        nearest_ahead={"x": 0.00, "y": 0.22, "z": 0.0, "snr_raw": 250},
        frame_number=3,
    )
    stop_state = build_robot_nav_state(stop_frame)
    assert stop_state["command"] == "STOP"

    smoother = CommandVoteSmoother(window_size=9)
    seq = [
        {"decision": {"command": "FORWARD", "reason": "raw forward"}, "zones": {"emergency_stop": False}},
        {"decision": {"command": "LEFT", "reason": "raw left"}, "zones": {"emergency_stop": False}},
        {"decision": {"command": "FORWARD", "reason": "raw forward"}, "zones": {"emergency_stop": False}},
        {"decision": {"command": "LEFT", "reason": "raw left"}, "zones": {"emergency_stop": False}},
        {"decision": {"command": "LEFT", "reason": "raw left"}, "zones": {"emergency_stop": False}},
    ]
    smoothed = None
    for sample in seq:
        smoothed = smoother.update(sample)
    assert smoothed["command"] in {"FORWARD", "LEFT"}

    emergency = smoother.update({
        "decision": {"command": "STOP", "reason": "raw stop"},
        "zones": {"emergency_stop": True},
    })
    assert emergency["command"] == "STOP"
    print("[SELFTEST] PASS")


def main():
    ap = argparse.ArgumentParser(description="High-level robot navigation wrapper for TI mmWave UART data")
    ap.add_argument("--cfg-port", help="COM port for CFG / CLI UART, e.g. COM6")
    ap.add_argument("--cfg-file", help="Path to .cfg file")
    ap.add_argument("--data-port", help="COM port for DATA UART, e.g. COM5")
    ap.add_argument("--frames", type=int, default=10, help="Number of navigation frames to print")
    ap.add_argument("--self-test", action="store_true", help="Run robot navigation self-test and exit")
    args = ap.parse_args()

    if args.self_test:
        run_self_test()
        return

    if not args.data_port:
        ap.error("--data-port is required unless --self-test is used")

    radar = RobotRadarNavigator(
        data_port=args.data_port,
        cfg_port=args.cfg_port,
        cfg_file=args.cfg_file,
    )

    try:
        radar.start()
        for state in radar.iter_reads(frames=args.frames):
            print(
                f"[NAV] frame={state['frame_number']} "
                f"command={state['command']} "
                f"filtered={state['points_filtered']}/{state['points_total']} "
                f"danger={state['danger_box_count']} "
                f"left={state['left_density']} "
                f"right={state['right_density']} "
                f"turn={state['turn_hint']} "
                f"stop={state['stop']}"
            )
            print(
                f"      zones: front={state['zones']['front_count']} "
                f"left={state['zones']['left_count']} "
                f"right={state['zones']['right_count']} "
                f"front_score={state['zones']['front_score']:.1f} "
                f"left_score={state['zones']['left_score']:.1f} "
                f"right_score={state['zones']['right_score']:.1f} "
                f"front_blocked={state['zones']['front_blocked']}"
            )
            nearest = state["nearest_ahead"]
            if nearest is None:
                print("      nearest_ahead=None")
            else:
                print(
                    f"      nearest_ahead: x={nearest['x']:.3f} y={nearest['y']:.3f} "
                    f"dist={state['nearest_ahead_distance']:.3f}"
                )
            print(f"      reason: {state['reason']}")
    finally:
        radar.close()


if __name__ == "__main__":
    main()
