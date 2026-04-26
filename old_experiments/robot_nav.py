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
    "filter_min_snr_raw": 230,
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
    "front_min_points": 4,
    "side_min_points": 4,
    "stop_distance_y": 0.35,
    "emergency_distance_y": 0.25,
    "side_bias_threshold": 2,
}


def merge_decision_config(overrides=None):
    config = {
        "front_box": dict(DEFAULT_DECISION_CONFIG["front_box"]),
        "left_box": dict(DEFAULT_DECISION_CONFIG["left_box"]),
        "right_box": dict(DEFAULT_DECISION_CONFIG["right_box"]),
        "front_min_points": DEFAULT_DECISION_CONFIG["front_min_points"],
        "side_min_points": DEFAULT_DECISION_CONFIG["side_min_points"],
        "stop_distance_y": DEFAULT_DECISION_CONFIG["stop_distance_y"],
        "emergency_distance_y": DEFAULT_DECISION_CONFIG["emergency_distance_y"],
        "side_bias_threshold": DEFAULT_DECISION_CONFIG["side_bias_threshold"],
    }
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


def choose_turn_hint(left_right_density, bias_threshold: int = 2):
    left = left_right_density["left"]
    right = left_right_density["right"]
    delta = left - right

    if abs(delta) < bias_threshold:
        return "straight"
    if delta > 0:
        return "right"
    return "left"


def classify_obstacle_zones(filtered_points, nearest_ahead, decision_config=None):
    config = merge_decision_config(decision_config)
    front_count = count_points_in_box(filtered_points, **config["front_box"])
    left_count = count_points_in_box(filtered_points, **config["left_box"])
    right_count = count_points_in_box(filtered_points, **config["right_box"])
    nearest_y = nearest_ahead["y"] if nearest_ahead else None

    front_blocked = (
        front_count >= config["front_min_points"] or
        (nearest_y is not None and nearest_y <= config["stop_distance_y"])
    )
    left_blocked = left_count >= config["side_min_points"]
    right_blocked = right_count >= config["side_min_points"]
    emergency_stop = nearest_y is not None and nearest_y <= config["emergency_distance_y"]

    return {
        "config": config,
        "front_count": front_count,
        "left_count": left_count,
        "right_count": right_count,
        "front_blocked": front_blocked,
        "left_blocked": left_blocked,
        "right_blocked": right_blocked,
        "nearest_y": nearest_y,
        "emergency_stop": emergency_stop,
    }


def decide_motion(zones, turn_hint: str):
    front_count = zones["front_count"]
    left_count = zones["left_count"]
    right_count = zones["right_count"]
    front_blocked = zones["front_blocked"]
    left_blocked = zones["left_blocked"]
    right_blocked = zones["right_blocked"]
    nearest_y = zones["nearest_y"]
    emergency_stop = zones["emergency_stop"]
    bias_threshold = zones["config"]["side_bias_threshold"]

    if not front_blocked:
        if nearest_y is None:
            reason = "front clear with no close obstacle in the center lane"
        else:
            reason = f"front clear; nearest centered obstacle at y={nearest_y:.3f} m"
        return {"command": "FORWARD", "reason": reason}

    if emergency_stop and left_blocked and right_blocked:
        return {
            "command": "STOP",
            "reason": (
                f"emergency stop; obstacle ahead at y={nearest_y:.3f} m and both sides are occupied"
            ),
        }

    if left_blocked and right_blocked:
        if abs(left_count - right_count) >= bias_threshold:
            if left_count < right_count:
                return {
                    "command": "LEFT",
                    "reason": (
                        f"front blocked; both sides occupied, but left is clearer ({left_count}<{right_count})"
                    ),
                }
            return {
                "command": "RIGHT",
                "reason": (
                    f"front blocked; both sides occupied, but right is clearer ({right_count}<{left_count})"
                ),
            }
        return {
            "command": "STOP",
            "reason": (
                f"front blocked and side occupancy is similar (left={left_count}, right={right_count})"
            ),
        }

    if left_blocked and not right_blocked:
        return {
            "command": "RIGHT",
            "reason": f"front blocked and left side is busier (left={left_count}, right={right_count})",
        }

    if right_blocked and not left_blocked:
        return {
            "command": "LEFT",
            "reason": f"front blocked and right side is busier (right={right_count}, left={left_count})",
        }

    if left_count < right_count:
        return {
            "command": "LEFT",
            "reason": f"front blocked; left side is clearer ({left_count}<{right_count})",
        }
    if right_count < left_count:
        return {
            "command": "RIGHT",
            "reason": f"front blocked; right side is clearer ({right_count}<{left_count})",
        }
    if turn_hint == "left":
        return {"command": "LEFT", "reason": "front blocked; side counts tie so using overall left/right density"}
    if turn_hint == "right":
        return {"command": "RIGHT", "reason": "front blocked; side counts tie so using overall left/right density"}
    return {"command": "STOP", "reason": "front blocked and no clear turn preference"}


def build_robot_nav_state(
    decoded_frame,
    stop_danger_count: int = 5,
    turn_bias_threshold: int = 2,
    decision_config=None,
):
    nav = decoded_frame["navigation"]
    nearest = nav["nearest_ahead"]
    turn_hint = choose_turn_hint(nav["left_right_density"], bias_threshold=turn_bias_threshold)
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
        "turn_hint": turn_hint,
        "stop": nav["danger_box_count"] >= stop_danger_count,
        "zones": zones,
        "decision": decision,
        "command": decision["command"],
        "reason": decision["reason"],
        "raw_frame": decoded_frame,
    }


class CommandVoteSmoother:
    def __init__(self, window_size: int = 5):
        self.window = deque(maxlen=max(1, window_size))
        self.last_output = None
        self.priority = ("STOP", "LEFT", "RIGHT", "FORWARD")

    def update(self, state):
        raw_command = state["decision"]["command"]
        raw_reason = state["decision"]["reason"]

        if raw_command == "STOP" and state["zones"]["emergency_stop"]:
            self.window.append(raw_command)
            voted_command = "STOP"
            counts = Counter(self.window)
        else:
            self.window.append(raw_command)
            counts = Counter(self.window)
            highest = max(counts.values())
            candidates = [command for command, count in counts.items() if count == highest]

            if self.last_output in candidates:
                voted_command = self.last_output
            else:
                voted_command = next(
                    command for command in self.priority
                    if command in candidates
                )

        self.last_output = voted_command

        if voted_command == raw_command:
            stable_reason = raw_reason
        else:
            stable_reason = (
                f"majority vote kept {voted_command} over raw {raw_command} "
                f"(window={list(self.window)})"
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
        command_vote_window: int = 5,
    ):
        self.data_port = data_port
        self.cfg_port = cfg_port
        self.cfg_file = cfg_file
        self.data_baud = data_baud
        self.nav_config = merge_nav_config(nav_config or DEFAULT_ROBOT_NAV_CONFIG)
        self.decision_config = merge_decision_config(decision_config)
        self.stop_danger_count = stop_danger_count
        self.turn_bias_threshold = turn_bias_threshold
        self.command_smoother = CommandVoteSmoother(window_size=command_vote_window)
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
        {"x": -0.45, "y": 0.55, "z": 0.0},
        {"x": 0.42, "y": 0.62, "z": 0.0},
    ]
    clear_frame = _fake_decoded_frame(clear_points, nearest_ahead=None, frame_number=1)
    clear_state = build_robot_nav_state(clear_frame)
    assert clear_state["command"] == "FORWARD"

    left_turn_points = [
        {"x": 0.00, "y": 0.24, "z": 0.0},
        {"x": 0.05, "y": 0.28, "z": 0.0},
        {"x": -0.45, "y": 0.55, "z": 0.0},
        {"x": 0.28, "y": 0.36, "z": 0.0},
        {"x": 0.34, "y": 0.48, "z": 0.0},
        {"x": 0.30, "y": 0.62, "z": 0.0},
        {"x": 0.26, "y": 0.73, "z": 0.0},
    ]
    left_turn_frame = _fake_decoded_frame(
        left_turn_points,
        nearest_ahead={"x": 0.00, "y": 0.24, "z": 0.0},
        frame_number=2,
    )
    left_turn_state = build_robot_nav_state(left_turn_frame)
    assert left_turn_state["command"] == "LEFT"

    stop_points = [
        {"x": -0.02, "y": 0.22, "z": 0.0},
        {"x": 0.04, "y": 0.24, "z": 0.0},
        {"x": -0.16, "y": 0.31, "z": 0.0},
        {"x": -0.22, "y": 0.45, "z": 0.0},
        {"x": -0.28, "y": 0.58, "z": 0.0},
        {"x": -0.34, "y": 0.70, "z": 0.0},
        {"x": 0.18, "y": 0.33, "z": 0.0},
        {"x": 0.24, "y": 0.46, "z": 0.0},
        {"x": 0.31, "y": 0.59, "z": 0.0},
        {"x": 0.38, "y": 0.71, "z": 0.0},
    ]
    stop_frame = _fake_decoded_frame(
        stop_points,
        nearest_ahead={"x": 0.00, "y": 0.22, "z": 0.0},
        frame_number=3,
    )
    stop_state = build_robot_nav_state(stop_frame)
    assert stop_state["command"] == "STOP"

    smoother = CommandVoteSmoother(window_size=5)
    seq = [
        {"decision": {"command": "LEFT", "reason": "raw left"}, "zones": {"emergency_stop": False}},
        {"decision": {"command": "RIGHT", "reason": "raw right"}, "zones": {"emergency_stop": False}},
        {"decision": {"command": "LEFT", "reason": "raw left"}, "zones": {"emergency_stop": False}},
        {"decision": {"command": "LEFT", "reason": "raw left"}, "zones": {"emergency_stop": False}},
        {"decision": {"command": "RIGHT", "reason": "raw right"}, "zones": {"emergency_stop": False}},
    ]
    smoothed = None
    for sample in seq:
        smoothed = smoother.update(sample)
    assert smoothed["command"] == "LEFT"

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
