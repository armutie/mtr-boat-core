import argparse
import math
import time

import pygame

from mmwave_uart import build_navigation_summary
from mmwave_uart import merge_nav_config
from robot_nav import build_robot_nav_state
from robot_nav import CommandVoteSmoother
from robot_nav import DEFAULT_ROBOT_NAV_CONFIG
from robot_nav import merge_decision_config
from robot_nav import RobotRadarNavigator


BACKGROUND = (11, 16, 24)
PANEL = (18, 26, 38)
PANEL_ALT = (24, 34, 48)
GRID_MAJOR = (60, 84, 112)
GRID_MINOR = (34, 48, 68)
TEXT = (230, 236, 244)
MUTED = (141, 159, 181)
RAW_POINT = (103, 119, 141)
FILTERED_POINT = (83, 223, 221)
NEAREST_POINT = (255, 245, 157)
ROBOT_COLOR = (248, 113, 113)
FRONT_BOX = (251, 113, 133)
LEFT_BOX = (96, 165, 250)
RIGHT_BOX = (250, 204, 21)

COMMAND_COLORS = {
    "FORWARD": (52, 211, 153),
    "LEFT": (96, 165, 250),
    "RIGHT": (250, 204, 21),
    "STOP": (248, 113, 113),
}


def clamp(value, low, high):
    return max(low, min(high, value))


def make_point(x, y, snr=240, z=0.0):
    return {
        "x": x,
        "y": y,
        "z": z,
        "doppler": 0.0,
        "snr_raw": snr,
        "noise_raw": 900,
    }


class DemoRadarSource:
    def __init__(self, nav_config=None, decision_config=None):
        self.nav_config = merge_nav_config(nav_config or DEFAULT_ROBOT_NAV_CONFIG)
        self.decision_config = merge_decision_config(decision_config)
        self.command_smoother = CommandVoteSmoother(window_size=5)
        self.frame_number = 0
        self.started_at = None

    def start(self):
        self.started_at = time.monotonic()
        return self

    def close(self):
        return None

    def _build_points(self, t):
        points = []
        sway = 0.03 * math.sin(t * 0.9)

        for idx in range(6):
            y = 0.30 + idx * 0.14
            points.append(make_point(-0.62 + sway, y, snr=225))
            points.append(make_point(0.62 + sway, y, snr=225))

        phase = int(t / 4.0) % 3

        if phase == 0:
            for idx in range(4):
                points.append(make_point(-0.35 + 0.03 * idx, 0.45 + 0.10 * idx, snr=235))
                points.append(make_point(0.38 - 0.02 * idx, 0.40 + 0.11 * idx, snr=235))
        elif phase == 1:
            for row in range(4):
                points.append(make_point(0.02, 0.28 + row * 0.07, snr=255))
                points.append(make_point(0.12, 0.32 + row * 0.08, snr=255))
            for idx in range(5):
                points.append(make_point(0.26 + 0.03 * idx, 0.30 + 0.10 * idx, snr=245))
            points.append(make_point(-0.42, 0.62, snr=235))
            points.append(make_point(-0.32, 0.78, snr=235))
        else:
            for row in range(4):
                points.append(make_point(-0.04, 0.22 + row * 0.06, snr=260))
                points.append(make_point(0.05, 0.25 + row * 0.06, snr=260))
            for idx in range(5):
                points.append(make_point(-0.32 - 0.04 * idx, 0.28 + idx * 0.10, snr=245))
                points.append(make_point(0.28 + 0.04 * idx, 0.30 + idx * 0.10, snr=245))

        return points

    def read(self):
        if self.started_at is None:
            raise RuntimeError("DemoRadarSource.start() must be called before read()")

        self.frame_number += 1
        t = time.monotonic() - self.started_at
        combined_points = self._build_points(t)
        navigation = build_navigation_summary(combined_points, nav_config=self.nav_config)
        decoded_frame = {
            "header": {"frame_number": self.frame_number},
            "combined_points": combined_points,
            "navigation": navigation,
        }
        state = build_robot_nav_state(
            decoded_frame,
            decision_config=self.decision_config,
        )
        return self.command_smoother.update(state)


class LiveRadarSource:
    def __init__(self, data_port, cfg_port=None, cfg_file=None, nav_config=None, decision_config=None):
        self.navigator = RobotRadarNavigator(
            data_port=data_port,
            cfg_port=cfg_port,
            cfg_file=cfg_file,
            nav_config=nav_config or DEFAULT_ROBOT_NAV_CONFIG,
            decision_config=decision_config,
        )
        self.nav_config = self.navigator.nav_config
        self.decision_config = self.navigator.decision_config

    def start(self):
        self.navigator.start()
        return self

    def close(self):
        self.navigator.close()

    def read(self):
        return self.navigator.read()


def world_to_screen(plot_rect, x, y, x_span, y_span):
    center_x = plot_rect.left + plot_rect.width / 2.0
    px = center_x + (x / x_span) * (plot_rect.width / 2.0)
    py = plot_rect.bottom - (y / y_span) * plot_rect.height
    return int(px), int(py)


def wrap_text(text, font, max_width):
    words = text.split()
    lines = []
    current = ""

    for word in words:
        candidate = word if not current else f"{current} {word}"
        if font.size(candidate)[0] <= max_width:
            current = candidate
        else:
            if current:
                lines.append(current)
            current = word

    if current:
        lines.append(current)
    return lines


def draw_text(surface, font, text, color, position):
    surface.blit(font.render(text, True, color), position)


def draw_alpha_rect(surface, color, rect, alpha):
    overlay = pygame.Surface((rect.width, rect.height), pygame.SRCALPHA)
    overlay.fill((*color, alpha))
    surface.blit(overlay, rect.topleft)


def draw_grid(surface, plot_rect, x_span, y_span, label_font):
    draw_alpha_rect(surface, PANEL, plot_rect, 255)
    pygame.draw.rect(surface, GRID_MAJOR, plot_rect, width=1, border_radius=18)

    x_steps = max(1, int(x_span / 0.2))
    for step in range(-x_steps, x_steps + 1):
        x = step * 0.2
        sx, _ = world_to_screen(plot_rect, x, 0.0, x_span, y_span)
        color = GRID_MAJOR if abs(x) < 1e-6 else GRID_MINOR
        pygame.draw.line(surface, color, (sx, plot_rect.top), (sx, plot_rect.bottom), 1)
        if x != 0:
            draw_text(surface, label_font, f"{x:.1f}", MUTED, (sx - 10, plot_rect.bottom + 8))

    max_steps = int(y_span / 0.2)
    for step in range(max_steps + 1):
        y = step * 0.2
        _, sy = world_to_screen(plot_rect, 0.0, y, x_span, y_span)
        color = GRID_MAJOR if step % 5 == 0 else GRID_MINOR
        pygame.draw.line(surface, color, (plot_rect.left, sy), (plot_rect.right, sy), 1)
        if step > 0:
            draw_text(surface, label_font, f"{y:.1f}m", MUTED, (plot_rect.left - 48, sy - 8))


def draw_zone(surface, plot_rect, zone, color, label, x_span, y_span, active=False):
    top_left = world_to_screen(plot_rect, zone["x_min"], zone["y_max"], x_span, y_span)
    bottom_right = world_to_screen(plot_rect, zone["x_max"], zone["y_min"], x_span, y_span)
    rect = pygame.Rect(
        min(top_left[0], bottom_right[0]),
        min(top_left[1], bottom_right[1]),
        abs(bottom_right[0] - top_left[0]),
        abs(bottom_right[1] - top_left[1]),
    )
    draw_alpha_rect(surface, color, rect, 44 if active else 22)
    pygame.draw.rect(surface, color, rect, width=2, border_radius=10)
    draw_text(surface, pygame.font.SysFont("consolas", 18), label, color, (rect.left + 8, rect.top + 6))


def draw_robot(surface, plot_rect, x_span, y_span, command_color):
    robot_x, robot_y = world_to_screen(plot_rect, 0.0, 0.0, x_span, y_span)
    body = [
        (robot_x, robot_y - 18),
        (robot_x - 14, robot_y + 12),
        (robot_x + 14, robot_y + 12),
    ]
    pygame.draw.polygon(surface, command_color, body)
    pygame.draw.circle(surface, BACKGROUND, (robot_x, robot_y + 4), 5)


def draw_points(surface, plot_rect, raw_points, filtered_points, nearest_point, x_span, y_span):
    filtered_lookup = {id(point) for point in filtered_points}

    for point in raw_points:
        sx, sy = world_to_screen(plot_rect, point["x"], point["y"], x_span, y_span)
        radius = 4 if id(point) in filtered_lookup else 3
        color = FILTERED_POINT if id(point) in filtered_lookup else RAW_POINT
        pygame.draw.circle(surface, color, (sx, sy), radius)

    if nearest_point:
        sx, sy = world_to_screen(plot_rect, nearest_point["x"], nearest_point["y"], x_span, y_span)
        pygame.draw.circle(surface, NEAREST_POINT, (sx, sy), 9, width=2)
        pygame.draw.circle(surface, NEAREST_POINT, (sx, sy), 3)


def draw_command_badge(surface, rect, command):
    color = COMMAND_COLORS.get(command, TEXT)
    badge = pygame.Rect(rect.left, rect.top, 168, 56)
    draw_alpha_rect(surface, color, badge, 48)
    pygame.draw.rect(surface, color, badge, width=2, border_radius=14)
    font = pygame.font.SysFont("bahnschrift", 30, bold=True)
    label = font.render(command, True, color)
    surface.blit(label, (badge.left + 18, badge.top + 11))


def draw_bar(surface, rect, label, value, color, max_value):
    pygame.draw.rect(surface, PANEL_ALT, rect, border_radius=10)
    fill_width = 0 if max_value <= 0 else int(rect.width * (value / max_value))
    fill_rect = pygame.Rect(rect.left, rect.top, fill_width, rect.height)
    pygame.draw.rect(surface, color, fill_rect, border_radius=10)
    font = pygame.font.SysFont("consolas", 18)
    draw_text(surface, font, f"{label}: {value}", TEXT, (rect.left + 10, rect.top + 6))


def render_state(surface, state, nav_config, decision_config, frame_age_s):
    width, height = surface.get_size()
    plot_rect = pygame.Rect(84, 54, int(width * 0.60), height - 120)
    side_rect = pygame.Rect(plot_rect.right + 28, 54, width - plot_rect.right - 82, height - 120)

    x_span = max(
        abs(nav_config["filter_lateral_limit"]) + 0.25,
        abs(decision_config["left_box"]["x_min"]) + 0.15,
        abs(decision_config["right_box"]["x_max"]) + 0.15,
    )
    y_span = max(nav_config["filter_max_y"] + 0.15, decision_config["front_box"]["y_max"] + 0.25)

    title_font = pygame.font.SysFont("bahnschrift", 32, bold=True)
    section_font = pygame.font.SysFont("bahnschrift", 24, bold=True)
    body_font = pygame.font.SysFont("consolas", 20)
    small_font = pygame.font.SysFont("consolas", 16)

    surface.fill(BACKGROUND)
    draw_grid(surface, plot_rect, x_span, y_span, small_font)

    zones = state["zones"]
    raw_points = state["raw_frame"]["combined_points"]
    filtered_points = state["raw_frame"]["navigation"]["filtered_points"]
    nearest_point = state["nearest_ahead"]
    command = state["command"]
    command_color = COMMAND_COLORS.get(command, TEXT)

    draw_zone(surface, plot_rect, decision_config["front_box"], FRONT_BOX, "FRONT", x_span, y_span, active=zones["front_blocked"])
    draw_zone(surface, plot_rect, decision_config["left_box"], LEFT_BOX, "LEFT", x_span, y_span, active=zones["left_blocked"])
    draw_zone(surface, plot_rect, decision_config["right_box"], RIGHT_BOX, "RIGHT", x_span, y_span, active=zones["right_blocked"])
    draw_points(surface, plot_rect, raw_points, filtered_points, nearest_point, x_span, y_span)
    draw_robot(surface, plot_rect, x_span, y_span, command_color)

    draw_text(surface, title_font, "Radar Field View", TEXT, (plot_rect.left, 14))
    draw_text(
        surface,
        small_font,
        f"filtered points are cyan; raw points are gray; age={frame_age_s:.2f}s",
        MUTED,
        (plot_rect.left, plot_rect.bottom + 40),
    )

    draw_alpha_rect(surface, PANEL, side_rect, 255)
    pygame.draw.rect(surface, GRID_MAJOR, side_rect, width=1, border_radius=18)
    draw_command_badge(surface, pygame.Rect(side_rect.left + 22, side_rect.top + 20, 0, 0), command)

    draw_text(surface, section_font, f"Frame {state['frame_number']}", TEXT, (side_rect.left + 22, side_rect.top + 96))
    draw_text(
        surface,
        body_font,
        f"Nearest ahead: {state['nearest_ahead_distance']:.2f} m" if state["nearest_ahead_distance"] is not None else "Nearest ahead: none",
        TEXT,
        (side_rect.left + 22, side_rect.top + 136),
    )
    draw_text(
        surface,
        body_font,
        f"Filtered points: {state['points_filtered']} / {state['points_total']}",
        TEXT,
        (side_rect.left + 22, side_rect.top + 168),
    )
    draw_text(
        surface,
        body_font,
        f"Turn hint: {state['turn_hint']}",
        MUTED,
        (side_rect.left + 22, side_rect.top + 200),
    )
    draw_text(
        surface,
        body_font,
        f"Raw command: {state.get('raw_command', state['command'])}",
        MUTED,
        (side_rect.left + 22, side_rect.top + 224),
    )

    bar_left = side_rect.left + 22
    bar_width = side_rect.width - 44
    max_bar = max(6, zones["front_count"], zones["left_count"], zones["right_count"])
    draw_bar(surface, pygame.Rect(bar_left, side_rect.top + 248, bar_width, 26), "Front", zones["front_count"], FRONT_BOX, max_bar)
    draw_bar(surface, pygame.Rect(bar_left, side_rect.top + 282, bar_width, 26), "Left", zones["left_count"], LEFT_BOX, max_bar)
    draw_bar(surface, pygame.Rect(bar_left, side_rect.top + 316, bar_width, 26), "Right", zones["right_count"], RIGHT_BOX, max_bar)

    draw_text(surface, section_font, "Decision", TEXT, (side_rect.left + 22, side_rect.top + 356))
    reason_lines = wrap_text(state["reason"], body_font, side_rect.width - 44)
    for idx, line in enumerate(reason_lines[:4]):
        draw_text(surface, body_font, line, TEXT if idx == 0 else MUTED, (side_rect.left + 22, side_rect.top + 392 + idx * 24))

    vote = state.get("command_vote", {})
    counts = vote.get("counts", {})
    draw_text(
        surface,
        small_font,
        "vote counts: "
        f"F={counts.get('FORWARD', 0)} "
        f"L={counts.get('LEFT', 0)} "
        f"R={counts.get('RIGHT', 0)} "
        f"S={counts.get('STOP', 0)}",
        MUTED,
        (side_rect.left + 22, side_rect.top + 500),
    )

    draw_text(surface, section_font, "Thresholds", TEXT, (side_rect.left + 22, side_rect.top + 528))
    draw_text(
        surface,
        small_font,
        f"front_min={decision_config['front_min_points']}  side_min={decision_config['side_min_points']}",
        MUTED,
        (side_rect.left + 22, side_rect.top + 564),
    )
    draw_text(
        surface,
        small_font,
        f"stop_y={decision_config['stop_distance_y']:.2f}  emergency_y={decision_config['emergency_distance_y']:.2f}",
        MUTED,
        (side_rect.left + 22, side_rect.top + 586),
    )
    draw_text(
        surface,
        small_font,
        f"nav snr>={nav_config['filter_min_snr_raw']}  vote_window={vote.get('window_size', 0)}",
        MUTED,
        (side_rect.left + 22, side_rect.top + 608),
    )
    draw_text(surface, small_font, "Esc closes. Demo mode cycles scenarios every 4 seconds.", MUTED, (side_rect.left + 22, side_rect.bottom - 34))


def run_viewer(source):
    pygame.init()
    pygame.display.set_caption("TI mmWave Radar Viewer")
    screen = pygame.display.set_mode((1280, 720), pygame.RESIZABLE)
    clock = pygame.time.Clock()

    source.start()
    last_state = None
    last_state_time = time.monotonic()
    last_error = None

    try:
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    running = False

            try:
                state = source.read()
                if state is not None:
                    last_state = state
                    last_state_time = time.monotonic()
                    last_error = None
            except Exception as exc:
                last_error = str(exc)

            screen.fill(BACKGROUND)
            if last_state is None:
                font = pygame.font.SysFont("bahnschrift", 34, bold=True)
                small = pygame.font.SysFont("consolas", 20)
                draw_text(screen, font, "Waiting for radar frames...", TEXT, (54, 64))
                draw_text(screen, small, "Check COM ports and config, or run with --demo.", MUTED, (54, 112))
                if last_error:
                    draw_text(screen, small, f"Reader error: {last_error}", FRONT_BOX, (54, 146))
            else:
                render_state(
                    screen,
                    last_state,
                    nav_config=source.nav_config,
                    decision_config=source.decision_config,
                    frame_age_s=time.monotonic() - last_state_time,
                )
                if last_error:
                    warn_font = pygame.font.SysFont("consolas", 16)
                    draw_text(screen, warn_font, f"reader error: {last_error}", FRONT_BOX, (84, 18))

            pygame.display.flip()
            clock.tick(30)
    finally:
        source.close()
        pygame.quit()


def main():
    ap = argparse.ArgumentParser(description="Live pygame viewer for TI mmWave radar data")
    ap.add_argument("--cfg-port", help="COM port for CFG / CLI UART, e.g. COM6")
    ap.add_argument("--cfg-file", help="Path to .cfg file")
    ap.add_argument("--data-port", help="COM port for DATA UART, e.g. COM5")
    ap.add_argument("--demo", action="store_true", help="Run synthetic demo mode instead of opening the radar")
    args = ap.parse_args()

    if args.demo:
        source = DemoRadarSource()
    else:
        if not args.data_port:
            ap.error("--data-port is required unless --demo is used")
        source = LiveRadarSource(
            data_port=args.data_port,
            cfg_port=args.cfg_port,
            cfg_file=args.cfg_file,
        )

    run_viewer(source)


if __name__ == "__main__":
    main()
