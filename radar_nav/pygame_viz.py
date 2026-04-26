from __future__ import annotations

import math

from .config import NavConfig
from .models import NavOutput


class RadarPygameViz:
    def __init__(self, cfg: NavConfig, width: int = 1280, height: int = 720):
        try:
            import pygame
        except ImportError as exc:
            raise RuntimeError("pygame is required for visualization. Install it with: python -m pip install pygame") from exc

        self.pygame = pygame
        self.cfg = cfg
        self.width = width
        self.height = height
        self.show_singletons = True
        self.show_raw = True
        self.paused = False
        self.logging_enabled = False
        self.font = None
        self.big_font = None
        self.clock = None
        self.screen = None
        self.last_fps = 0.0
        self.throttle_history: list[float] = []
        self.max_history = 180

        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("AWR1843 Radar Navigation")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("consolas", 18)
        self.small_font = pygame.font.SysFont("consolas", 14)
        self.big_font = pygame.font.SysFont("consolas", 42, bold=True)

    def reset_clock(self) -> None:
        self.clock.tick()

    def handle_events(self) -> tuple[bool, list[str]]:
        pygame = self.pygame
        actions = []
        running = True
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False
                elif event.key == pygame.K_s:
                    self.show_singletons = not self.show_singletons
                elif event.key == pygame.K_g:
                    self.show_raw = not self.show_raw
                elif event.key == pygame.K_p:
                    self.paused = not self.paused
                elif event.key == pygame.K_l:
                    actions.append("toggle_logging")
                elif event.key == pygame.K_r:
                    actions.append("reset")
                elif event.key == pygame.K_RIGHT:
                    self.cfg.alpha += 0.02
                    self.cfg.clamp_values()
                    actions.append("config_changed")
                elif event.key == pygame.K_LEFT:
                    self.cfg.alpha -= 0.02
                    self.cfg.clamp_values()
                    actions.append("config_changed")
                elif event.key == pygame.K_RIGHTBRACKET:
                    self.cfg.cluster_eps_m += 0.05
                    self.cfg.clamp_values()
                    actions.append("config_changed")
                elif event.key == pygame.K_LEFTBRACKET:
                    self.cfg.cluster_eps_m -= 0.05
                    self.cfg.clamp_values()
                    actions.append("config_changed")
                elif event.key in (pygame.K_EQUALS, pygame.K_PLUS):
                    if self.cfg.min_snr_raw is None:
                        self.cfg.min_snr_raw = 0
                    self.cfg.min_snr_raw += 10
                    self.cfg.clamp_values()
                    actions.append("config_changed")
                elif event.key == pygame.K_MINUS:
                    if self.cfg.min_snr_raw is not None:
                        self.cfg.min_snr_raw -= 10
                    self.cfg.clamp_values()
                    actions.append("config_changed")
                elif event.key == pygame.K_UP:
                    self.cfg.front_on_thresh += 0.05
                    self.cfg.front_off_thresh += 0.05
                    self.cfg.clamp_values()
                    actions.append("config_changed")
                elif event.key == pygame.K_DOWN:
                    self.cfg.front_on_thresh -= 0.05
                    self.cfg.front_off_thresh -= 0.05
                    self.cfg.clamp_values()
                    actions.append("config_changed")
                elif event.key == pygame.K_SPACE:
                    actions.append("step")
        return running, actions

    def _plot_rect(self):
        margin = 28
        panel_w = 350
        bottom_h = 92
        return self.pygame.Rect(margin, margin, self.width - panel_w - margin * 2, self.height - bottom_h - margin * 2)

    def _world_to_screen(self, x: float, y: float, rect):
        cfg = self.cfg
        sx = rect.left + (x - cfg.viz_x_min) / (cfg.viz_x_max - cfg.viz_x_min) * rect.width
        sy = rect.bottom - (y - cfg.viz_y_min) / (cfg.viz_y_max - cfg.viz_y_min) * rect.height
        return int(sx), int(sy)

    def _draw_text(self, text: str, pos, color=(230, 236, 241), font=None) -> None:
        surface = (font or self.font).render(text, True, color)
        self.screen.blit(surface, pos)

    def _draw_bar(self, label: str, value: float, x: int, y: int, color) -> None:
        pygame = self.pygame
        self._draw_text(f"{label:<6} {value:0.2f}", (x, y))
        bar = pygame.Rect(x, y + 24, 250, 14)
        pygame.draw.rect(self.screen, (38, 44, 50), bar, border_radius=3)
        fill = pygame.Rect(bar.left, bar.top, int(bar.width * max(0.0, min(value, 1.0))), bar.height)
        pygame.draw.rect(self.screen, color, fill, border_radius=3)

    def _draw_signed_bar(self, label: str, value: float, x: int, y: int, color) -> None:
        pygame = self.pygame
        value = max(-1.0, min(1.0, value))
        self._draw_text(f"{label:<6} {value:+0.2f}", (x, y))
        bar = pygame.Rect(x, y + 24, 250, 14)
        center = bar.left + bar.width // 2
        pygame.draw.rect(self.screen, (38, 44, 50), bar, border_radius=3)
        pygame.draw.line(self.screen, (120, 132, 144), (center, bar.top - 2), (center, bar.bottom + 2), 1)
        if value < 0:
            fill = pygame.Rect(center + int(value * bar.width / 2), bar.top, int(abs(value) * bar.width / 2), bar.height)
        else:
            fill = pygame.Rect(center, bar.top, int(value * bar.width / 2), bar.height)
        pygame.draw.rect(self.screen, color, fill, border_radius=3)

    def _draw_yoke(self, center, radius: int, steering: float) -> None:
        pygame = self.pygame
        steering = max(-1.0, min(1.0, steering))
        angle = steering * math.radians(80)
        cx, cy = center
        pygame.draw.circle(self.screen, (38, 44, 50), center, radius + 8, 2)
        pygame.draw.circle(self.screen, (76, 88, 102), center, radius, 3)

        def rotated_point(px: float, py: float):
            dx = px - cx
            dy = py - cy
            ca = math.cos(angle)
            sa = math.sin(angle)
            return int(cx + dx * ca - dy * sa), int(cy + dx * sa + dy * ca)

        left = rotated_point(cx - radius * 0.78, cy)
        right = rotated_point(cx + radius * 0.78, cy)
        top = rotated_point(cx, cy - radius * 0.58)
        bottom = rotated_point(cx, cy + radius * 0.50)
        pygame.draw.line(self.screen, (230, 236, 241), left, right, 5)
        pygame.draw.line(self.screen, (230, 236, 241), top, bottom, 4)
        pygame.draw.circle(self.screen, (80, 170, 220), center, 8)
        self._draw_text("STEER", (cx - 32, cy + radius + 13), (152, 164, 175), self.small_font)

    def _draw_velocity_chart(self, rect) -> None:
        pygame = self.pygame
        pygame.draw.rect(self.screen, (18, 24, 30), rect)
        pygame.draw.rect(self.screen, (92, 105, 118), rect, 1)
        self._draw_text("THROTTLE HISTORY", (rect.left + 8, rect.top + 6), (152, 164, 175), self.small_font)
        for i in range(1, 4):
            y = rect.top + int(rect.height * i / 4)
            pygame.draw.line(self.screen, (34, 43, 52), (rect.left, y), (rect.right, y))
        if len(self.throttle_history) < 2:
            return
        points = []
        history = self.throttle_history[-self.max_history:]
        for i, value in enumerate(history):
            x = rect.left + int(i / max(len(history) - 1, 1) * (rect.width - 1))
            y = rect.bottom - 8 - int(max(0.0, min(1.0, value)) * (rect.height - 28))
            points.append((x, y))
        pygame.draw.lines(self.screen, (79, 211, 141), False, points, 2)

    def draw(self, output: NavOutput | None) -> None:
        pygame = self.pygame
        if output is not None:
            self.throttle_history.append(output.throttle)
            if len(self.throttle_history) > self.max_history:
                del self.throttle_history[: len(self.throttle_history) - self.max_history]

        self.screen.fill((13, 17, 22))
        plot = self._plot_rect()
        pygame.draw.rect(self.screen, (18, 24, 30), plot)
        pygame.draw.rect(self.screen, (92, 105, 118), plot, 1)

        cfg = self.cfg
        for x in [cfg.viz_x_min, -1.0, -0.5, 0.0, 0.5, 1.0, cfg.viz_x_max]:
            sx, _ = self._world_to_screen(x, cfg.viz_y_min, plot)
            pygame.draw.line(self.screen, (34, 43, 52), (sx, plot.top), (sx, plot.bottom))
        for y in [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]:
            _, sy = self._world_to_screen(0.0, y, plot)
            pygame.draw.line(self.screen, (34, 43, 52), (plot.left, sy), (plot.right, sy))
            self._draw_text(f"{y:.1f}m", (plot.left + 6, sy - 18), (126, 140, 153), self.small_font)

        for boundary in (-cfg.front_half_width, cfg.front_half_width):
            sx, _ = self._world_to_screen(boundary, 0.0, plot)
            pygame.draw.line(self.screen, (168, 115, 59), (sx, plot.top), (sx, plot.bottom), 2)

        origin = self._world_to_screen(0.0, 0.0, plot)
        pygame.draw.polygon(
            self.screen,
            (80, 170, 220),
            [(origin[0], origin[1] - 14), (origin[0] - 13, origin[1] + 12), (origin[0] + 13, origin[1] + 12)],
        )

        if output is not None:
            if self.show_raw:
                for point in output.raw_points:
                    sx, sy = self._world_to_screen(float(point.get("x", 0.0)), float(point.get("y", 0.0)), plot)
                    if plot.collidepoint(sx, sy):
                        pygame.draw.circle(self.screen, (70, 78, 88), (sx, sy), 3)

            for point in output.filtered_points:
                sx, sy = self._world_to_screen(float(point.get("x", 0.0)), float(point.get("y", 0.0)), plot)
                if plot.collidepoint(sx, sy):
                    pygame.draw.circle(self.screen, (232, 238, 244), (sx, sy), 4)

            for cluster in output.clusters:
                if cluster.is_singleton and not self.show_singletons:
                    continue
                sx, sy = self._world_to_screen(cluster.cx, cluster.cy, plot)
                if not plot.collidepoint(sx, sy):
                    continue
                color = {
                    "front": (232, 86, 86),
                    "left": (224, 181, 76),
                    "right": (224, 181, 76),
                    "unknown": (150, 160, 170),
                }[cluster.zone]
                radius = 8 + int(cluster.confidence * 16)
                width = 2 if cluster.is_singleton else 0
                pygame.draw.circle(self.screen, color, (sx, sy), radius, width)
                pygame.draw.circle(self.screen, (255, 255, 255), (sx, sy), 3)

        panel_x = plot.right + 30
        self._draw_text("RADAR NAV", (panel_x, plot.top), (245, 248, 250))
        if output is not None:
            rows = [
                f"Frame: {output.frame_number}",
                f"Raw points: {len(output.raw_points)}",
                f"Filtered: {len(output.filtered_points)}",
                f"Clusters: {len(output.clusters)}",
                f"Blocked: {str(output.front_blocked).upper()}",
                f"Throttle: {output.throttle:0.2f} -> {output.target_throttle:0.2f}",
                f"Steering: {output.steering:+0.2f} -> {output.target_steering:+0.2f}",
                f"FPS: {self.last_fps:0.1f}",
            ]
        else:
            rows = ["Waiting for frames..."]
        y = plot.top + 38
        for row in rows:
            self._draw_text(row, (panel_x, y), (205, 214, 222))
            y += 24

        y += 14
        if output is not None:
            self._draw_bar("LEFT", output.left_score, panel_x, y, (224, 181, 76))
            self._draw_bar("FRONT", output.front_score, panel_x, y + 58, (232, 86, 86))
            self._draw_bar("RIGHT", output.right_score, panel_x, y + 116, (224, 181, 76))
            self._draw_bar("THROT", output.throttle, panel_x, y + 174, (79, 211, 141))
            self._draw_signed_bar("STEER", output.steering, panel_x, y + 232, (80, 170, 220))

        if output is not None:
            self._draw_yoke((panel_x + 70, plot.bottom - 95), 42, output.steering)
            self._draw_velocity_chart(pygame.Rect(panel_x + 135, plot.bottom - 145, 190, 92))

        y = plot.bottom - 42
        self._draw_text(f"alpha {cfg.alpha:.2f}   eps {cfg.cluster_eps_m:.2f}m", (panel_x, y), (152, 164, 175), self.small_font)
        self._draw_text(f"snr {cfg.min_snr_raw}   block {cfg.front_on_thresh:.2f}/{cfg.front_off_thresh:.2f}", (panel_x, y + 20), (152, 164, 175), self.small_font)

        command = output.command if output else "waiting"
        command_color = {
            "forward": (53, 151, 92),
            "turn_left": (217, 158, 59),
            "turn_right": (217, 158, 59),
            "stop": (205, 68, 68),
            "waiting": (78, 91, 106),
        }[command]
        status = pygame.Rect(28, self.height - 76, self.width - 56, 48)
        pygame.draw.rect(self.screen, command_color, status, border_radius=5)
        self._draw_text(command.upper(), (status.left + 18, status.top + 1), (255, 255, 255), self.big_font)
        if output is not None:
            self._draw_text(output.reason[:85], (status.left + 300, status.top + 15), (255, 255, 255), self.font)

        pygame.display.flip()
        self.last_fps = self.clock.get_fps()
        self.clock.tick(60)

    def close(self) -> None:
        self.pygame.quit()
