import argparse
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from radar_nav import NavConfig, RadarNavPipeline
from radar_nav.sim import (
    BoatConfig,
    BoatState,
    ControllerState,
    ObstacleCourse,
    SimControl,
    SimMetrics,
    WaypointConfig,
    WaypointState,
    choose_candidate_control,
    compute_waypoint_control,
    generate_radar_points,
    reset_sim,
    update_boat,
    update_metrics,
)


class RadarBoatSimViz:
    def __init__(self, width: int = 1280, height: int = 720):
        try:
            import pygame
        except ImportError as exc:
            raise RuntimeError("pygame is required for simulation. Install it with: python -m pip install pygame") from exc

        self.pygame = pygame
        self.width = width
        self.height = height
        self.paused = False
        self.clock = None
        self.screen = None
        self.font = None
        self.small_font = None
        self.big_font = None
        self.last_fps = 0.0

        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Radar Navigation Boat Simulation")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("consolas", 18)
        self.small_font = pygame.font.SysFont("consolas", 14)
        self.big_font = pygame.font.SysFont("consolas", 28, bold=True)

    def handle_events(self) -> tuple[bool, list[str]]:
        pygame = self.pygame
        running = True
        actions = []
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False
                elif event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                elif event.key == pygame.K_r:
                    actions.append("reset")
                elif event.key == pygame.K_p:
                    actions.append("step")
        return running, actions

    def _draw_text(self, text: str, pos, color=(230, 236, 241), font=None) -> None:
        surface = (font or self.font).render(text, True, color)
        self.screen.blit(surface, pos)

    def _world_rect(self):
        pygame = self.pygame
        return pygame.Rect(28, 28, self.width - 458, self.height - 56)

    def _radar_rect(self):
        pygame = self.pygame
        return pygame.Rect(self.width - 400, 260, 360, 250)

    def _minimap_rect(self):
        pygame = self.pygame
        return pygame.Rect(46, 46, 180, 180)

    def _world_transform(self, course: ObstacleCourse, boat: BoatState, rect):
        _ = course
        view_w = 5.0
        view_h = 8.5
        min_x = boat.x - view_w / 2.0
        max_x = boat.x + view_w / 2.0
        min_y = boat.y - 1.45
        max_y = min_y + view_h
        scale = min(rect.width / view_w, rect.height / view_h)

        def to_screen(x: float, y: float):
            sx = rect.left + int((x - min_x) * scale)
            sy = rect.bottom - int((y - min_y) * scale)
            return sx, sy

        return to_screen, scale

    def _draw_obstacles(self, course: ObstacleCourse, to_screen, scale: float) -> None:
        pygame = self.pygame
        from radar_nav.sim import CircleObstacle, RectObstacle

        arena_center = to_screen(0.0, 0.0)
        pygame.draw.circle(self.screen, (54, 73, 90), arena_center, max(1, int(course.arena_radius * scale)), 2)

        for obstacle in course.obstacles:
            if isinstance(obstacle, CircleObstacle):
                center = to_screen(obstacle.x, obstacle.y)
                pygame.draw.circle(self.screen, (202, 92, 84), center, max(3, int(obstacle.radius * scale)))
                pygame.draw.circle(self.screen, (255, 205, 195), center, max(3, int(obstacle.radius * scale)), 2)
            elif isinstance(obstacle, RectObstacle):
                left, top = to_screen(obstacle.x - obstacle.width / 2.0, obstacle.y + obstacle.height / 2.0)
                right, bottom = to_screen(obstacle.x + obstacle.width / 2.0, obstacle.y - obstacle.height / 2.0)
                rect = pygame.Rect(left, top, right - left, bottom - top)
                pygame.draw.rect(self.screen, (202, 92, 84), rect)
                pygame.draw.rect(self.screen, (255, 205, 195), rect, 2)

    def _draw_boat(self, boat: BoatState, boat_cfg: BoatConfig, waypoint: WaypointState, to_screen, scale: float) -> None:
        pygame = self.pygame
        if len(boat.trail) > 1:
            trail = [to_screen(x, y) for x, y in boat.trail]
            pygame.draw.lines(self.screen, (80, 150, 205), False, trail, 2)

        center = to_screen(boat.x, boat.y)
        waypoint_pos = to_screen(waypoint.x, waypoint.y)
        pygame.draw.line(self.screen, (82, 126, 170), center, waypoint_pos, 1)
        pygame.draw.circle(self.screen, (104, 211, 145), waypoint_pos, max(7, int(0.12 * scale)))
        pygame.draw.circle(self.screen, (225, 255, 235), waypoint_pos, max(12, int(0.45 * scale)), 2)

        heading = boat.heading
        forward = (math.sin(heading), math.cos(heading))
        right = (math.cos(heading), -math.sin(heading))
        length = 0.42 * scale
        width = 0.20 * scale
        nose = (center[0] + forward[0] * length, center[1] - forward[1] * length)
        left = (
            center[0] - forward[0] * length * 0.65 - right[0] * width,
            center[1] + forward[1] * length * 0.65 + right[1] * width,
        )
        right_pt = (
            center[0] - forward[0] * length * 0.65 + right[0] * width,
            center[1] + forward[1] * length * 0.65 - right[1] * width,
        )
        color = (238, 196, 90) if not boat.collided else (255, 82, 82)
        pygame.draw.circle(self.screen, (97, 112, 128), center, max(4, int(boat_cfg.collision_radius * scale)), 1)
        pygame.draw.polygon(self.screen, color, [nose, left, right_pt])
        pygame.draw.polygon(self.screen, (255, 245, 210), [nose, left, right_pt], 2)

    def _draw_minimap(self, course: ObstacleCourse, boat: BoatState, waypoint: WaypointState, rect) -> None:
        pygame = self.pygame
        from radar_nav.sim import CircleObstacle, RectObstacle

        pygame.draw.rect(self.screen, (14, 20, 26), rect, border_radius=4)
        pygame.draw.rect(self.screen, (94, 108, 122), rect, 1, border_radius=4)
        center = (rect.centerx, rect.centery)
        radius_px = int(min(rect.width, rect.height) * 0.43)
        pygame.draw.circle(self.screen, (40, 55, 69), center, radius_px)
        pygame.draw.circle(self.screen, (126, 146, 163), center, radius_px, 2)

        def to_map(x: float, y: float):
            sx = center[0] + int(x / course.arena_radius * radius_px)
            sy = center[1] - int(y / course.arena_radius * radius_px)
            return sx, sy

        for obstacle in course.obstacles:
            if isinstance(obstacle, CircleObstacle):
                pygame.draw.circle(
                    self.screen,
                    (202, 92, 84),
                    to_map(obstacle.x, obstacle.y),
                    max(2, int(obstacle.radius / course.arena_radius * radius_px)),
                )
            elif isinstance(obstacle, RectObstacle):
                obstacle_center = to_map(obstacle.x, obstacle.y)
                w = max(2, int(obstacle.width / course.arena_radius * radius_px))
                h = max(2, int(obstacle.height / course.arena_radius * radius_px))
                pygame.draw.rect(self.screen, (202, 92, 84), pygame.Rect(obstacle_center[0] - w // 2, obstacle_center[1] - h // 2, w, h))

        if len(boat.trail) > 1:
            trail = [to_map(x, y) for x, y in boat.trail]
            pygame.draw.lines(self.screen, (80, 150, 205), False, trail, 1)
        pygame.draw.circle(self.screen, (104, 211, 145), to_map(waypoint.x, waypoint.y), 5)
        boat_pos = to_map(boat.x, boat.y)
        pygame.draw.circle(self.screen, (238, 196, 90), boat_pos, 5)
        nose = (
            boat_pos[0] + int(math.sin(boat.heading) * 10),
            boat_pos[1] - int(math.cos(boat.heading) * 10),
        )
        pygame.draw.line(self.screen, (255, 245, 210), boat_pos, nose, 2)
        self._draw_text("ARENA", (rect.left + 8, rect.top + 6), (205, 214, 222), self.small_font)

    def _radar_to_screen(self, x: float, y: float, nav_cfg: NavConfig, rect):
        x_min = -nav_cfg.lateral_limit
        x_max = nav_cfg.lateral_limit
        y_min = 0.0
        y_max = nav_cfg.max_y or 2.5
        sx = rect.left + int((x - x_min) / (x_max - x_min) * rect.width)
        sy = rect.bottom - int((y - y_min) / (y_max - y_min) * rect.height)
        return sx, sy

    def _draw_radar(self, nav_cfg: NavConfig, output, rect) -> None:
        pygame = self.pygame
        pygame.draw.rect(self.screen, (18, 24, 30), rect)
        pygame.draw.rect(self.screen, (92, 105, 118), rect, 1)
        self._draw_text("BOAT-RELATIVE RADAR", (rect.left, rect.top - 24), (205, 214, 222), self.small_font)

        for x in (-nav_cfg.front_half_width, nav_cfg.front_half_width):
            sx, _ = self._radar_to_screen(x, 0.0, nav_cfg, rect)
            pygame.draw.line(self.screen, (168, 115, 59), (sx, rect.top), (sx, rect.bottom), 1)
        sx, sy = self._radar_to_screen(0.0, 0.0, nav_cfg, rect)
        pygame.draw.polygon(self.screen, (80, 170, 220), [(sx, sy - 12), (sx - 10, sy + 10), (sx + 10, sy + 10)])

        if output is None:
            return
        for point in output.raw_points:
            px, py = self._radar_to_screen(float(point["x"]), float(point["y"]), nav_cfg, rect)
            if rect.collidepoint(px, py):
                pygame.draw.circle(self.screen, (232, 238, 244), (px, py), 3)
        for cluster in output.clusters:
            px, py = self._radar_to_screen(cluster.cx, cluster.cy, nav_cfg, rect)
            if rect.collidepoint(px, py):
                color = {"front": (232, 86, 86), "left": (224, 181, 76), "right": (224, 181, 76), "unknown": (150, 160, 170)}[cluster.zone]
                pygame.draw.circle(self.screen, color, (px, py), 6 + int(cluster.confidence * 12), 2)

    def _draw_bar(self, label: str, value: float, x: int, y: int, color, width: int = 285) -> None:
        pygame = self.pygame
        self._draw_text(f"{label:<8} {value:0.2f}", (x, y), (230, 236, 241), self.small_font)
        rect = pygame.Rect(x, y + 18, width, 10)
        pygame.draw.rect(self.screen, (38, 44, 50), rect)
        fill = pygame.Rect(rect.left, rect.top, int(rect.width * max(0.0, min(value, 1.0))), rect.height)
        pygame.draw.rect(self.screen, color, fill)

    def _draw_signed_bar(self, label: str, value: float, x: int, y: int, color, width: int = 285) -> None:
        pygame = self.pygame
        value = max(-1.0, min(1.0, value))
        self._draw_text(f"{label:<8} {value:+0.2f}", (x, y), (230, 236, 241), self.small_font)
        rect = pygame.Rect(x, y + 18, width, 10)
        center = rect.left + rect.width // 2
        pygame.draw.rect(self.screen, (38, 44, 50), rect)
        pygame.draw.line(self.screen, (130, 140, 150), (center, rect.top - 2), (center, rect.bottom + 2))
        if value < 0.0:
            fill = pygame.Rect(center + int(value * rect.width / 2), rect.top, int(abs(value) * rect.width / 2), rect.height)
        else:
            fill = pygame.Rect(center, rect.top, int(value * rect.width / 2), rect.height)
        pygame.draw.rect(self.screen, color, fill)

    def draw(
        self,
        boat: BoatState,
        boat_cfg: BoatConfig,
        course: ObstacleCourse,
        nav_cfg: NavConfig,
        waypoint: WaypointState,
        output,
        control: SimControl | None,
        metrics: SimMetrics,
        sim_time: float,
    ) -> None:
        pygame = self.pygame
        self.screen.fill((12, 17, 22))
        world_rect = self._world_rect()
        pygame.draw.rect(self.screen, (18, 24, 30), world_rect)
        pygame.draw.rect(self.screen, (92, 105, 118), world_rect, 1)
        to_screen, scale = self._world_transform(course, boat, world_rect)
        self._draw_obstacles(course, to_screen, scale)
        self._draw_boat(boat, boat_cfg, waypoint, to_screen, scale)
        self._draw_minimap(course, boat, waypoint, self._minimap_rect())

        panel_x = self.width - 400
        self._draw_text("BOAT SIM", (panel_x, 28), (245, 248, 250), self.big_font)
        rows = [
            f"Time: {sim_time:0.1f}s   FPS: {self.last_fps:0.1f}",
            f"Speed: {boat.speed:0.2f} m/s   Yaw: {boat.yaw_rate:+0.2f}",
            f"Pos: x={boat.x:+0.2f} y={boat.y:+0.2f}",
            f"Waypoint: x={waypoint.x:+0.2f} y={waypoint.y:+0.2f}",
            f"WP dist: {(control.waypoint_distance if control else 0.0):0.2f} m",
            f"WP reached: {metrics.waypoints_reached}",
            f"Status: {'FAILED' if metrics.failed else 'RUNNING'}",
            f"Arena edge: {course.arena_clearance(boat.x, boat.y):0.2f} m",
            f"Collisions: {metrics.collisions}",
            f"Distance: {metrics.distance_traveled:0.2f} m",
            f"Avg speed: {metrics.average_speed:0.2f} m/s",
            f"Stuck time: {metrics.stuck_time:0.2f}s",
            f"Min clearance: {(metrics.min_obstacle_clearance or 0.0):+0.2f} m",
            f"Steer jitter: {metrics.steering_jitter:0.2f}",
        ]
        y = 70
        for row in rows:
            self._draw_text(row, (panel_x, y), (205, 214, 222), self.small_font)
            y += 21

        self._draw_radar(nav_cfg, output, self._radar_rect())

        y = 530
        if output is not None:
            final_throttle = control.throttle if control else output.throttle
            final_steering = control.steering if control else output.steering
            self._draw_bar("THROTTLE", final_throttle, panel_x, y, (79, 211, 141))
            self._draw_signed_bar("STEERING", final_steering, panel_x, y + 38, (80, 170, 220))
            self._draw_bar("LEFT", output.left_score, panel_x, y + 78, (224, 181, 76))
            self._draw_bar("FRONT", output.front_score, panel_x, y + 116, (232, 86, 86))
            self._draw_bar("RIGHT", output.right_score, panel_x, y + 154, (224, 181, 76))
            self._draw_text(f"{output.command.upper()}  blocked={output.front_blocked}", (panel_x, y + 198), (245, 248, 250), self.small_font)
            if control:
                clearance = "none" if control.min_predicted_clearance is None else f"{control.min_predicted_clearance:+0.2f}m"
                self._draw_text(
                    f"score {control.score:0.1f}   predicted clearance {clearance}",
                    (panel_x, y + 220),
                    (205, 214, 222),
                    self.small_font,
                )
        else:
            self._draw_text("Waiting for sim frame...", (panel_x, y), (205, 214, 222), self.small_font)

        if metrics.failed:
            banner = pygame.Rect(world_rect.left + 24, world_rect.top + 24, 430, 64)
            pygame.draw.rect(self.screen, (165, 42, 42), banner, border_radius=4)
            pygame.draw.rect(self.screen, (255, 205, 195), banner, 2, border_radius=4)
            reason = metrics.failure_reason.upper() if metrics.failure_reason else "RUN FAILED"
            self._draw_text(reason, (banner.left + 18, banner.top + 10), (255, 255, 255), self.big_font)
            self._draw_text("Press R to reset", (banner.left + 20, banner.top + 42), (255, 235, 225), self.small_font)

        self._draw_text("Q/ESC quit   R reset   Space pause   P next frame", (34, self.height - 24), (152, 164, 175), self.small_font)
        pygame.display.flip()
        self.last_fps = self.clock.get_fps()
        self.clock.tick(60)

    def close(self) -> None:
        self.pygame.quit()


def main() -> None:
    ap = argparse.ArgumentParser(description="2D boat simulation for continuous radar navigation controls")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--dt", type=float, default=1.0 / 30.0)
    ap.add_argument("--speed", type=float, default=1.0, help="Simulation speed multiplier")
    args = ap.parse_args()

    nav_cfg = NavConfig()
    nav_cfg.clamp_values()
    boat_cfg = BoatConfig()
    course = ObstacleCourse.default()
    waypoint_cfg = WaypointConfig()
    waypoint = WaypointState()
    waypoint.reset(BoatState(), waypoint_cfg, course)
    pipeline = RadarNavPipeline(nav_cfg)
    boat = BoatState()
    metrics = SimMetrics()
    controller_state = ControllerState()
    viz = RadarBoatSimViz(width=args.width, height=args.height)
    sim_time = 0.0
    last_output = None
    last_control = None

    print("[SIM] Keys: Q/ESC quit, R reset, Space pause, P next frame.")
    try:
        running = True
        while running:
            running, actions = viz.handle_events()
            if "reset" in actions:
                reset_sim(boat, metrics)
                course.reset()
                pipeline.reset()
                controller_state.reset()
                waypoint.reset(boat, waypoint_cfg, course)
                sim_time = 0.0
                last_output = None
                last_control = None
                print("[SIM] Reset.")

            step = "step" in actions
            should_step = (not viz.paused) or step
            if should_step and not metrics.failed:
                dt = max(args.dt, 0.001) * max(args.speed, 0.01)
                course.update_for_boat(boat.y)
                points = generate_radar_points(boat, course, nav_cfg)
                last_output = pipeline.process_points(points, now=sim_time)
                waypoint_control = compute_waypoint_control(boat, waypoint, waypoint_cfg)
                if waypoint_control.reached:
                    metrics.waypoints_reached += 1
                    waypoint.advance_from(boat, waypoint_cfg, course)
                    waypoint_control = compute_waypoint_control(boat, waypoint, waypoint_cfg)
                last_control = choose_candidate_control(
                    boat,
                    boat_cfg,
                    waypoint,
                    waypoint_cfg,
                    last_output,
                    course,
                    controller_state,
                )
                controller_state.apply(last_control)
                update_boat(boat, boat_cfg, last_control.throttle, last_control.steering, dt)
                update_metrics(metrics, boat, boat_cfg, course, last_output, dt, control_steering=last_control.steering)
                sim_time += dt

            viz.draw(boat, boat_cfg, course, nav_cfg, waypoint, last_output, last_control, metrics, sim_time)
    finally:
        viz.close()


if __name__ == "__main__":
    main()
