from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Iterable, Protocol

from .config import NavConfig
from .models import NavOutput


def clamp(value: float, lo: float, hi: float) -> float:
    return min(max(value, lo), hi)


@dataclass
class BoatConfig:
    max_thrust: float = 1.15
    linear_drag: float = 0.22
    quadratic_drag: float = 0.18
    rudder_strength: float = 1.55
    yaw_damping: float = 1.65
    max_speed: float = 2.0
    collision_radius: float = 0.18
    trail_limit: int = 360
    stuck_speed: float = 0.05


@dataclass
class BoatState:
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0
    speed: float = 0.0
    yaw_rate: float = 0.0
    trail: list[tuple[float, float]] = field(default_factory=list)
    collided: bool = False
    failed: bool = False

    def reset(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.speed = 0.0
        self.yaw_rate = 0.0
        self.trail.clear()
        self.collided = False
        self.failed = False


@dataclass
class SimMetrics:
    distance_traveled: float = 0.0
    collisions: int = 0
    stuck_time: float = 0.0
    elapsed_time: float = 0.0
    steering_jitter: float = 0.0
    min_obstacle_clearance: float | None = None
    failed: bool = False
    failure_reason: str = ""
    waypoints_reached: int = 0
    _last_position: tuple[float, float] | None = None
    _was_colliding: bool = False
    _last_steering: float | None = None

    @property
    def average_speed(self) -> float:
        if self.elapsed_time <= 0.0:
            return 0.0
        return self.distance_traveled / self.elapsed_time

    def reset(self) -> None:
        self.distance_traveled = 0.0
        self.collisions = 0
        self.stuck_time = 0.0
        self.elapsed_time = 0.0
        self.steering_jitter = 0.0
        self.min_obstacle_clearance = None
        self.failed = False
        self.failure_reason = ""
        self.waypoints_reached = 0
        self._last_position = None
        self._was_colliding = False
        self._last_steering = None


class Obstacle(Protocol):
    def sample_points(self, spacing: float = 0.15) -> list[tuple[float, float]]:
        ...

    def sample_visible_points(self, origin_x: float, origin_y: float, spacing: float = 0.15) -> list[tuple[float, float]]:
        ...

    def clearance(self, x: float, y: float) -> float:
        ...


@dataclass
class WaypointConfig:
    reach_radius: float = 0.45
    min_distance_from_boat: float = 5.0
    arena_margin: float = 0.9
    obstacle_margin: float = 0.8
    heading_for_full_steer_rad: float = math.radians(70.0)
    heading_slowdown_rad: float = math.radians(55.0)
    min_throttle: float = 0.18
    approach_slow_radius: float = 1.6
    avoidance_steering_weight: float = 1.0


@dataclass
class WaypointState:
    x: float = 0.0
    y: float = 8.0
    seed: int | None = None
    _rng: random.Random = field(default_factory=random.Random, init=False, repr=False)

    def reset(
        self,
        boat: BoatState,
        cfg: WaypointConfig,
        course: "ObstacleCourse | None" = None,
        randomize_seed: bool = True,
    ) -> None:
        if randomize_seed or self.seed is None:
            self.seed = random.randrange(1, 2_147_483_647)
        self._rng = random.Random(self.seed)
        self.advance_from(boat, cfg, course)

    def advance_from(self, boat: BoatState, cfg: WaypointConfig, course: "ObstacleCourse | None" = None) -> None:
        arena_radius = course.arena_radius if course else 10.0
        usable_radius = max(1.0, arena_radius - cfg.arena_margin)
        for _ in range(200):
            x, y = random_point_in_circle(self._rng, usable_radius)
            if math.hypot(x - boat.x, y - boat.y) < cfg.min_distance_from_boat:
                continue
            if course is not None and course.clearance(x, y) < cfg.obstacle_margin:
                continue
            self.x = x
            self.y = y
            return
        self.x, self.y = random_point_in_circle(self._rng, usable_radius)


@dataclass
class WaypointControl:
    throttle: float
    steering: float
    distance: float
    heading_error: float
    reached: bool


@dataclass
class SimControl:
    throttle: float
    steering: float
    waypoint_throttle: float
    waypoint_steering: float
    avoidance_throttle: float
    avoidance_steering: float
    waypoint_distance: float
    heading_error: float
    score: float = 0.0
    min_predicted_clearance: float | None = None


@dataclass
class ControllerState:
    throttle: float = 0.0
    steering: float = 0.0

    def reset(self) -> None:
        self.throttle = 0.0
        self.steering = 0.0

    def apply(self, control: SimControl) -> None:
        self.throttle = control.throttle
        self.steering = control.steering


@dataclass(frozen=True)
class CandidateControl:
    throttle: float
    steering: float


@dataclass
class CandidateControllerConfig:
    horizon_s: float = 2.2
    step_s: float = 0.18
    steering_deltas: tuple[float, ...] = (-0.08, -0.04, 0.0, 0.04, 0.08)
    throttle_deltas: tuple[float, ...] = (-0.06, 0.0, 0.04)
    obstacle_point_radius: float = 0.18
    clearance_buffer: float = 0.65
    waypoint_distance_weight: float = 3.0
    waypoint_progress_weight: float = 8.0
    heading_error_weight: float = 0.55
    low_clearance_weight: float = 18.0
    speed_near_obstacle_weight: float = 3.5
    arena_margin_weight: float = 24.0
    collision_penalty: float = 900.0
    steering_effort_weight: float = 0.10
    low_speed_weight: float = 0.22
    idle_throttle_threshold: float = 0.001
    idle_when_clear_penalty: float = 80.0


@dataclass(frozen=True)
class CircleObstacle:
    x: float
    y: float
    radius: float

    def sample_points(self, spacing: float = 0.15) -> list[tuple[float, float]]:
        count = max(10, int(math.ceil(2.0 * math.pi * self.radius / max(spacing, 0.03))))
        return [
            (
                self.x + math.cos(2.0 * math.pi * i / count) * self.radius,
                self.y + math.sin(2.0 * math.pi * i / count) * self.radius,
            )
            for i in range(count)
        ]

    def sample_visible_points(self, origin_x: float, origin_y: float, spacing: float = 0.15) -> list[tuple[float, float]]:
        visible = []
        for point_x, point_y in self.sample_points(spacing=spacing):
            normal_x = point_x - self.x
            normal_y = point_y - self.y
            to_origin_x = origin_x - point_x
            to_origin_y = origin_y - point_y
            if normal_x * to_origin_x + normal_y * to_origin_y > 0.0:
                visible.append((point_x, point_y))
        return visible

    def clearance(self, x: float, y: float) -> float:
        return math.hypot(x - self.x, y - self.y) - self.radius


@dataclass(frozen=True)
class RectObstacle:
    x: float
    y: float
    width: float
    height: float

    def sample_points(self, spacing: float = 0.15) -> list[tuple[float, float]]:
        half_w = self.width / 2.0
        half_h = self.height / 2.0
        xs = _samples_between(self.x - half_w, self.x + half_w, spacing)
        ys = _samples_between(self.y - half_h, self.y + half_h, spacing)
        points = [(x, self.y - half_h) for x in xs]
        points += [(x, self.y + half_h) for x in xs]
        points += [(self.x - half_w, y) for y in ys]
        points += [(self.x + half_w, y) for y in ys]
        return list(dict.fromkeys(points))

    def sample_visible_points(self, origin_x: float, origin_y: float, spacing: float = 0.15) -> list[tuple[float, float]]:
        half_w = self.width / 2.0
        half_h = self.height / 2.0
        left = self.x - half_w
        right = self.x + half_w
        bottom = self.y - half_h
        top = self.y + half_h
        xs = _samples_between(left, right, spacing)
        ys = _samples_between(bottom, top, spacing)
        edges = [
            ([(x, bottom) for x in xs], (0.0, -1.0), (self.x, bottom)),
            ([(x, top) for x in xs], (0.0, 1.0), (self.x, top)),
            ([(left, y) for y in ys], (-1.0, 0.0), (left, self.y)),
            ([(right, y) for y in ys], (1.0, 0.0), (right, self.y)),
        ]
        visible = []
        for points, normal, midpoint in edges:
            to_origin = (origin_x - midpoint[0], origin_y - midpoint[1])
            if normal[0] * to_origin[0] + normal[1] * to_origin[1] > 0.0:
                visible.extend(points)
        return list(dict.fromkeys(visible))

    def clearance(self, x: float, y: float) -> float:
        dx = abs(x - self.x) - self.width / 2.0
        dy = abs(y - self.y) - self.height / 2.0
        outside_x = max(dx, 0.0)
        outside_y = max(dy, 0.0)
        outside_distance = math.hypot(outside_x, outside_y)
        if dx <= 0.0 and dy <= 0.0:
            return max(dx, dy)
        return outside_distance


def _samples_between(start: float, stop: float, spacing: float) -> list[float]:
    distance = max(stop - start, 0.0)
    count = max(2, int(math.ceil(distance / max(spacing, 0.03))) + 1)
    return [start + distance * i / (count - 1) for i in range(count)]


@dataclass
class ObstacleCourse:
    obstacles: list[Obstacle]
    arena_radius: float = 20.0
    obstacle_count: int = 48
    start_clear_radius: float = 1.3
    obstacle_spacing: float = 0.55
    seed: int | None = None
    _rng: random.Random = field(default_factory=lambda: random.Random(7), init=False, repr=False)

    @classmethod
    def default(cls, seed: int | None = None) -> "ObstacleCourse":
        course = cls([], seed=seed)
        course.reset(randomize_seed=seed is None)
        return course

    def reset(self, randomize_seed: bool = True) -> None:
        if randomize_seed or self.seed is None:
            self.seed = random.randrange(1, 2_147_483_647)
        self._rng = random.Random(self.seed)
        self.obstacles = []
        self._scatter_obstacles()

    def update_for_boat(self, boat_y: float) -> None:
        _ = boat_y

    def _scatter_obstacles(self) -> None:
        attempts = 0
        while len(self.obstacles) < self.obstacle_count and attempts < self.obstacle_count * 80:
            attempts += 1
            obstacle = self._make_random_obstacle()
            if self._can_place(obstacle):
                self.obstacles.append(obstacle)

    def _make_random_obstacle(self) -> Obstacle:
        if self._rng.random() < 0.72:
            radius = self._rng.uniform(0.18, 0.38)
            x, y = random_point_in_circle(self._rng, self.arena_radius - radius - 0.35)
            return CircleObstacle(x, y, radius)
        width = self._rng.uniform(0.42, 0.9)
        height = self._rng.uniform(0.22, 0.48)
        clearance = math.hypot(width / 2.0, height / 2.0)
        x, y = random_point_in_circle(self._rng, self.arena_radius - clearance - 0.35)
        return RectObstacle(x, y, width, height)

    def _can_place(self, candidate: Obstacle) -> bool:
        min_x, max_x, min_y, max_y = obstacle_bounds([candidate])
        if min(math.hypot(min_x, min_y), math.hypot(min_x, max_y), math.hypot(max_x, min_y), math.hypot(max_x, max_y)) < self.start_clear_radius:
            return False
        for obstacle in self.obstacles:
            for x, y in candidate.sample_points(spacing=0.25):
                if obstacle.clearance(x, y) < self.obstacle_spacing:
                    return False
        return True

    def sample_points(self, spacing: float = 0.15) -> list[tuple[float, float]]:
        points: list[tuple[float, float]] = []
        for obstacle in self.obstacles:
            points.extend(obstacle.sample_points(spacing=spacing))
        return points

    def clearance(self, x: float, y: float) -> float:
        if not self.obstacles:
            return math.inf
        return min(obstacle.clearance(x, y) for obstacle in self.obstacles)

    def arena_clearance(self, x: float, y: float) -> float:
        return self.arena_radius - math.hypot(x, y)


def random_point_in_circle(rng: random.Random, radius: float) -> tuple[float, float]:
    angle = rng.uniform(0.0, 2.0 * math.pi)
    distance = radius * math.sqrt(rng.random())
    return math.cos(angle) * distance, math.sin(angle) * distance


def world_to_boat_relative(boat: BoatState, world_x: float, world_y: float) -> tuple[float, float]:
    dx = world_x - boat.x
    dy = world_y - boat.y
    sin_h = math.sin(boat.heading)
    cos_h = math.cos(boat.heading)
    lateral_x = dx * cos_h - dy * sin_h
    forward_y = dx * sin_h + dy * cos_h
    return lateral_x, forward_y


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def compute_waypoint_control(boat: BoatState, waypoint: WaypointState, cfg: WaypointConfig) -> WaypointControl:
    dx = waypoint.x - boat.x
    dy = waypoint.y - boat.y
    distance = math.hypot(dx, dy)
    target_heading = math.atan2(dx, dy)
    heading_error = normalize_angle(target_heading - boat.heading)
    steering = clamp(heading_error / cfg.heading_for_full_steer_rad, -1.0, 1.0)

    heading_factor = 1.0 - clamp(abs(heading_error) / cfg.heading_slowdown_rad, 0.0, 0.75)
    approach_factor = clamp(distance / cfg.approach_slow_radius, cfg.min_throttle, 1.0)
    throttle = clamp(heading_factor * approach_factor, 0.0, 1.0)
    reached = distance <= cfg.reach_radius
    if reached:
        throttle = 0.0
        steering = 0.0

    return WaypointControl(
        throttle=throttle,
        steering=steering,
        distance=distance,
        heading_error=heading_error,
        reached=reached,
    )


def blend_waypoint_and_avoidance(
    waypoint_control: WaypointControl,
    nav_output: NavOutput,
    cfg: WaypointConfig,
) -> SimControl:
    steering = clamp(
        waypoint_control.steering + nav_output.steering * cfg.avoidance_steering_weight,
        -1.0,
        1.0,
    )
    throttle = min(waypoint_control.throttle, nav_output.throttle)
    return SimControl(
        throttle=throttle,
        steering=steering,
        waypoint_throttle=waypoint_control.throttle,
        waypoint_steering=waypoint_control.steering,
        avoidance_throttle=nav_output.throttle,
        avoidance_steering=nav_output.steering,
        waypoint_distance=waypoint_control.distance,
        heading_error=waypoint_control.heading_error,
    )


def choose_candidate_control(
    boat: BoatState,
    boat_cfg: BoatConfig,
    waypoint: WaypointState,
    waypoint_cfg: WaypointConfig,
    nav_output: NavOutput,
    course: ObstacleCourse,
    controller_state: ControllerState | None = None,
    controller_cfg: CandidateControllerConfig | None = None,
) -> SimControl:
    cfg = controller_cfg or CandidateControllerConfig()
    current_control = controller_state or ControllerState()
    waypoint_control = compute_waypoint_control(boat, waypoint, waypoint_cfg)
    if waypoint_control.reached:
        return SimControl(
            throttle=0.0,
            steering=0.0,
            waypoint_throttle=0.0,
            waypoint_steering=0.0,
            avoidance_throttle=nav_output.throttle,
            avoidance_steering=nav_output.steering,
            waypoint_distance=waypoint_control.distance,
            heading_error=waypoint_control.heading_error,
            score=0.0,
            min_predicted_clearance=None,
        )

    radar_world_points = radar_points_to_world(boat, nav_output.filtered_points)
    candidates = _candidate_controls_around(current_control, cfg)
    initial_distance = waypoint_control.distance
    best_control: SimControl | None = None
    best_cost = math.inf

    for candidate in candidates:
        cost, min_clearance = score_candidate_control(
            boat,
            boat_cfg,
            waypoint,
            course,
            radar_world_points,
            candidate,
            cfg,
            initial_distance,
            front_blocked=nav_output.front_blocked,
        )
        if cost < best_cost:
            best_cost = cost
            final_wp = compute_waypoint_control(_rollout_boat(boat, boat_cfg, candidate, cfg.horizon_s, cfg.step_s), waypoint, waypoint_cfg)
            best_control = SimControl(
                throttle=candidate.throttle,
                steering=candidate.steering,
                waypoint_throttle=waypoint_control.throttle,
                waypoint_steering=waypoint_control.steering,
                avoidance_throttle=nav_output.throttle,
                avoidance_steering=nav_output.steering,
                waypoint_distance=waypoint_control.distance,
                heading_error=final_wp.heading_error,
                score=cost,
                min_predicted_clearance=min_clearance,
            )

    if best_control is None:
        return blend_waypoint_and_avoidance(waypoint_control, nav_output, waypoint_cfg)
    return best_control


def _candidate_controls_around(controller_state: ControllerState, cfg: CandidateControllerConfig) -> list[CandidateControl]:
    steering_values = {
        round(clamp(controller_state.steering + delta, -1.0, 1.0), 3)
        for delta in cfg.steering_deltas
    }
    throttle_values = {
        round(clamp(controller_state.throttle + delta, 0.0, 1.0), 3)
        for delta in cfg.throttle_deltas
    }
    return [
        CandidateControl(throttle=throttle, steering=steering)
        for throttle in sorted(throttle_values)
        for steering in sorted(steering_values)
    ]


def _copy_boat_for_rollout(boat: BoatState) -> BoatState:
    return BoatState(
        x=boat.x,
        y=boat.y,
        heading=boat.heading,
        speed=boat.speed,
        yaw_rate=boat.yaw_rate,
    )


def _rollout_boat(
    boat: BoatState,
    boat_cfg: BoatConfig,
    candidate: CandidateControl,
    horizon_s: float,
    step_s: float,
) -> BoatState:
    probe = _copy_boat_for_rollout(boat)
    steps = max(1, int(math.ceil(horizon_s / step_s)))
    dt = horizon_s / steps
    for _ in range(steps):
        update_boat(probe, boat_cfg, candidate.throttle, candidate.steering, dt)
    return probe


def radar_points_to_world(boat: BoatState, points: list[dict]) -> list[tuple[float, float]]:
    sin_h = math.sin(boat.heading)
    cos_h = math.cos(boat.heading)
    world_points = []
    for point in points:
        rel_x = float(point.get("x", 0.0))
        rel_y = float(point.get("y", 0.0))
        dx = rel_x * cos_h + rel_y * sin_h
        dy = -rel_x * sin_h + rel_y * cos_h
        world_points.append((boat.x + dx, boat.y + dy))
    return world_points


def score_candidate_control(
    boat: BoatState,
    boat_cfg: BoatConfig,
    waypoint: WaypointState,
    course: ObstacleCourse,
    radar_world_points: list[tuple[float, float]],
    candidate: CandidateControl,
    cfg: CandidateControllerConfig,
    initial_waypoint_distance: float | None = None,
    front_blocked: bool = False,
) -> tuple[float, float | None]:
    probe = _copy_boat_for_rollout(boat)
    steps = max(1, int(math.ceil(cfg.horizon_s / cfg.step_s)))
    dt = cfg.horizon_s / steps
    min_clearance: float | None = None
    cost = 0.0

    for _ in range(steps):
        update_boat(probe, boat_cfg, candidate.throttle, candidate.steering, dt)
        radar_clearance = _point_cloud_clearance(probe, radar_world_points, boat_cfg, cfg)
        arena_clearance = course.arena_clearance(probe.x, probe.y) - boat_cfg.collision_radius
        if radar_clearance is not None:
            min_clearance = radar_clearance if min_clearance is None else min(min_clearance, radar_clearance)
            if radar_clearance <= 0.0:
                cost += cfg.collision_penalty * (1.0 + candidate.throttle * 2.0)
            elif radar_clearance < cfg.clearance_buffer:
                cost += cfg.low_clearance_weight * ((cfg.clearance_buffer - radar_clearance) / cfg.clearance_buffer) ** 2
            if radar_clearance < cfg.clearance_buffer * 1.8:
                cost += cfg.speed_near_obstacle_weight * candidate.throttle
        if arena_clearance <= 0.0:
            cost += cfg.collision_penalty
        elif arena_clearance < cfg.clearance_buffer:
            cost += cfg.arena_margin_weight * ((cfg.clearance_buffer - arena_clearance) / cfg.clearance_buffer) ** 2

    final_distance = math.hypot(waypoint.x - probe.x, waypoint.y - probe.y)
    initial_distance = initial_waypoint_distance
    if initial_distance is None:
        initial_distance = math.hypot(waypoint.x - boat.x, waypoint.y - boat.y)
    progress = initial_distance - final_distance
    heading_error = abs(normalize_angle(math.atan2(waypoint.x - probe.x, waypoint.y - probe.y) - probe.heading))
    cost += cfg.waypoint_distance_weight * final_distance
    cost -= cfg.waypoint_progress_weight * progress
    cost += cfg.heading_error_weight * heading_error
    cost += cfg.steering_effort_weight * abs(candidate.steering)
    cost += cfg.low_speed_weight * (1.0 - candidate.throttle)
    if not front_blocked and initial_distance > 1.0 and candidate.throttle <= cfg.idle_throttle_threshold:
        cost += cfg.idle_when_clear_penalty
    return cost, min_clearance


def _point_cloud_clearance(
    boat: BoatState,
    points: list[tuple[float, float]],
    boat_cfg: BoatConfig,
    cfg: CandidateControllerConfig,
) -> float | None:
    if not points:
        return None
    return min(
        math.hypot(boat.x - point_x, boat.y - point_y) - boat_cfg.collision_radius - cfg.obstacle_point_radius
        for point_x, point_y in points
    )


def generate_radar_points(
    boat: BoatState,
    course: ObstacleCourse,
    nav_cfg: NavConfig,
    sample_spacing: float = 0.14,
    snr_raw: int = 260,
) -> list[dict]:
    points = []
    for obstacle in course.obstacles:
        for world_x, world_y in obstacle.sample_visible_points(boat.x, boat.y, spacing=sample_spacing):
            x, y = world_to_boat_relative(boat, world_x, world_y)
            if y < nav_cfg.min_y:
                continue
            if nav_cfg.max_y is not None and y > nav_cfg.max_y:
                continue
            if nav_cfg.lateral_limit is not None and abs(x) > nav_cfg.lateral_limit:
                continue
            points.append(
                {
                    "x": x,
                    "y": y,
                    "z": 0.0,
                    "doppler": 0.0,
                    "snr_raw": snr_raw,
                    "noise_raw": 900,
                }
            )
    return points


def update_boat(boat: BoatState, cfg: BoatConfig, throttle: float, steering: float, dt: float) -> None:
    throttle = clamp(throttle, 0.0, 1.0)
    steering = clamp(steering, -1.0, 1.0)
    speed_sign = 1.0 if boat.speed >= 0.0 else -1.0
    drag = cfg.linear_drag * boat.speed + cfg.quadratic_drag * boat.speed * abs(boat.speed)

    boat.speed += (cfg.max_thrust * throttle - drag) * dt
    if speed_sign > 0.0 and boat.speed < 0.0:
        boat.speed = 0.0
    boat.speed = clamp(boat.speed, 0.0, cfg.max_speed)

    rudder_effect = steering * cfg.rudder_strength * min(abs(boat.speed), cfg.max_speed)
    boat.yaw_rate += (rudder_effect - cfg.yaw_damping * boat.yaw_rate) * dt
    boat.heading += boat.yaw_rate * dt

    boat.x += math.sin(boat.heading) * boat.speed * dt
    boat.y += math.cos(boat.heading) * boat.speed * dt
    boat.trail.append((boat.x, boat.y))
    if len(boat.trail) > cfg.trail_limit:
        del boat.trail[: len(boat.trail) - cfg.trail_limit]


def update_metrics(
    metrics: SimMetrics,
    boat: BoatState,
    boat_cfg: BoatConfig,
    course: ObstacleCourse,
    output: NavOutput | None,
    dt: float,
    control_steering: float | None = None,
) -> None:
    position = (boat.x, boat.y)
    if metrics._last_position is not None:
        metrics.distance_traveled += math.hypot(
            position[0] - metrics._last_position[0],
            position[1] - metrics._last_position[1],
        )
    metrics._last_position = position
    metrics.elapsed_time += dt

    clearance = course.clearance(boat.x, boat.y) - boat_cfg.collision_radius
    arena_clearance = course.arena_clearance(boat.x, boat.y) - boat_cfg.collision_radius
    metrics.min_obstacle_clearance = (
        clearance
        if metrics.min_obstacle_clearance is None
        else min(metrics.min_obstacle_clearance, clearance)
    )
    boat.collided = clearance <= 0.0
    if boat.collided and not metrics._was_colliding:
        metrics.collisions += 1
        boat.failed = True
        metrics.failed = True
        metrics.failure_reason = "collision"
    if arena_clearance <= 0.0 and not metrics.failed:
        boat.failed = True
        metrics.failed = True
        metrics.failure_reason = "left arena"
    metrics._was_colliding = boat.collided

    if output is not None:
        if output.throttle > 0.4 and boat.speed < boat_cfg.stuck_speed:
            metrics.stuck_time += dt
        steering = output.steering if control_steering is None else control_steering
        if metrics._last_steering is not None:
            metrics.steering_jitter += abs(steering - metrics._last_steering)
        metrics._last_steering = steering


def reset_sim(
    boat: BoatState,
    metrics: SimMetrics,
    start_x: float = 0.0,
    start_y: float = 0.0,
    heading: float = 0.0,
) -> None:
    boat.reset()
    boat.x = start_x
    boat.y = start_y
    boat.heading = heading
    metrics.reset()


def obstacle_bounds(obstacles: Iterable[Obstacle]) -> tuple[float, float, float, float]:
    xs: list[float] = []
    ys: list[float] = []
    for obstacle in obstacles:
        for x, y in obstacle.sample_points(spacing=0.25):
            xs.append(x)
            ys.append(y)
    if not xs or not ys:
        return -1.0, 1.0, 0.0, 6.0
    return min(xs), max(xs), min(ys), max(ys)
