from __future__ import annotations

import threading
import time
from dataclasses import asdict, dataclass, field

from boat_core.heading import HeadingEstimator, HeadingEstimatorConfig
from gnss.geo import bearing_deg, distance_m, heading_error_deg
from radar_nav.waypoint import GeoWaypoint, WaypointControl, WaypointNavConfig


@dataclass
class AutoConfig:
    controller: str = "smooth_pd_v1"
    control_hz: float = 10.0
    min_speed_for_course_mps: float = 0.08
    gnss_reanchor_speed_mps: float = 0.3
    gnss_heading_blend: float = 0.02
    gnss_stale_s: float = 3.0
    route_match_tolerance_m: float = 1.0
    imu_stale_s: float = 2.0
    heading_command_stale_s: float = 0.75
    heading_learn_min_forward_us: int = 1565
    heading_learn_max_thruster_delta_us: int = 20
    heading_learn_duration_s: float = 1.0
    heading_learn_min_samples: int = 5
    heading_course_stability_deg: float = 10.0
    heading_course_agreement_deg: float = 45.0
    heading_deadband_deg: float = 8.0
    yaw_rate_deadband_dps: float = 2.0
    yaw_lookahead_s: float = 2.0
    pulse_turn_enter_deg: float = 18.0
    pulse_turn_exit_deg: float = 6.0
    pulse_reverse_deg: float = 38.0
    pulse_duration_s: float = 0.25
    pulse_observe_s: float = 0.35
    smooth_kp: float = 0.025
    smooth_kd: float = 0.035
    smooth_turn_deadband: float = 0.08
    smooth_pwm_slew_us_per_s: float = 300.0
    behind_enter_deg: float = 125.0
    behind_exit_deg: float = 70.0
    neutral_us: int = 1500
    level1_us: int = 1565
    level2_us: int = 1575
    level3_us: int = 1650
    reverse_level1_us: int = 1460
    reverse_level2_us: int = 1445
    reverse_level3_us: int = 1425
    waypoint: WaypointNavConfig = field(default_factory=WaypointNavConfig)


class AutoController:
    def __init__(
        self,
        control_state,
        gnss_reader,
        imu_reader=None,
        config: AutoConfig | None = None,
        thruster_reader=None,
        clock=None,
    ) -> None:
        self.control_state = control_state
        self.gnss_reader = gnss_reader
        self.imu_reader = imu_reader
        self.thruster_reader = thruster_reader
        self.config = config or AutoConfig()
        self._lock = threading.Lock()
        self._waypoints: list[GeoWaypoint] = []
        self._active_index = 0
        self._status = self._base_status("idle", "no route")
        self._heading_estimator = HeadingEstimator(
            HeadingEstimatorConfig(
                imu_stale_s=self.config.imu_stale_s,
                command_stale_s=self.config.heading_command_stale_s,
                min_forward_us=self.config.heading_learn_min_forward_us,
                max_thruster_delta_us=(
                    self.config.heading_learn_max_thruster_delta_us
                ),
                min_speed_mps=self.config.gnss_reanchor_speed_mps,
                learn_duration_s=self.config.heading_learn_duration_s,
                learn_min_samples=self.config.heading_learn_min_samples,
                course_stability_deg=self.config.heading_course_stability_deg,
                course_agreement_deg=self.config.heading_course_agreement_deg,
                correction_blend=self.config.gnss_heading_blend,
            ),
            clock=clock,
        )
        self._heading_reset_pending = False
        self._pulse_action: str | None = None
        self._pulse_until = 0.0
        self._pulse_observe_until = 0.0
        self._latched_turn: str | None = None
        self._last_active_pwm: tuple[int, int] = (self.config.neutral_us, self.config.neutral_us)
        self._smooth_left_us = float(self.config.neutral_us)
        self._smooth_right_us = float(self.config.neutral_us)
        self._smooth_last_at: float | None = None
        self._behind_turn: str | None = None
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def shutdown(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=1.0)

    def tick(self) -> None:
        """Run one control update from the latest sensor snapshots."""
        self._tick()

    def relearn_heading(self) -> dict:
        """Forget the world-heading anchor and wait for a new straight run."""
        with self._lock:
            self._heading_reset_pending = True
        status = self._base_status(
            "acquiring_heading",
            "heading reset; drive straight in manual",
        )
        status["heading"] = {
            "state": "uninitialized",
            "reason": "heading reset; waiting for straight manual motion",
            "heading_deg": None,
            "source": "uninitialized",
            "confidence": "none",
        }
        self._set_status(status)
        self.control_state.apply_auto_pwm(
            self.config.neutral_us,
            self.config.neutral_us,
            status["reason"],
            status,
        )
        return status

    def set_waypoints(self, records: list[dict]) -> dict:
        waypoints = [self._coerce_waypoint(record, index) for index, record in enumerate(records)]
        with self._lock:
            active_index = self._next_active_index_for_route_update(waypoints)
            status = {
                "state": "idle",
                "reason": "route loaded" if waypoints else "route cleared",
                "active_index": active_index,
                "total": len(waypoints),
                "target": None,
                "distance_m": None,
                "bearing_deg": None,
                "heading_deg": None,
                "heading_error_deg": None,
                "controller": self.config.controller,
                "control_hz": self.config.control_hz,
            }
            self._waypoints = waypoints
            self._active_index = active_index
            self._status = status
            self._clear_controller_state()
        self.control_state.set_auto_status(self.status())
        return self.route_snapshot()

    def route_snapshot(self) -> dict:
        return {"waypoints": [asdict(item) for item in self._waypoints], "status": self.status()}

    def status(self) -> dict:
        with self._lock:
            return dict(self._status)

    def _next_active_index_for_route_update(self, new_waypoints: list[GeoWaypoint]) -> int:
        if not new_waypoints:
            return 0
        current_index = min(max(self._active_index, 0), len(new_waypoints))
        if not self._waypoints:
            return 0
        if current_index > len(self._waypoints):
            return 0
        for index in range(min(current_index, len(self._waypoints))):
            if not self._same_waypoint(self._waypoints[index], new_waypoints[index]):
                return 0
        if current_index < len(self._waypoints) and current_index < len(new_waypoints):
            if not self._same_waypoint(self._waypoints[current_index], new_waypoints[current_index]):
                return 0
        return current_index

    def can_arm(self) -> tuple[bool, str]:
        with self._lock:
            has_route = bool(self._waypoints)
        if not has_route:
            return False, "auto requires at least one waypoint"
        return True, "auto ready"

    def _run(self) -> None:
        period = 1.0 / max(self.config.control_hz, 1.0)
        while not self._stop_event.is_set():
            self._tick()
            self._stop_event.wait(period)

    def _tick(self) -> None:
        with self.control_state._lock:  # noqa: SLF001 - dashboard-local coordination
            mode = self.control_state.mode
        health, gnss = self._gnss_snapshot()
        imu_health, imu = self._imu_snapshot()
        command_health, command = self._thruster_snapshot()
        with self._lock:
            reset_heading = self._heading_reset_pending
            self._heading_reset_pending = False
        if reset_heading:
            self._heading_estimator.reset()
        gnss_ok, gnss_reason = self._valid_gnss(health, gnss)
        estimate = self._heading_estimator.update(
            gnss,
            imu_health,
            imu,
            command_health,
            command,
        )

        if mode != "auto":
            status = self._base_status("idle", "auto not armed")
            status["heading"] = estimate
            self._set_status(status)
            self.control_state.set_auto_status(self.status())
            return

        if not gnss_ok:
            self._block(gnss_reason)
            return

        with self._lock:
            if not self._waypoints:
                waypoint = None
            elif self._active_index >= len(self._waypoints):
                waypoint = None
            else:
                waypoint = self._waypoints[self._active_index]
                active_index = self._active_index
                total = len(self._waypoints)

        if waypoint is None:
            self._set_status(self._base_status("reached", "route complete"))
            self.control_state.apply_auto_pwm(self.config.neutral_us, self.config.neutral_us, "route complete", self.status())
            return

        dist = distance_m(float(gnss["lat"]), float(gnss["lon"]), waypoint.lat, waypoint.lon)
        bearing = bearing_deg(float(gnss["lat"]), float(gnss["lon"]), waypoint.lat, waypoint.lon)

        heading = estimate["heading_deg"]
        if heading is None:
            status = self._status_for(
                "acquiring_heading",
                "heading not locked; drive straight in manual",
                waypoint,
                active_index,
                total,
                dist,
                bearing,
                None,
                None,
                {
                    "action": "wait_for_heading",
                    "left_us": self.config.neutral_us,
                    "right_us": self.config.neutral_us,
                    "heading": estimate,
                },
            )
            status["heading"] = estimate
            self._set_status(status)
            self.control_state.apply_auto_pwm(
                self.config.neutral_us,
                self.config.neutral_us,
                status["reason"],
                status,
            )
            return

        error = heading_error_deg(heading, bearing)
        control = self._compute_waypoint_control(dist, error, imu_health, imu)

        if control.reached:
            with self._lock:
                self._active_index += 1
                done = self._active_index >= len(self._waypoints)
            if done:
                status = self._status_for("reached", "route complete", waypoint, active_index, total, dist, bearing, error, heading)
                status["heading"] = estimate
                self._set_status(status)
                self.control_state.apply_auto_pwm(self.config.neutral_us, self.config.neutral_us, "route complete", status)
            else:
                self._set_status(self._base_status("navigating", "waypoint reached; advancing"))
            return

        status = self._status_for(
            "navigating",
            f"target {active_index + 1}/{total}: {dist:.1f} m",
            waypoint,
            active_index,
            total,
            dist,
            bearing,
            error,
            heading,
            control.metadata,
        )
        status["heading"] = estimate
        self._set_status(status)
        self.control_state.apply_auto_pwm(control.left_us, control.right_us, status["reason"], status)

    def _block(self, reason: str) -> None:
        status = self._base_status("blocked", reason)
        self._set_status(status)
        self.control_state.apply_auto_pwm(self.config.neutral_us, self.config.neutral_us, reason, status)

    def _gnss_snapshot(self) -> tuple[str, dict]:
        if self.gnss_reader is None:
            return "unavailable", {}
        return self.gnss_reader.snapshot()

    def _imu_snapshot(self) -> tuple[str, dict]:
        if self.imu_reader is None:
            return "unavailable", {}
        return self.imu_reader.snapshot()

    def _thruster_snapshot(self) -> tuple[str, dict]:
        if self.thruster_reader is None:
            return "unavailable", {}
        return self.thruster_reader.snapshot()

    def _valid_gnss(self, health: str, gnss: dict) -> tuple[bool, str]:
        if health not in ("live", "waiting"):
            return False, f"GNSS {health}"
        if gnss.get("fix") not in ("fix", "estimated"):
            return False, "GNSS fix unavailable"
        if gnss.get("lat") is None or gnss.get("lon") is None:
            return False, "GNSS position unavailable"
        age = gnss.get("age_s")
        if age is not None and float(age) > self.config.gnss_stale_s:
            return False, "GNSS stale"
        return True, "GNSS ok"

    def _valid_imu_yaw_rate(self, health: str, imu: dict) -> float | None:
        if health != "live":
            return None
        age = imu.get("age_s")
        if age is not None and float(age) > self.config.imu_stale_s:
            return None
        yaw_rate = imu.get("gyro_z_dps")
        if yaw_rate is None:
            return None
        yaw_rate = float(yaw_rate)
        return 0.0 if abs(yaw_rate) < self.config.yaw_rate_deadband_dps else yaw_rate

    def _compute_waypoint_control(self, dist: float, error: float, imu_health: str, imu: dict) -> WaypointControl:
        if self.config.controller == "smooth_pd_v1":
            return self._compute_smooth_pd_control(dist, error, imu_health, imu)
        if self.config.controller == "pulse_yaw_v1":
            return self._compute_pulse_yaw_control(dist, error, imu_health, imu)
        return self._compute_guesstimate_control(dist, error, imu_health, imu)

    def _compute_guesstimate_control(self, dist: float, error: float, imu_health: str, imu: dict) -> WaypointControl:
        reached = dist <= self.config.waypoint.reach_radius_m
        metadata = {
            "controller": "guesstimate_rate_v1",
            "raw_heading_error_deg": round(error, 2),
            "imu_health": imu_health,
            "yaw_rate_dps": None,
            "predicted_error_deg": round(error, 2),
        }
        if reached:
            return WaypointControl(
                distance_m=dist,
                heading_error_deg=error,
                reached=True,
                left_us=self.config.neutral_us,
                right_us=self.config.neutral_us,
                action="reached",
                metadata={**metadata, "action": "reached"},
            )

        yaw_rate = self._valid_imu_yaw_rate(imu_health, imu)
        predicted_error = error
        if yaw_rate is not None:
            predicted_error = heading_error_deg(yaw_rate * self.config.yaw_lookahead_s, error)
            metadata["yaw_rate_dps"] = round(yaw_rate, 2)
            metadata["predicted_error_deg"] = round(predicted_error, 2)

        abs_error = abs(error)
        if abs(predicted_error) <= self.config.heading_deadband_deg:
            action = "forward"
            steering_reason = "coasting/aligned"
        else:
            action = "arc_right" if predicted_error > 0 else "arc_left"
            steering_reason = "correcting predicted heading error"

        level = 3
        throttle_reason = "max auto output"

        # When we are already rotating quickly toward the target, do not add
        # extra differential thrust; let the boat's momentum carry the arc.
        if yaw_rate is not None and error * yaw_rate > 0 and abs(yaw_rate) * self.config.yaw_lookahead_s >= abs_error * 0.65:
            action = "coast"
            steering_reason = "yaw rate already correcting"

        left_us, right_us = self._action_to_pwm(action, level)
        return WaypointControl(
            distance_m=dist,
            heading_error_deg=error,
            reached=False,
            left_us=left_us,
            right_us=right_us,
            action=action,
            metadata={
                **metadata,
                "action": action,
                "left_us": left_us,
                "right_us": right_us,
                "steering_reason": steering_reason,
                "throttle_reason": throttle_reason,
                "level": level,
                "level1_us": self.config.level1_us,
                "level2_us": self.config.level2_us,
                "level3_us": self.config.level3_us,
            },
        )

    def _compute_pulse_yaw_control(self, dist: float, error: float, imu_health: str, imu: dict) -> WaypointControl:
        reached = dist <= self.config.waypoint.reach_radius_m
        yaw_rate = self._valid_imu_yaw_rate(imu_health, imu)
        predicted_error = error
        if yaw_rate is not None:
            predicted_error = heading_error_deg(yaw_rate * self.config.yaw_lookahead_s, error)

        metadata = {
            "controller": "pulse_yaw_v1",
            "raw_heading_error_deg": round(error, 2),
            "predicted_error_deg": round(predicted_error, 2),
            "yaw_rate_dps": None if yaw_rate is None else round(yaw_rate, 2),
            "imu_health": imu_health,
            "latched_turn": self._latched_turn,
        }
        if reached:
            self._clear_pulse_state()
            return WaypointControl(
                distance_m=dist,
                heading_error_deg=error,
                reached=True,
                left_us=self.config.neutral_us,
                right_us=self.config.neutral_us,
                action="reached",
                metadata={**metadata, "action": "reached"},
            )

        now = time.monotonic()
        level = 3
        abs_error = abs(error)
        abs_predicted = abs(predicted_error)
        display_action = None

        if self._pulse_action is not None and now < self._pulse_until:
            action = self._pulse_action
            steering_reason = "committed turn pulse"
        elif now < self._pulse_observe_until:
            action = "observe"
            steering_reason = "observing yaw after pulse"
            display_action = self._latched_turn or "hold"
        else:
            self._pulse_action = None
            if abs_error <= self.config.pulse_turn_exit_deg or abs_predicted <= self.config.pulse_turn_exit_deg:
                self._latched_turn = None
                action = "forward"
                steering_reason = "aligned"
            else:
                desired_turn = "arc_right" if predicted_error > 0 else "arc_left"
                opposite_turn = "arc_left" if desired_turn == "arc_right" else "arc_right"
                yaw_correcting = yaw_rate is not None and error * yaw_rate > 0
                yaw_will_cover = yaw_correcting and abs(yaw_rate) * self.config.yaw_lookahead_s >= abs_error * 0.55

                if yaw_will_cover:
                    action = "observe"
                    steering_reason = "yaw rate already carrying turn"
                    display_action = self._latched_turn or desired_turn
                elif self._latched_turn == opposite_turn and abs_predicted < self.config.pulse_reverse_deg:
                    action = "observe"
                    steering_reason = "holding reversal until error is clear"
                    display_action = self._latched_turn or "hold"
                elif abs_predicted >= self.config.pulse_turn_enter_deg:
                    action = desired_turn
                    self._latched_turn = desired_turn
                    self._pulse_action = action
                    self._pulse_until = now + self.config.pulse_duration_s
                    self._pulse_observe_until = self._pulse_until + self.config.pulse_observe_s
                    steering_reason = "starting turn pulse"
                elif self._latched_turn is not None and abs_predicted > self.config.pulse_turn_exit_deg:
                    action = "observe"
                    steering_reason = "between pulse thresholds"
                    display_action = self._latched_turn
                else:
                    self._latched_turn = None
                    action = "forward"
                    steering_reason = "near aligned"

        left_us, right_us = self._action_to_pwm(action, level)
        display_action = display_action or action
        return WaypointControl(
            distance_m=dist,
            heading_error_deg=error,
            reached=False,
            left_us=left_us,
            right_us=right_us,
            action=action,
            metadata={
                **metadata,
                "behind_latch": self._behind_turn,
                "action": action,
                "display_action": display_action,
                "left_us": left_us,
                "right_us": right_us,
                "steering_reason": steering_reason,
                "level": level,
                "pulse_duration_s": self.config.pulse_duration_s,
                "pulse_observe_s": self.config.pulse_observe_s,
                "pulse_turn_enter_deg": self.config.pulse_turn_enter_deg,
                "pulse_turn_exit_deg": self.config.pulse_turn_exit_deg,
                "pulse_reverse_deg": self.config.pulse_reverse_deg,
            },
        )

    def _compute_smooth_pd_control(self, dist: float, error: float, imu_health: str, imu: dict) -> WaypointControl:
        reached = dist <= self.config.waypoint.reach_radius_m
        yaw_rate = self._valid_imu_yaw_rate(imu_health, imu)
        yaw_rate_f = 0.0 if yaw_rate is None else yaw_rate
        predicted_error = heading_error_deg(yaw_rate_f * self.config.yaw_lookahead_s, error)
        metadata = {
            "controller": "smooth_pd_v1",
            "raw_heading_error_deg": round(error, 2),
            "predicted_error_deg": round(predicted_error, 2),
            "yaw_rate_dps": None if yaw_rate is None else round(yaw_rate, 2),
            "imu_health": imu_health,
            "behind_latch": self._behind_turn,
            "pwm_slew_us_per_s": self.config.smooth_pwm_slew_us_per_s,
        }
        if reached:
            self._clear_controller_state()
            return WaypointControl(
                distance_m=dist,
                heading_error_deg=error,
                reached=True,
                left_us=self.config.neutral_us,
                right_us=self.config.neutral_us,
                action="reached",
                metadata={**metadata, "action": "reached", "display_action": "reached"},
            )

        action, display_action, controller_error, steering_reason = self._smooth_pd_action(error, predicted_error, yaw_rate_f)
        turn_effort = max(-1.0, min(1.0, self.config.smooth_kp * controller_error - self.config.smooth_kd * yaw_rate_f))
        if not action.startswith("behind_turn"):
            if abs(turn_effort) <= self.config.smooth_turn_deadband or abs(predicted_error) <= self.config.heading_deadband_deg:
                action = "forward"
                display_action = "forward"
                steering_reason = "small predicted/PD correction"
                turn_effort = 0.0
            elif turn_effort > 0.0:
                action = "turn_right"
                display_action = "turn right"
                steering_reason = "smooth right correction"
            else:
                action = "turn_left"
                display_action = "turn left"
                steering_reason = "smooth left correction"
        pivot_reverse = action.startswith("behind_turn")
        target_left, target_right = self._smooth_turn_to_pwm(
            turn_effort,
            pivot_reverse=pivot_reverse,
        )
        left_us, right_us = self._slew_smooth_pwm(target_left, target_right)
        return WaypointControl(
            distance_m=dist,
            heading_error_deg=error,
            reached=False,
            left_us=left_us,
            right_us=right_us,
            action=action,
            metadata={
                **metadata,
                "behind_latch": self._behind_turn,
                "action": action,
                "display_action": display_action,
                "left_us": left_us,
                "right_us": right_us,
                "target_left_us": round(target_left, 1),
                "target_right_us": round(target_right, 1),
                "turn_effort": round(turn_effort, 3),
                "controller_error_deg": round(controller_error, 2),
                "steering_reason": steering_reason,
                "smooth_kp": self.config.smooth_kp,
                "smooth_kd": self.config.smooth_kd,
                "smooth_turn_deadband": self.config.smooth_turn_deadband,
                "behind_enter_deg": self.config.behind_enter_deg,
                "behind_exit_deg": self.config.behind_exit_deg,
                "pivot_reverse": pivot_reverse,
                "reverse_level1_us": self.config.reverse_level1_us,
                "reverse_level2_us": self.config.reverse_level2_us,
                "reverse_level3_us": self.config.reverse_level3_us,
            },
        )

    def _smooth_pd_action(self, error: float, predicted_error: float, yaw_rate: float) -> tuple[str, str, float, str]:
        abs_error = abs(error)
        if self._behind_turn is not None:
            if abs_error <= self.config.behind_exit_deg:
                self._behind_turn = None
            else:
                sign = 1.0 if self._behind_turn == "right" else -1.0
                return (
                    f"behind_turn_{self._behind_turn}",
                    f"behind turn {self._behind_turn}",
                    sign * max(abs_error, self.config.behind_enter_deg),
                    "holding behind-target turn",
                )

        if abs_error >= self.config.behind_enter_deg:
            if abs(yaw_rate) > self.config.yaw_rate_deadband_dps:
                self._behind_turn = "right" if yaw_rate > 0 else "left"
            else:
                self._behind_turn = "right" if error > 0 else "left"
            sign = 1.0 if self._behind_turn == "right" else -1.0
            return (
                f"behind_turn_{self._behind_turn}",
                f"behind turn {self._behind_turn}",
                sign * abs_error,
                "entering behind-target turn",
            )

        self._behind_turn = None
        if abs(predicted_error) <= self.config.heading_deadband_deg:
            return "forward", "forward", error, "predicted alignment"
        if predicted_error > 0:
            return "turn_right", "turn right", error, "smooth right correction"
        return "turn_left", "turn left", error, "smooth left correction"

    def _smooth_turn_to_pwm(
        self,
        turn_effort: float,
        *,
        pivot_reverse: bool = False,
    ) -> tuple[float, float]:
        neutral = float(self.config.neutral_us)
        active = float(self.config.level3_us)
        span = max(0.0, active - neutral)
        if turn_effort > 0.0:
            inside = (
                float(self.config.reverse_level3_us)
                if pivot_reverse
                else active - min(1.0, turn_effort) * span
            )
            return active, inside
        if turn_effort < 0.0:
            inside = (
                float(self.config.reverse_level3_us)
                if pivot_reverse
                else active - min(1.0, abs(turn_effort)) * span
            )
            return inside, active
        return active, active

    def _slew_smooth_pwm(self, target_left: float, target_right: float) -> tuple[int, int]:
        now = time.monotonic()
        if self._smooth_last_at is None:
            dt = 1.0 / max(self.config.control_hz, 1.0)
        else:
            dt = max(0.0, now - self._smooth_last_at)
        self._smooth_last_at = now
        step = max(0.0, self.config.smooth_pwm_slew_us_per_s) * dt
        self._smooth_left_us = self._slew_value(self._smooth_left_us, target_left, step)
        self._smooth_right_us = self._slew_value(self._smooth_right_us, target_right, step)
        left = int(round(self._smooth_left_us))
        right = int(round(self._smooth_right_us))
        self._last_active_pwm = (left, right)
        return left, right

    @staticmethod
    def _slew_value(current: float, target: float, step: float) -> float:
        if step <= 0.0 or abs(target - current) <= step:
            return target
        return current + step if target > current else current - step

    def _clear_controller_state(self) -> None:
        self._pulse_action = None
        self._pulse_until = 0.0
        self._pulse_observe_until = 0.0
        self._latched_turn = None
        self._behind_turn = None
        self._smooth_left_us = float(self.config.neutral_us)
        self._smooth_right_us = float(self.config.neutral_us)
        self._smooth_last_at = None
        self._last_active_pwm = (self.config.neutral_us, self.config.neutral_us)

    def _clear_pulse_state(self) -> None:
        self._clear_controller_state()

    def _action_to_pwm(self, action: str, level: int) -> tuple[int, int]:
        neutral = int(self.config.neutral_us)
        if level >= 3:
            active = int(self.config.level3_us)
        elif level >= 2:
            active = int(self.config.level2_us)
        else:
            active = int(self.config.level1_us)
        if action == "coast":
            return neutral, neutral
        if action == "observe":
            return self._last_active_pwm
        if action.startswith("forward"):
            self._last_active_pwm = (active, active)
            return self._last_active_pwm
        if action.startswith("arc_right"):
            self._last_active_pwm = (active, neutral)
            return self._last_active_pwm
        if action.startswith("arc_left"):
            self._last_active_pwm = (neutral, active)
            return self._last_active_pwm
        return neutral, neutral

    def _set_status(self, status: dict) -> None:
        with self._lock:
            self._status = status

    def _base_status(self, state: str, reason: str) -> dict:
        with self._lock:
            total = len(self._waypoints)
            active = self._active_index
        return {
            "state": state,
            "reason": reason,
            "active_index": active,
            "total": total,
            "target": None,
            "distance_m": None,
            "bearing_deg": None,
            "heading_deg": None,
            "heading_error_deg": None,
            "controller": self.config.controller,
            "control_hz": self.config.control_hz,
        }

    def _status_for(
        self,
        state: str,
        reason: str,
        waypoint: GeoWaypoint,
        active_index: int,
        total: int,
        dist: float,
        bearing: float,
        error: float | None,
        heading: float | None,
        control_metadata: dict | None = None,
    ) -> dict:
        return {
            "state": state,
            "reason": reason,
            "active_index": active_index,
            "total": total,
            "target": asdict(waypoint),
            "distance_m": round(dist, 2),
            "bearing_deg": round(bearing, 1),
            "heading_deg": None if heading is None else round(heading, 1),
            "heading_error_deg": None if error is None else round(error, 1),
            "controller": self.config.controller,
            "control": control_metadata or {},
            "control_hz": self.config.control_hz,
        }

    @staticmethod
    def _coerce_waypoint(record: dict, index: int) -> GeoWaypoint:
        if not isinstance(record, dict):
            raise ValueError("waypoint must be an object")
        lat = float(record["lat"])
        lon = float(record["lon"])
        if not -90.0 <= lat <= 90.0:
            raise ValueError("waypoint lat must be -90..90")
        if not -180.0 <= lon <= 180.0:
            raise ValueError("waypoint lon must be -180..180")
        label = str(record.get("label") or f"WP {index + 1}")
        return GeoWaypoint(lat=lat, lon=lon, label=label)

    def _same_waypoint(self, left: GeoWaypoint, right: GeoWaypoint) -> bool:
        return distance_m(left.lat, left.lon, right.lat, right.lon) <= self.config.route_match_tolerance_m
