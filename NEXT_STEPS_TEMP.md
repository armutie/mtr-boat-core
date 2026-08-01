# MTR Boat Next Steps

Temporary working plan. Do not treat this as final architecture.

## Done (remove from active backlog)

- **Manual control on dashboard** — wheel, throttle, stop, mode pills, command staleness, actuator bridge @ 20 Hz, dry-run/live ESP32 path.
- **Auto mission planner UI** — map (OSM + overzoom), add/move pins, route line, GNSS boat marker + track, Follow/Fit, health chips, auto status readouts.
- **Auto backend v1** — `boat_core/autonomy.py` behind
  `boat_ros/autonomy_node.py`; the dashboard publishes routes and the node
  publishes `/cmd_vel/auto`.
- **UI/server mode split** — `surfaceMode` (planner vs wheel) separate from server drive mode; auto planner stays open even when arm preconditions fail.
- **Offline helpers** — `scripts/test_auto_waypoint.py`, `scripts/test_gnss_imu_heading.py`, `scripts/test_auto_guesstimate.py`.
- **Waypoint input format (for now)** — map pin click + localStorage route persistence.

## Next up — IMU heading when stationary (important)

**Reminder:** Auto arm/drive currently requires GNSS **course while moving** (`speed_mps >= 0.3`, `heading_deg` present). At standstill or slow troll it blocks even if position fix is good.

**Implement:**

- When `speed_mps < min_speed_for_course_mps`, use **IMU yaw delta anchored to last good GNSS course** instead of raw `heading_deg`.
- Anchor GNSS course when speed is above threshold; carry heading with `imu.yaw_relative_deg` (or integrated gyro-Z) between anchors.
- Allow **arming on fix alone** for planner use; only require fused heading when actually issuing drive commands (optional split if arm still feels too strict).
- Log/disagreement guard: if GNSS course and IMU-carried heading diverge beyond a threshold, block auto and show reason.

**Use data from:** `scripts/test_gnss_imu_heading.py` — pick anchor speed threshold, jitter-aware reach radius, and max GNSS–IMU disagreement from recorded phases.

**Files likely touched:** `boat_core/autonomy.py` and
`boat_ros/autonomy_node.py`.

## Auto mode — still rough (after IMU heading)

- Tune guesstimate PWM levels from bench/lake logs (1565/1575 only today).
- Set `reach_radius_m` from stationary GNSS jitter (must be larger than jitter radius).
- Consider lowering `min_speed_for_course_mps` only after heading fusion exists — not before.
- First lake test: short waypoint, open water, manual push to get moving, then arm auto.
- No radar constraint yet — stay in open water.

## 4. Radar obstacle avoidance into auto (not started)

Goal: waypoint command = desired motion; radar = constraint.

- Blend or min-throttle with existing `radar_nav` pipeline output (`blend_waypoint_and_avoidance` in `radar_nav/sim.py` is the reference).
- Dashboard should show when radar deflected/slowed auto.
- False returns must not permanently poison state.

## 1. Prove dashboard over Wi-Fi (status unknown)

- Test Pi on Wi-Fi / hotspot, bind `server.py` on LAN, open from phone/laptop.
- Document IP discovery command and failure modes when Wi-Fi drops.

## Architecture decision

- ROS 2 is the long-term runtime and sensor integration layer.
- Keep the Python dashboard as an operator interface, not the owner of hardware
  drivers or actuator serial output.

## Open decisions

- Mission interface beyond the current transient route topic.
- Phone-first vs laptop-first operator UI polish.
