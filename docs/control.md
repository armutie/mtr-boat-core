# ROS 2 control

The dashboard sends routes and operator intent through ROS. The autonomy node
owns automatic decisions:

```text
dashboard -> cmd_vel/operator ----------------------\
dashboard -> autonomy/route -> autonomy -> thrusters/auto -> supervisor
                                                        -> thrusters/command -> ESP32
```

The supervisor supports `off`, `manual`, and `auto`. Manual velocity intent is
mapped to PWM once. Autonomy publishes the exact left/right PWM it calculates,
so its commands are not converted to velocity and back. In Auto, holding the
dashboard wheel temporarily replaces Auto steering while Auto retains throttle.

## Start

Build and source the workspace, then start the complete software runtime:

```bash
ros2 launch mtr_boat_core boat.launch.py \
  params_file:="$PARAMS"
```

The thruster node is off by default. Inspect the final command with:

```bash
ros2 topic echo /thrusters/command
```

Only after checking the dual-thruster firmware, ESC neutral, and physical
cutoff, start the serial owner:

```bash
ros2 launch mtr_boat_core boat.launch.py \
  params_file:="$PARAMS" enable_thruster:=true
```

The node publishes the ESP32 response to each serial command on
`/thrusters/status`. A healthy neutral acknowledgement looks like:

```text
data: OK L1500 R1500
```

Inspect it with:

```bash
ros2 topic echo /thrusters/status
```

## Calibrated pivot turns

The measured PWM levels are:

| Level | Forward | Reverse |
| --- | ---: | ---: |
| 1 | 1565 | 1460 |
| 2 | 1575 | 1445 |
| 3 | 1650 | 1425 |

Ordinary steering remains forward differential steering. Above 75% steering
lock, the inside thruster blends through neutral into reverse while the outside
thruster increases. Zero throttle always produces `L1500 R1500`. Crossing an
individual channel between forward and reverse holds it at neutral for 0.2 s.

Auto uses the full measured reverse level only for its latched behind-target
turn; ordinary heading corrections remain forward-only.

Use `web_dashboard --direct-control` only for an isolated legacy bench test,
never alongside `thruster_node`.

Each thruster-node process publishes a unique session ID. A new session forces
the control supervisor to `off`, and the thruster node independently keeps its
output neutral until it observes `off` followed by a deliberate `manual` or
`auto` selection. Consequently, automatic process respawn restores the serial
connection without automatically restoring permission to move.
