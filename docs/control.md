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
so its commands are not converted to velocity and back. In auto mode, a fresh
operator command can still be applied as a temporary correction.

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

Use `web_dashboard --direct-control` only for an isolated legacy bench test,
never alongside `thruster_node`.

Each thruster-node process publishes a unique session ID. A new session forces
the control supervisor to `off`, and the thruster node independently keeps its
output neutral until it observes `off` followed by a deliberate `manual` or
`auto` selection. Consequently, automatic process respawn restores the serial
connection without automatically restoring permission to move.
