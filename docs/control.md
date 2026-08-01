# ROS 2 control

The dashboard sends routes and operator intent through ROS. The autonomy node
owns automatic decisions:

```text
dashboard -> cmd_vel/operator ----------------------\
dashboard -> autonomy/route -> autonomy -> cmd_vel/auto -> supervisor -> ESP32
```

The supervisor supports `off`, `manual`, and `auto`. In auto mode, a fresh
operator command is added as a temporary correction; this lets future steering
and throttle controls correct autonomy without changing the architecture.

## Start

Build and source the workspace, then start the complete software runtime:

```bash
ros2 launch mtr_boat_core boat.launch.py \
  params_file:="$PARAMS"
```

The thruster node is off by default. Inspect the final command with:

```bash
ros2 topic echo /cmd_vel
```

Only after checking the dual-thruster firmware, ESC neutral, and physical
cutoff, start the serial owner:

```bash
ros2 launch mtr_boat_core boat.launch.py \
  params_file:="$PARAMS" enable_thruster:=true
```

Use `web_dashboard --direct-control` only for an isolated legacy bench test,
never alongside `thruster_node`.
