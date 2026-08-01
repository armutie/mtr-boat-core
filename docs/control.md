# ROS 2 control

The dashboard can send both manual and its existing automatic commands through
one ROS control path:

```text
dashboard -> cmd_vel/operator --\
                                control supervisor -> cmd_vel -> thruster node -> ESP32
dashboard -> cmd_vel/auto -----/
```

The supervisor supports `off`, `manual`, and `auto`. In auto mode, a fresh
operator command is added as a temporary correction; this lets future steering
and throttle controls correct autonomy without changing the architecture.

## Start

Build and source the workspace, then use separate terminals:

```bash
ros2 launch mtr_boat_core control.launch.py \
  params_file:="$PARAMS"

python3 web_dashboard/server.py \
  --config config/boat.local.json \
  --ros-control
```

That is a safe software-only test: the thruster node is off. Inspect the final
command with:

```bash
ros2 topic echo /cmd_vel
```

Only after checking the dual-thruster firmware, ESC neutral, and physical
cutoff, start the serial owner:

```bash
ros2 launch mtr_boat_core control.launch.py \
  params_file:="$PARAMS" enable_thruster:=true
```

Do not run the legacy direct-serial dashboard at the same time as
`thruster_node`.
