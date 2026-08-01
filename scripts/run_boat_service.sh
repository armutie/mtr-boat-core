#!/usr/bin/env bash
# ROS 2 Humble's generated setup scripts read optional variables before they
# are defined, so nounset (-u) cannot be enabled while sourcing them.
set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/.." && pwd)"
ros_setup="/opt/ros/humble/setup.bash"
workspace_setup="${repo_root}/install/setup.bash"

if [[ ! -f "${ros_setup}" ]]; then
  echo "ROS 2 Humble setup not found: ${ros_setup}" >&2
  exit 1
fi

if [[ ! -f "${workspace_setup}" ]]; then
  echo "Workspace has not been built: ${workspace_setup}" >&2
  echo "Run: cd ${repo_root} && colcon build --symlink-install" >&2
  exit 1
fi

source "${ros_setup}"
source "${workspace_setup}"

# Start the ESP32 serial owner so the dashboard is ready for operation. The
# control supervisor and dashboard both initialize in off mode, which sends
# neutral 1500/1500 until an operator explicitly selects manual or auto.
exec ros2 launch mtr_boat_core boat.launch.py enable_thruster:=true
