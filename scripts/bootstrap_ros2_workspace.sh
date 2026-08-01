#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 /absolute/path/to/ros_workspace" >&2
  exit 2
fi

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="$1"

if [[ "${workspace_dir}" != /* ]]; then
  echo "Workspace path must be absolute: ${workspace_dir}" >&2
  exit 2
fi

expected_repo_dir="${workspace_dir}/src/mtr-boat-core"
if [[ "${repo_dir}" != "${expected_repo_dir}" ]]; then
  echo "Clone this repository at ${expected_repo_dir}, then rerun the script." >&2
  exit 2
fi

for command_name in vcs colcon cmake; do
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "Missing required command: ${command_name}" >&2
    exit 1
  fi
done

ros_setup="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ ! -f "${ros_setup}" ]]; then
  echo "ROS setup file not found: ${ros_setup}" >&2
  exit 1
fi

mkdir -p "${workspace_dir}/src"
vcs import "${workspace_dir}/src" < "${repo_dir}/dependencies.repos"

sdk_dir="${workspace_dir}/src/inno-lidar-sdk"
(
  cd "${sdk_dir}/build"
  ./build_unix.sh
)

for sdk_library in innolidarsdkclient innolidarsdkcommon innolidarutils; do
  if [[ ! -f "${sdk_dir}/lib/lib${sdk_library}.a" ]]; then
    echo "SDK build did not produce lib${sdk_library}.a" >&2
    exit 1
  fi
done

set +u
source "${ros_setup}"
set -u

(
  cd "${workspace_dir}"
  colcon build \
    --base-paths \
      "${repo_dir}" \
      "${repo_dir}/ros2/seyond_mapping" \
      "${workspace_dir}/src/kiss-icp" \
    --symlink-install \
    --cmake-args \
      -DSDK_ROOT="${sdk_dir}" \
      -DPython3_EXECUTABLE=/usr/bin/python3
)

echo "Build complete. Run: source ${workspace_dir}/install/setup.bash"
