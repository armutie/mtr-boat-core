# seyond_mapping

This bundled ROS 2 package provides the Seyond Hummingbird D1-R PointCloud2
driver, KISS-ICP launch integration, voxelized map accumulation with PCD/PLY
saving, and the optional first-person live WebGL viewer.

It is separate from Seyond's SDK targets and does not modify the original SDK
demo. The repository-level `dependencies.repos` pins the SDK and KISS-ICP
commits, while `scripts/bootstrap_ros2_workspace.sh` builds the complete
workspace.

The primary target is Ubuntu 22.04 ARM64 with ROS 2 Humble on the Orange Pi 5
Plus. The driver publishes REP-103 `sensor_msgs/msg/PointCloud2` data on its
`points` output; the boat launch remaps this to `/lidar/points`.

For standalone mapping after building and sourcing the workspace:

```bash
ros2 launch seyond_mapping handheld_mapping.launch.py
ros2 service call /mapping/save_map std_srvs/srv/Trigger '{}'
```

The save service writes `maps/handheld_map.pcd` and
`maps/handheld_map.ply`. For a raw browser view without mapping:

```bash
ros2 launch seyond_mapping live_view.launch.py
```

This package is Apache-2.0 licensed. Seyond's SDK and KISS-ICP retain their
respective upstream licenses.
