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

Open `http://localhost:8080` on the boat computer. The viewer shows incoming
FPS, point count, and range. Its indicator turns green only after a point cloud
arrives; if no clouds arrive for one second it displays a STALE warning. Drag
to look around, use WASD to move, and select Distance or Height coloring to
inspect the scene. Pause explicitly freezes the display.

The standalone viewer launch publishes `/seyond/points`. From the repository
root, after sourcing `/opt/ros/humble/setup.bash`, check its stream with:

```bash
python3 scripts/check_lidar_health.py --topic /seyond/points --duration 30
```

For the boat launch or an isolated driver remapped to `/lidar/points`, omit
`--topic`. PASS requires at least 8 Hz, no gap exceeding 0.5 seconds, consistent
message dimensions/buffer sizes, increasing timestamps, and finite sampled
XYZ coordinates. This checks stream health, not physical mounting/calibration
or every individual point.

The point-cloud conversion filters once into a buffer sized for the input
point count, then shrinks the buffer. The previous count/write passes evaluated
the range with different floating-point addition orders; points near a range
cutoff could be rejected while counting and accepted while writing, causing
a heap-buffer overflow. `test_point_cloud` includes a reproducer for that case.

The driver launch paths also load `config/fastdds.xml`, which increases its
Fast DDS shared-memory segment from 512 KB to 8 MB for large clouds. The
profile applies to the driver process; it does not change system-wide
network or memory settings. Direct executable launches should set
`FASTRTPS_DEFAULT_PROFILES_FILE` to that file's absolute path.

This package is Apache-2.0 licensed. Seyond's SDK and KISS-ICP retain their
respective upstream licenses.
