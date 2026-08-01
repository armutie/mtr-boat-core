# Seyond D1-R LiDAR

The bundled `ros2/seyond_mapping` package publishes the robotics version of the
Seyond D1-R as `/lidar/points`.

## Build

`dependencies.repos` pins the Seyond SDK and KISS-ICP inputs. From a clone at
`<workspace>/src/mtr-boat-core`, run:

```bash
MTR_WS=/absolute/path/to/workspace
"$MTR_WS/src/mtr-boat-core/scripts/bootstrap_ros2_workspace.sh" "$MTR_WS"
source "$MTR_WS/install/setup.bash"
```

The script imports and builds the pinned dependencies, the boat package, and
`seyond_mapping`.

## Run

LiDAR startup is opt-in:

```bash
ros2 launch mtr_boat_core sensors.launch.py enable_lidar:=true
```

For a LiDAR-only test:

```bash
ros2 launch mtr_boat_core sensors.launch.py \
  enable_gnss:=false enable_imu:=false enable_camera:=false \
  enable_lidar:=true
```

Default network parameters are in `config/ros/boat.example.yaml`.

## Point cloud contract

| Field | Value |
| --- | --- |
| Topic | `/lidar/points` |
| Type | `sensor_msgs/msg/PointCloud2` |
| Frame | `lidar_link` |
| QoS | Sensor data |
| Fields | `x`, `y`, `z`, `intensity`, `time` |
| Coordinates | X forward, Y left, Z up |
| Timestamp | Orange Pi acquisition time |

The driver converts the SDK's native X-up, Y-right, Z-forward coordinates to
the ROS convention.

The measured `base_link -> lidar_link` transform is off by default. Set
`publish_lidar_tf:=true` and provide the measured translation and rotation only
after the sensor is mounted.

See [`ros2/seyond_mapping/README.md`](../ros2/seyond_mapping/README.md) for the
mapping and live-view tools.
