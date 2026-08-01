# ROS 2 foundation

The boat uses ROS 2 as a small integration layer around hardware drivers and
testable Python libraries. New nodes and topics should be added only when they
create a real hardware, safety, or computation boundary.

## Initial sensor graph

```text
GNSS driver ------> /gnss/fix          sensor_msgs/NavSatFix
IMU driver -------> /imu/data_raw      sensor_msgs/Imu
camera driver ----> /camera/image_raw  sensor_msgs/Image
                 -> /camera/camera_info
LiDAR driver -----> /lidar/points      sensor_msgs/PointCloud2
```

GNSS, IMU, and camera are implemented in `boat_ros`. The LiDAR entry defines
the interface its existing driver should meet when added.

## Camera contract

The Arducam UVC integration owns the stable `/dev/mtr_camera` udev alias in
`camera_node` and exposes:

```text
topic:       /camera/image_raw
type:        sensor_msgs/msg/Image
frame:       camera_optical_frame
QoS:         sensor-data (best effort, volatile)
encoding:    bgr8
timestamp:   Orange Pi acquisition time
web viewer:  http://<orange-pi-ip>:8081/
```

The default capture profile is hardware MJPEG at 1280x720 and 30 FPS.
GStreamer sends those native JPEG frames directly to browser clients without a
second encode. A separate latest-frame worker decodes the standard ROS image,
so slow perception or DDS consumers cannot queue stale operator video. Depth
estimation remains a separate consumer so it cannot add latency to the
operator stream.

The browser server can be disabled independently with `enable_web: false`, and
a bind failure does not stop ROS image publication. Its default `0.0.0.0:8081`
endpoint is unauthenticated and uses permissive CORS, so it is for a trusted
boat LAN only.

The image frame follows the ROS optical convention: X right, Y down, Z forward.
`sensors.launch.py` always publishes the fixed
`camera_link -> camera_optical_frame` axis transform while the camera is
enabled. The configurable `base_link -> camera_link` mounting transform is
disabled by default and requires `publish_camera_tf:=true`. Set `camera_x`,
`camera_y`, `camera_z`, `camera_roll`, `camera_pitch`, and `camera_yaw` only
after measuring the installed pose.

No placeholder calibration is published. Add `/camera/camera_info` only after
calibrating the real lens at the selected resolution.

## Design rules

- Prefer standard ROS messages over JSON or project-specific messages.
- Keep hardware access in one owning node per device.
- Keep algorithms in ordinary Python modules when they do not need ROS.
- Use topics for continuous data, services for short state changes, and actions
  for missions that take time.
- Use the sensor-data QoS profile for high-rate sensor streams.
- Timestamp data at acquisition and give every physical sensor a frame ID.
- Do not send actuator output directly from the dashboard or perception nodes.
- Preserve the ESP32 timeout-to-neutral behavior and physical power cutoff.

## Expected control boundary

```text
operator/autonomy request
          |
          v
   command + safety node
          |
          v
     thruster driver
          |
          v
         ESP32
```

Only the thruster driver may open the ESP32 serial port. The command and safety
node will eventually reject stale, disarmed, or unhealthy requests before they
reach that driver.

## Orange Pi constraints

- Use a supported 64-bit Ubuntu and matching ROS 2 LTS distribution.
- Install the ROS base variant; run visualization on a laptop.
- Avoid forwarding full-rate image and point-cloud streams over Wi-Fi by
  default.
- Measure CPU load, memory, temperature, and storage use with all sensors
  running before choosing camera resolution or perception rates.
