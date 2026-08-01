# Camera

The camera node publishes the Arducam UVC feed as `/camera/image_raw` and
serves a low-latency browser stream. The default capture profile is MJPEG,
1280x720, 30 FPS.

## Stable device name

Install all included MTR hardware rules once:

```bash
sudo ./scripts/install_udev_rules.sh
ls -l /dev/mtr_camera
```

The rule targets the tested Arducam UC684. Update it if the camera is replaced
with a unit that has different USB identifiers.

## Run

The camera is enabled by default in `sensors.launch.py`. For a camera-only
test:

```bash
ros2 launch mtr_boat_core sensors.launch.py \
  enable_gnss:=false enable_imu:=false
```

Open the viewer from a device on the same network:

```text
http://<orange-pi-ip>:8081/
```

Useful endpoints:

- `/stream.mjpg`
- `/snapshot.jpg`
- `/health`

The camera reconnects automatically when `/dev/mtr_camera` returns. Set
`enable_web: false` in the ROS YAML if only `/camera/image_raw` is needed.
`web_rotation_deg` rotates the browser view without changing the ROS image.

The viewer listens on `0.0.0.0:8081`, has no authentication, and permits CORS.
Use it only on a trusted boat network.

## Frames and calibration

The ROS image uses `camera_optical_frame`: X right, Y down, Z forward. The
fixed `camera_link -> camera_optical_frame` axis transform is published while
the camera is enabled.

The measured `base_link -> camera_link` transform is off by default. Enable it
with `publish_camera_tf:=true` only after measuring the installed pose.

No placeholder `/camera/camera_info` is published. Add it after calibrating the
real lens at the selected resolution.
