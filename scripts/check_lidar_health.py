#!/usr/bin/env python3
"""Check live PointCloud2 continuity, layout, and sampled finite coordinates."""

import argparse
import math
import struct
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/lidar/points")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--min-rate", type=float, default=8.0)
    parser.add_argument("--max-gap", type=float, default=0.5)
    args = parser.parse_args()
    if args.duration <= 0 or args.max_gap <= 0 or args.min_rate < 0:
        parser.error("duration/max-gap must be positive and min-rate nonnegative")

    rclpy.init()
    node = rclpy.create_node("lidar_health_check")
    arrivals, counts, errors, frames = [], [], [], set()
    last_stamp = None

    def on_cloud(cloud):
        nonlocal last_stamp
        arrivals.append(time.monotonic())
        count = cloud.width * cloud.height
        counts.append(count)
        frames.add(cloud.header.frame_id)
        offsets = {f.name: f.offset for f in cloud.fields
                   if f.datatype == PointField.FLOAT32 and f.count == 1}
        if not count or not cloud.header.frame_id:
            errors.append("empty cloud or frame ID")
        if (cloud.point_step == 0 or cloud.row_step < cloud.width * cloud.point_step
                or len(cloud.data) != cloud.row_step * cloud.height):
            errors.append("inconsistent cloud dimensions/buffer")
            return
        if any(name not in offsets or offsets[name] + 4 > cloud.point_step
               for name in ("x", "y", "z")):
            errors.append("missing/invalid FLOAT32 xyz fields")
            return
        stamp = cloud.header.stamp.sec * 1000000000 + cloud.header.stamp.nanosec
        if last_stamp is not None and stamp <= last_stamp:
            errors.append("non-increasing timestamp")
        last_stamp = stamp
        endian = ">" if cloud.is_bigendian else "<"
        # Sample throughout each cloud without adding a large CPU load.
        for index in range(0, count, max(1, count // 64)):
            row, column = divmod(index, cloud.width)
            base = row * cloud.row_step + column * cloud.point_step
            if not all(math.isfinite(struct.unpack_from(
                    endian + "f", cloud.data, base + offsets[name])[0])
                    for name in ("x", "y", "z")):
                errors.append("nonfinite sampled coordinates")
                break

    subscription = node.create_subscription(
        PointCloud2, args.topic, on_cloud, qos_profile_sensor_data)
    print(f"Checking {args.topic} for {args.duration:g}s (discovery allowed 10s)", flush=True)
    discovery_deadline = time.monotonic() + 10
    end, next_report = None, None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            current = time.monotonic()
            if arrivals and end is None:
                end = arrivals[0] + args.duration
                next_report = arrivals[0] + 10
            if end is None:
                if current >= discovery_deadline:
                    errors.append("no point clouds received")
                    break
                continue
            if current >= next_report:
                print(f"{len(arrivals)} clouds; latest={counts[-1]} points; "
                      f"data age={current-arrivals[-1]:.3f}s", flush=True)
                next_report += 10
            if current >= end:
                break
        elapsed = arrivals[-1] - arrivals[0] if len(arrivals) > 1 else 0
        rate = (len(arrivals) - 1) / elapsed if elapsed else 0
        gaps = [b - a for a, b in zip(arrivals, arrivals[1:])]
        largest_gap = max(gaps + ([time.monotonic() - arrivals[-1]] if arrivals else [0]))
        if rate < args.min_rate:
            errors.append(f"rate below {args.min_rate:g} Hz")
        if largest_gap > args.max_gap:
            errors.append(f"gap exceeds {args.max_gap:g}s")
        print(f"{'FAIL' if errors else 'PASS'}: {len(arrivals)} clouds, {rate:.2f} Hz, "
              f"max gap {largest_gap:.3f}s, points {min(counts, default=0)}.."
              f"{max(counts, default=0)}, frames={','.join(sorted(frames))}", flush=True)
        for error in sorted(set(errors)):
            print(f"  {error}", flush=True)
        return 1 if errors else 0
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
