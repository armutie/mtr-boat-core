from __future__ import annotations

from typing import Iterable

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


POINT_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="doppler", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="snr_raw", offset=16, datatype=PointField.FLOAT32, count=1),
    PointField(name="noise_raw", offset=20, datatype=PointField.FLOAT32, count=1),
]


def point_rows(points: Iterable[dict]) -> list[tuple[float, float, float, float, float, float]]:
    rows = []
    for point in points:
        rows.append(
            (
                float(point.get("x", 0.0)),
                float(point.get("y", 0.0)),
                float(point.get("z", 0.0)),
                float(point.get("doppler", 0.0)),
                float(point.get("snr_raw", 0.0)),
                float(point.get("noise_raw", 0.0)),
            )
        )
    return rows


def make_header(stamp, frame_id: str) -> Header:
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    return header


def points_to_cloud(points: list[dict], stamp, frame_id: str) -> PointCloud2:
    return point_cloud2.create_cloud(
        header=make_header(stamp, frame_id),
        fields=POINT_FIELDS,
        points=point_rows(points),
    )


def cloud_to_points(cloud: PointCloud2) -> list[dict]:
    points = []
    for row in point_cloud2.read_points(
        cloud,
        field_names=("x", "y", "z", "doppler", "snr_raw", "noise_raw"),
        skip_nans=True,
    ):
        points.append(
            {
                "x": float(row[0]),
                "y": float(row[1]),
                "z": float(row[2]),
                "doppler": float(row[3]),
                "snr_raw": float(row[4]),
                "noise_raw": float(row[5]),
            }
        )
    return points
