from __future__ import annotations

import json
from dataclasses import dataclass, fields
from pathlib import Path

from .geo import EARTH_RADIUS_M, distance_m
from .nmea import GnssFix


GNSS_FIX_FIELDS = {field.name for field in fields(GnssFix)}


@dataclass
class GnssBounds:
    min_lat: float | None
    max_lat: float | None
    min_lon: float | None
    max_lon: float | None


@dataclass
class GnssReplaySummary:
    sample_count: int
    positioned_count: int
    duration_s: float
    average_hz: float
    first_lat: float | None
    first_lon: float | None
    last_lat: float | None
    last_lon: float | None
    bounds: GnssBounds
    distance_m: float
    speed_min_mps: float | None
    speed_max_mps: float | None
    speed_avg_mps: float | None
    heading_min_deg: float | None
    heading_max_deg: float | None
    satellites_min: int | None
    satellites_max: int | None
    hdop_min: float | None
    hdop_max: float | None
    fix_counts: dict[str, int]


def load_gnss_log(path: str | Path) -> list[GnssFix]:
    fixes: list[GnssFix] = []
    with Path(path).open("r", encoding="utf-8-sig") as file:
        for line in file:
            line = line.strip()
            if not line:
                continue
            record = json.loads(line)
            fix_record = {key: value for key, value in record.items() if key in GNSS_FIX_FIELDS}
            fixes.append(GnssFix(**fix_record))
    fixes.sort(key=lambda fix: fix.timestamp)
    return fixes


def positioned_fixes(fixes: list[GnssFix]) -> list[GnssFix]:
    return [fix for fix in fixes if fix.lat is not None and fix.lon is not None]


def total_distance_m(fixes: list[GnssFix]) -> float:
    total = 0.0
    previous: GnssFix | None = None
    for fix in positioned_fixes(fixes):
        if previous is not None:
            total += distance_m(previous.lat, previous.lon, fix.lat, fix.lon)
        previous = fix
    return total


def _range(values):
    values = [value for value in values if value is not None]
    if not values:
        return None, None
    return min(values), max(values)


def _average(values) -> float | None:
    values = [value for value in values if value is not None]
    return sum(values) / len(values) if values else None


def summarize_gnss_log(fixes: list[GnssFix]) -> GnssReplaySummary:
    if not fixes:
        return GnssReplaySummary(
            sample_count=0,
            positioned_count=0,
            duration_s=0.0,
            average_hz=0.0,
            first_lat=None,
            first_lon=None,
            last_lat=None,
            last_lon=None,
            bounds=GnssBounds(None, None, None, None),
            distance_m=0.0,
            speed_min_mps=None,
            speed_max_mps=None,
            speed_avg_mps=None,
            heading_min_deg=None,
            heading_max_deg=None,
            satellites_min=None,
            satellites_max=None,
            hdop_min=None,
            hdop_max=None,
            fix_counts={},
        )

    positioned = positioned_fixes(fixes)
    duration = max(0.0, fixes[-1].timestamp - fixes[0].timestamp)
    average_hz = (len(fixes) - 1) / duration if duration > 0.0 and len(fixes) > 1 else 0.0
    lats = [fix.lat for fix in positioned]
    lons = [fix.lon for fix in positioned]
    speed_min, speed_max = _range(fix.speed_mps for fix in fixes)
    heading_min, heading_max = _range(fix.heading_deg for fix in fixes)
    satellites_min, satellites_max = _range(fix.satellites for fix in fixes)
    hdop_min, hdop_max = _range(fix.hdop for fix in fixes)
    fix_counts: dict[str, int] = {}
    for fix in fixes:
        fix_counts[fix.fix] = fix_counts.get(fix.fix, 0) + 1

    first = positioned[0] if positioned else None
    last = positioned[-1] if positioned else None
    return GnssReplaySummary(
        sample_count=len(fixes),
        positioned_count=len(positioned),
        duration_s=duration,
        average_hz=average_hz,
        first_lat=None if first is None else first.lat,
        first_lon=None if first is None else first.lon,
        last_lat=None if last is None else last.lat,
        last_lon=None if last is None else last.lon,
        bounds=GnssBounds(
            min_lat=min(lats) if lats else None,
            max_lat=max(lats) if lats else None,
            min_lon=min(lons) if lons else None,
            max_lon=max(lons) if lons else None,
        ),
        distance_m=total_distance_m(fixes),
        speed_min_mps=speed_min,
        speed_max_mps=speed_max,
        speed_avg_mps=_average(fix.speed_mps for fix in fixes),
        heading_min_deg=heading_min,
        heading_max_deg=heading_max,
        satellites_min=satellites_min,
        satellites_max=satellites_max,
        hdop_min=hdop_min,
        hdop_max=hdop_max,
        fix_counts=fix_counts,
    )
