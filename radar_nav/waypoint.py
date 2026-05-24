from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class GeoWaypoint:
    lat: float
    lon: float
    label: str = ""


@dataclass
class WaypointNavConfig:
    reach_radius_m: float = 2.0
    approach_slow_radius_m: float = 8.0


@dataclass
class WaypointControl:
    distance_m: float
    heading_error_deg: float
    reached: bool
    left_us: int
    right_us: int
    action: str
    metadata: dict = field(default_factory=dict)
