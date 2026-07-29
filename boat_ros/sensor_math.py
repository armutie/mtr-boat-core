from __future__ import annotations

import math


STANDARD_GRAVITY_MPS2 = 9.80665


def acceleration_g_to_mps2(value_g: float) -> float:
    """Convert acceleration in standard gravity units to metres per second squared."""

    return value_g * STANDARD_GRAVITY_MPS2


def angular_velocity_dps_to_rad_s(value_dps: float) -> float:
    """Convert angular velocity in degrees per second to radians per second."""

    return math.radians(value_dps)
