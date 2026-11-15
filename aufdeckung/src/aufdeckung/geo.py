"""Frame conversions: local ENU (cage centre origin) <-> PX4 NED <-> WGS84.

The cage is 45 m across, so a local tangent-plane (equirectangular) projection
is accurate to well below a millimetre here.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class GeoOrigin:
    lat: float
    lon: float
    alt: float = 0.0

    def __post_init__(self) -> None:
        if not -90.0 < self.lat < 90.0 or not -180.0 <= self.lon <= 180.0:
            raise ValueError("origin latitude/longitude out of range")

    @property
    def m_per_deg_lat(self) -> float:
        phi = math.radians(self.lat)
        return 111132.92 - 559.82 * math.cos(2 * phi) + 1.175 * math.cos(4 * phi)

    @property
    def m_per_deg_lon(self) -> float:
        phi = math.radians(self.lat)
        return 111412.84 * math.cos(phi) - 93.5 * math.cos(3 * phi)

    def enu_to_wgs84(self, x: float, y: float, z: float = 0.0) -> tuple[float, float, float]:
        return (
            self.lat + y / self.m_per_deg_lat,
            self.lon + x / self.m_per_deg_lon,
            self.alt + z,
        )

    def wgs84_to_enu(self, lat: float, lon: float, alt: float | None = None) -> tuple[float, float, float]:
        z = 0.0 if alt is None else alt - self.alt
        return (
            (lon - self.lon) * self.m_per_deg_lon,
            (lat - self.lat) * self.m_per_deg_lat,
            z,
        )


def enu_to_ned(x: float, y: float, z: float) -> tuple[float, float, float]:
    """ENU (east, north, up) -> NED (north, east, down)."""
    return y, x, -z


def ned_to_enu(n: float, e: float, d: float) -> tuple[float, float, float]:
    return e, n, -d


def enu_yaw_to_ned_deg(yaw_enu_rad: float) -> float:
    """ENU yaw (0 = east, CCW) -> NED heading in degrees (0 = north, CW), in [0, 360)."""
    return (90.0 - math.degrees(yaw_enu_rad)) % 360.0
