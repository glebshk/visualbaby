"""Lagebild (situation picture) messages.

The official MQTT/JSON schema is only published for Teamphase 2 (31.01.2027),
so this is a placeholder with the information the Regelwerk asks for: object
class and georeference. Adapt :func:`detection_message` once the schema is out.
"""

from __future__ import annotations

from datetime import datetime, timezone
from typing import Any

from aufdeckung.config import ArenaConfig
from aufdeckung.geo import GeoOrigin

SCHEMA_VERSION = "placeholder-0.1"


def detection_message(cls: str, x: float, y: float, arena: ArenaConfig, t_mission: float,
                      confidence: float = 1.0, team: str = "team", object_id: str = "",
                      z: float = 0.0) -> dict[str, Any]:
    origin = GeoOrigin(arena.origin_lat, arena.origin_lon, arena.origin_alt)
    lat, lon, alt = origin.enu_to_wgs84(x, y, z)
    if not 0.0 <= confidence <= 1.0:
        raise ValueError("confidence must be in [0, 1]")
    return {
        "schema": SCHEMA_VERSION,
        "team": team,
        "id": object_id,
        "class": cls,
        "confidence": round(confidence, 3),
        "position": {"lat": round(lat, 8), "lon": round(lon, 8), "alt": round(alt, 2)},
        "local_enu": {"x": round(x, 3), "y": round(y, 3), "z": round(z, 3)},
        "mission_time_s": round(t_mission, 2),
        "timestamp": datetime.now(timezone.utc).isoformat(),
    }
