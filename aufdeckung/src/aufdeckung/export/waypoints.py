"""Waypoint export for PX4: NED list for the ROS bridge and a QGroundControl .plan."""

from __future__ import annotations

import json
from collections.abc import Sequence
from pathlib import Path
from typing import Any

from aufdeckung.config import ArenaConfig, FlightConfig
from aufdeckung.geo import GeoOrigin, enu_to_ned
from aufdeckung.planning.coverage_planner import Waypoint


def to_ned_setpoints(wps: Sequence[Waypoint], home: tuple[float, float], yaw_deg: float = 90.0
                     ) -> list[dict[str, Any]]:
    """Setpoints relative to the PX4 home (take-off) position.

    Field meaning matches ``/drone/in/offboard/pos/ned`` (``ros_msg/Vector4``) as
    consumed by ``px4_to_ros2``: x = north [m], y = east [m], z = altitude *up*
    [m] (the bridge negates it), w = yaw [deg]. ENU yaw 0 (camera x-axis east)
    is NED yaw 90 deg.
    """
    out: list[dict[str, Any]] = []
    for w in wps:
        n, e, _ = enu_to_ned(w.x - home[0], w.y - home[1], w.z)
        out.append({"x": round(n, 3), "y": round(e, 3), "z": round(w.z, 3), "w": yaw_deg,
                    "kind": w.kind})
    return out


def to_qgc_plan(wps: Sequence[Waypoint], arena: ArenaConfig, flight: FlightConfig,
                home: tuple[float, float]) -> dict[str, Any]:
    """QGroundControl mission (.plan); altitudes relative to home (MAV_FRAME_GLOBAL_RELATIVE_ALT)."""
    origin = GeoOrigin(arena.origin_lat, arena.origin_lon, arena.origin_alt)
    hlat, hlon, halt = origin.enu_to_wgs84(home[0], home[1], 0.0)
    items: list[dict[str, Any]] = []
    for i, w in enumerate(wps, start=1):
        lat, lon, _ = origin.enu_to_wgs84(w.x, w.y, w.z)
        items.append({
            "type": "SimpleItem", "command": 16, "frame": 3, "autoContinue": True, "doJumpId": i,
            "params": [0, flight.waypoint_tolerance, 0, None, round(lat, 8), round(lon, 8),
                       round(w.z, 2)],
            "AMSLAltAboveTerrain": None, "Altitude": round(w.z, 2), "AltitudeMode": 1,
        })
    return {
        "fileType": "Plan", "version": 1, "groundStation": "QGroundControl",
        "mission": {"version": 2, "firmwareType": 12, "vehicleType": 2,
                    "cruiseSpeed": flight.max_speed, "hoverSpeed": flight.max_speed,
                    "globalPlanAltitudeMode": 1,
                    "plannedHomePosition": [round(hlat, 8), round(hlon, 8), round(halt, 2)],
                    "items": items},
        "geoFence": {"version": 2, "circles": [], "polygons": []},
        "rallyPoints": {"version": 2, "points": []},
    }


def write_json(data: Any, path: str | Path) -> Path:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(data, indent=2), encoding="utf-8")
    return out
