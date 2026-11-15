"""Frame conversions between PX4 (local NED, origin = home) and the cage ENU frame.

Kept free of ROS imports so it is unit-tested without a ROS installation.
"""

from __future__ import annotations

import math

import numpy as np


def px4_ned_to_cage_enu(n: float, e: float, d: float, home: tuple[float, float]
                        ) -> tuple[float, float, float]:
    """PX4 local NED (relative to home) -> cage ENU (origin at the cage centre)."""
    return home[0] + e, home[1] + n, -d


def cage_enu_to_setpoint(x: float, y: float, z: float, home: tuple[float, float], yaw_deg: float
                         ) -> tuple[float, float, float, float]:
    """Cage ENU -> ``ros_msg/Vector4`` for ``/drone/in/offboard/pos/ned``.

    ``px4_to_ros2`` expects x = north, y = east, z = altitude *up* (it negates it
    into NED down) and w = yaw in degrees.
    """
    return y - home[1], x - home[0], z, yaw_deg


def heading_to_enu_yaw(heading_deg: float) -> float:
    """PX4 heading (deg, 0 = north, clockwise) -> ENU yaw (rad, 0 = east, counter-clockwise)."""
    return math.radians(90.0 - heading_deg)


def lidar_to_world(points_sensor: np.ndarray, drone_enu: tuple[float, float, float], yaw_enu: float,
                   v_max_deg: float, beam_step_deg: float) -> tuple[np.ndarray, np.ndarray]:
    """Rotate sensor-frame points (x forward, y left, z up) into the cage frame.

    Roll and pitch are ignored (small in hover/cruise). Returns world points and
    a mask of points that came from the highest beam.
    """
    pts = np.asarray(points_sensor, dtype=float).reshape(-1, 3)
    finite = np.isfinite(pts).all(axis=1)
    pts = pts[finite]
    c, s = math.cos(yaw_enu), math.sin(yaw_enu)
    world = np.empty_like(pts)
    world[:, 0] = drone_enu[0] + c * pts[:, 0] - s * pts[:, 1]
    world[:, 1] = drone_enu[1] + s * pts[:, 0] + c * pts[:, 1]
    world[:, 2] = drone_enu[2] + pts[:, 2]
    elevation = np.degrees(np.arctan2(pts[:, 2], np.hypot(pts[:, 0], pts[:, 1])))
    top = elevation >= v_max_deg - beam_step_deg / 2
    return world, top
