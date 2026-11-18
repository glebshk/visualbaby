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


def cloud_xyz(points: object) -> np.ndarray:
    """(N, 3) float array of x, y, z from what ``point_cloud2.read_points`` returns.

    Gazebo's lidar cloud also carries intensity and ring fields of other types, which
    ``read_points_numpy`` refuses (it needs one datatype for all fields), so the node reads a
    structured array instead. Older ROS versions return a list of tuples; both are accepted.
    """
    if isinstance(points, np.ndarray) and points.dtype.names:
        xyz = np.column_stack([points[name].astype(float).ravel() for name in ("x", "y", "z")])
    else:
        xyz = np.array([(p[0], p[1], p[2]) for p in points], dtype=float)  # type: ignore[attr-defined]
    return xyz.reshape(-1, 3)


# The lidar sits 12 cm above the x500's body; its motors and propellers are 0.25-0.4 m away and
# inside its field of view. Those self-hits looked like an obstacle at flight altitude right where
# the drone is, so its own cell and the take-off area became unflyable (mission went to HOLD).
SELF_FILTER_RADIUS_M = 0.6


def lidar_to_world(points_sensor: np.ndarray, drone_enu: tuple[float, float, float], yaw_enu: float,
                   v_max_deg: float, beam_step_deg: float, min_range: float = SELF_FILTER_RADIUS_M
                   ) -> tuple[np.ndarray, np.ndarray]:
    """Rotate sensor-frame points (x forward, y left, z up) into the cage frame.

    Points closer than ``min_range`` (the drone itself) and non-finite points (no return)
    are dropped. Roll and pitch are ignored (small in hover/cruise). Returns world points
    and a mask of points that came from the highest beam.
    """
    pts = np.asarray(points_sensor, dtype=float).reshape(-1, 3)
    finite = np.isfinite(pts).all(axis=1)
    pts = pts[finite]
    pts = pts[np.linalg.norm(pts, axis=1) >= min_range]
    c, s = math.cos(yaw_enu), math.sin(yaw_enu)
    world = np.empty_like(pts)
    world[:, 0] = drone_enu[0] + c * pts[:, 0] - s * pts[:, 1]
    world[:, 1] = drone_enu[1] + s * pts[:, 0] + c * pts[:, 1]
    world[:, 2] = drone_enu[2] + pts[:, 2]
    elevation = np.degrees(np.arctan2(pts[:, 2], np.hypot(pts[:, 0], pts[:, 1])))
    top = elevation >= v_max_deg - beam_step_deg / 2
    return world, top
