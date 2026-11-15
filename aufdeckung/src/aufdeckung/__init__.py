"""Aufdeckung: coverage planning, 2.5D mapping and headless simulation.

The package is ROS-free so the same planning code runs in the fast Python
simulator, in unit tests and behind the ROS 2 node (``src/aufdeckung_ros``).

Coordinate convention: local ENU in metres, origin at the cage centre,
``x`` = east, ``y`` = north, ``z`` = up. Conversions to PX4 NED and WGS84
live in :mod:`aufdeckung.geo`.
"""

__version__ = "0.1.0"
