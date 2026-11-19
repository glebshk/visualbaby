# Architecture

## Goal and rules that drive the design (Regelwerk 2027)

| rule | consequence in the code |
|---|---|
| cage 45 m diameter, 13 m high, recommended altitude limit 10 m | `ArenaConfig`, `FlightConfig.max_alt = 10` |
| max. 10 m/s | `FlightConfig.max_speed` is validated against it (default 3 m/s for sharp images) |
| obstacles anywhere, some up to the ceiling | 2.5D height map, per-cell minimum safe altitude, detours around what cannot be overflown |
| rain curtain switches on during the flight | rain zones are timed no-fly cylinders; the route is replanned, the drone escapes, the area is revisited later |
| task 1: 10 min, bonus for objects found in the first 3 min | time budget and return reserve; the planner sweeps first (cheap, broad coverage) and fills gaps later |
| detection counts within 2 m of the true position | the simulator reports positions with GNSS and detection noise; metrics count reports within 1 m radius |
| layout only known on competition day | **explore mode is the default**: the map is built from the lidar during the flight; a known layout is only an optional prior |

## Frames

* Cage frame: ENU, origin at the cage centre, x east, y north, z up (metres). Everything in `aufdeckung` uses it.
* PX4 local NED: origin at the take-off point (home). `src/aufdeckung_ros/aufdeckung_ros/conversions.py` converts.
* WGS84: `aufdeckung/geo.py`, origin `ArenaConfig.origin_lat/lon` (**placeholder, replace with the surveyed cage centre**).

## Pipeline (one controller update)

```
lidar points --> HeightMap (2.5D: max surface height per 0.5 m cell + "observed" flag;
                    only hits within `planner.lidar_map_range` (12 m) of the drone, not on
                    the net or ceiling, confirmed by 2 scans; top-beam hits within 10 m get
                    a 2 m "may continue upward" margin, which beams passing over the cell
                    later carve away again, never below a real hit; all beams carve)
                    |
                    v
             FlightMap: required altitude per cell
             = max height within `clearance` + `vertical_clearance`, >= min_alt;
             unobserved cells are assumed clear only up to cruise level;
             not flyable if > max_alt, near the net, or in an active rain zone
                    |
camera footprint -->|--> CoverageMap (which cells have been imaged)
                    v
             MissionController (TAKEOFF -> COVER <-> WAIT -> RETURN -> LAND -> DONE)
               - re-validates the remaining route after every map/rain change
               - replans with plan_coverage(...) when it becomes unsafe or runs out
               - returns home when time budget minus reserve is reached
                    |
                    v
             setpoint (x, y, z) -> PX4 offboard position (via px4_to_ros2)
```

### Coverage planner (multi-altitude)

1. Lawnmower rows along x and along y; the axis that images more (then: shorter) wins.
   Row spacing = camera footprint at cruise altitude x (1 - overlap).
2. Each row sample flies at its cell's transit altitude: cruise altitude, raised locally where the
   obstacles below need it. Low buildings are overflown (their roofs get imaged); taller ones split
   the row.
3. Rows are ordered greedily by real path length (A* on the flight map, smoothed by line-of-sight
   shortcuts that never climb higher than needed).
4. Gap filling: the route's coverage is predicted with occlusion; extra viewpoints are added by
   greedy set cover, also at low altitude (down to `min_alt`) where the map is known, which reaches
   alleys and ground next to tall buildings. Cells no reachable viewpoint can see (inside towers,
   under tree crowns) are marked hopeless; before returning home they get one more try with the
   map known by then (early verdicts come from a coarse map).
   If the map closes in so that the drone's flyable pocket no longer connects to home, planning
   starts with a rescue path out of the pocket instead of judging the arena from inside it.
5. 3D expansion: climb before a leg, descend after it, so every horizontal leg is at or above the
   required altitude along the whole leg. `tests/helpers.assert_route_safe` checks this invariant.

## Interfaces for the other team members

* **Lidar mapping**: anything that produces world-frame points can call
  `HeightMap.integrate_hits(points, sensor_z, top_beam)`. A real SLAM/odometry solution can replace
  the simple yaw-only transform in `aufdeckung_ros/conversions.py`.
* **Triangulation / object detection**: subscribe to `/camera/bottom/rgb` (+ depth) and the drone pose; send
  results with `export/lagebild.detection_message(...)` over MQTT once the official schema is out (31.01.2027).
* **Waypoints / MQTT interface**: `export/waypoints.py` produces PX4 NED setpoints and QGC `.plan` files.
* **Rain information**: publish active zones on `/aufdeckung/rain` (`Float32MultiArray`, `[x, y, r, ...]`).

## Validation

* Unit tests for every module; integration tests run whole missions in the headless simulator.
* `python -m aufdeckung benchmark` flies many random layouts per preset. Criteria (`sim/benchmark.Criteria`):
  zero collisions, minimum clearance >= 0.75 m, >= 95 % of the *observable* area imaged within 600 s,
  landed at home. "Observable" is computed by an oracle with the true map (`sim/metrics.observable_mask`).

## Known limitations / next steps

* The headless drone is a point mass with acceleration limits and wind gusts, not PX4 dynamics.
  Run the same arenas in Gazebo + PX4 SITL (`scripts/launch-sim-aufdeckung.sh`) before trusting timings.
* The lidar-to-world transform ignores roll and pitch; fine in slow flight, revisit for faster flight.
* 2.5D map: overhangs (tree crowns, bridges) are treated as solid down to the ground (conservative).
* Free-space carving only uses beams that hit something; beams without a return (into the sky) carry
  free-space evidence too but are dropped from the point cloud. Using them (organised Gazebo cloud,
  beam index -> direction) would clear margins faster in open areas.
* Thin obstacles taller than cruise altitude minus vertical clearance (> 6.5 m by default) must be
  seen by the lidar in time; poles up to 6 m are always overflown at cruise altitude.
* Rain zones are visual-only in Gazebo; switch them on by publishing `/aufdeckung/rain`.
* Cage geometry (16-gon) and WGS84 origin are approximations from Abb. 2: survey before the event.
