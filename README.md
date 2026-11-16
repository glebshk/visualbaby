# Aufdeckung Simulation (Drone Challenge Brunswick 2027)

Simulation foundation for the **Aufdeckalgorithmus**: fly over the whole 45 m DLR drone cage
without touching anything, image every reachable square metre with the downward camera, and do
it for *any* layout of buildings, trees, vehicles and rain curtains, because the real layout is
only known on competition day.

Built from the reusable parts of `visualbaby` (PX4 <-> ROS 2 bridge, x500 models, Gazebo setup);
everything specific to the old indoor challenge was removed.

```
aufdeckung/            pure-Python core (no ROS needed): planner, mapping, sensors, simulator, tests
src/aufdeckung_ros/    ROS 2 node: runs the same mission controller against PX4 (SITL or real)
src/px4_to_ros2/       MAVSDK <-> ROS 2 bridge (from visualbaby, unchanged)
src/ros_msg/           custom messages (from visualbaby, unchanged)
simulation/            Gazebo models: x500, x500_aufdeckung (camera + 3D lidar), OakD-Lite_down, LW20
scripts/, tmux/        Gazebo / PX4 SITL / QGC launch helpers
docs/ARCHITECTURE.md   design, interfaces to the other team members, open points
```

## 1. Quick start (any OS: Windows, macOS, Linux)

Needs Python 3.10+. No ROS, Gazebo or GPU required.

```bash
cd aufdeckung
python -m pip install -e ".[viz,dev]"
```

```bash
python -m aufdeckung simulate --preset dense_urban --seed 3 --plot --events
```

That flies one random arena in the headless simulator and writes `out/dense_urban_3_run.png`
(flown path coloured by altitude, and a map of when each cell was first imaged).

Other commands:

| command | what it does |
|---|---|
| `python -m aufdeckung generate --preset street --seed 1 --sdf --plot` | random arena as YAML, Gazebo world and plot |
| `python -m aufdeckung plan --scenario out/street_1.yaml --plot` | offline plan on a known layout, exported as PX4 NED setpoints and a QGroundControl `.plan` |
| `python -m aufdeckung simulate --scenario my_layout.yaml --prior` | closed loop; `--prior` gives the planner the layout up front, default is explore mode (map built from the lidar) |
| `python -m aufdeckung benchmark --presets all --seeds 20 --json out/report.json` | many random arenas per preset, checked against the acceptance criteria (exit code 1 on failure) |

Presets: `sparse`, `urban_ring` (like Abb. 3 of the Regelwerk), `dense_urban`, `center_tower`,
`street`, `mixed`. Change any parameter with `--config overrides.yaml`, for example:

```yaml
flight: {cruise_alt: 7.0, max_speed: 4.0, clearance: 2.0}
camera: {hfov: 1.3}
mission: {time_budget: 600}
```

## 2. Tests and the development loop

One script runs every check. CI on GitHub runs the same script, so green here means green there:

```bash
./scripts/check.sh
```

| option | adds | time |
|---|---|---|
| *(none)* | lint (ruff), types (mypy), fast tests | ~3 min |
| `--quick` | lint and types only | seconds |
| `--full` | slow tests: whole missions, two random layouts per preset | ~10 min |
| `--bench N` | benchmark, N random arenas per preset, explore and prior mode | ~1 min per arena per core |
| `--fix` | let ruff fix what it can first | |

The loop for every change:

1. **Reproduce** the problem as a failing test, or find it with the benchmark.
   `--first-seed` gives arenas no earlier run has seen, so improvements are not tuned to known layouts:
   `python -m aufdeckung benchmark --presets all --seeds 5 --first-seed 10 --workers 4`
2. **Diagnose** with one arena: `python -m aufdeckung simulate --preset street --seed 13 --events --plot`.
3. **Fix**, then `./scripts/check.sh` until green.
4. **Verify** with the benchmark, before and after, on the same seeds; safety numbers (collisions,
   minimum clearance) must not get worse.
5. **Push**; CI runs the same checks again.

Acceptance criteria (`sim/benchmark.Criteria`): no collision, at least 0.75 m clearance, at least 95 % of
the observable area imaged within 600 s, landed at home.

## 3. Gazebo + PX4 SITL (Linux, or Windows via WSL2 / Docker)

The full simulation needs ROS 2 Humble, Gazebo Harmonic, PX4-Autopilot and MAVSDK. All of that is
in the team's dev image (see the devcontainer configs in `.devcontainer/`).

```bash
docker pull ghcr.io/aachen-drone-development-initiative/imav/dev:latest
```

Inside the container, from the repository root:

```bash
./scripts/setup-sim.sh
```

```bash
./scripts/launch-sim-aufdeckung.sh dense_urban 3
```

This generates the arena, starts Gazebo with it, spawns `x500_aufdeckung` on the start pad,
starts PX4 SITL, the Gazebo->ROS bridge (`simulation/gz-bridge.yaml`), `px4_to_ros2` and the
mission node, all in a tmux session. The first PX4 build takes a while; if the drone does not
start, let the build finish and run the script again. Close the Gazebo GUI before killing the
tmux session (`Ctrl+B`, then `:kill-session`).

### Debugging in Gazebo

* Gazebo GUI: pause/step the world, *Entity Tree* to inspect models, right-click to move objects.
* `gz topic -l`, then `gz topic -e -t /lidar/points` to check raw sensor data.
* `ros2 topic echo /aufdeckung/phase` for the mission state, `ros2 topic hz /lidar/points`.
* `rviz2`, fixed frame `map`: add `/aufdeckung/height_map`, `/aufdeckung/coverage` (OccupancyGrid)
  and `/aufdeckung/route` (Path) to watch mapping and planning live.
* `./scripts/launch-qgc.sh` for QGroundControl (flight mode, failsafes, battery).
* Reproduce a SITL problem headless first: `python -m aufdeckung simulate --scenario simulation/worlds/<name>.yaml --events`.

## 4. Mapping to the team structure (Struktur.pdf)

| Struktur | where |
|---|---|
| Navigation / Aufdeckalgorithmus (Bild) | `aufdeckung/planning/coverage_planner.py`, `mission.py` |
| Navigation / Mapping (Lidar) | `aufdeckung/mapping/grid.py` (`HeightMap.integrate_hits`), `sensors/lidar.py` |
| Bildverarbeitung / Triangulierung | consumes `/camera/bottom/*`; reports go through `export/lagebild.py` |
| Schnittstelle / MQTT, Wegpunkte, Georeferenz | `export/lagebild.py`, `export/waypoints.py`, `geo.py` |
| Software (Drohne) / PX4, Failsafe | `src/px4_to_ros2`, `src/aufdeckung_ros` |

Details, assumptions and open points: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).
