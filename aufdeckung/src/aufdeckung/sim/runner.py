"""Closed-loop simulation: true world -> sensors -> MissionController -> drone."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from aufdeckung.config import DETECTION_RADIUS_M, Config
from aufdeckung.geometry import FloatArray, distance_to_shape
from aufdeckung.mapping.coverage import CoverageMap
from aufdeckung.mapping.grid import GridSpec, rasterize
from aufdeckung.planning.mission import MissionController, Phase
from aufdeckung.scenario.model import Scenario
from aufdeckung.sensors.camera import visible_cells
from aufdeckung.sensors.lidar import LidarSensor
from aufdeckung.sim.drone import KinematicDrone
from aufdeckung.sim.metrics import net_distance, observable_mask


@dataclass
class TargetReport:
    name: str
    cls: str
    true_xy: tuple[float, float]
    reported_xy: tuple[float, float]
    t_found: float

    @property
    def error(self) -> float:
        return math.dist(self.true_xy, self.reported_xy)


@dataclass
class RunResult:
    scenario: Scenario
    metrics: dict[str, Any]
    trajectory: FloatArray
    coverage: CoverageMap
    observable: np.ndarray
    reports: list[TargetReport]
    controller: MissionController
    collisions: list[tuple[float, str]] = field(default_factory=list)


def run_mission(scenario: Scenario, cfg: Config, use_prior: bool = False, record_every: int = 2
                ) -> RunResult:
    scenario.validate(cfg.arena)
    rng = np.random.default_rng(cfg.sim.seed + (scenario.seed or 0) * 7919)
    spec = GridSpec.for_arena(cfg.arena, cfg.planner.grid_res)
    true_h = rasterize(scenario.obstacles, spec)
    observable = observable_mask(scenario, cfg)
    truth_cov = CoverageMap(spec)
    lidar = LidarSensor(cfg.lidar)
    drone = KinematicDrone(cfg.flight, (scenario.start[0], scenario.start[1], 0.0),
                           wind_std=cfg.sim.wind_std, rng=rng)
    ctrl = MissionController(cfg, scenario.start, prior=scenario.obstacles if use_prior else None)

    dt = cfg.sim.dt
    t_end = cfg.mission.time_budget + 120.0
    next_lidar, next_cam = 0.0, 0.0
    min_clear, min_net = math.inf, math.inf
    collisions: list[tuple[float, str]] = []
    rain_exposure = 0.0
    path_len = 0.0
    traj: list[list[float]] = []
    reports: dict[str, TargetReport] = {}
    t = 0.0
    step = 0
    phase_times: dict[str, float] = {}
    while t <= t_end and not ctrl.done:
        true_pos = drone.pos.copy()
        est = true_pos + rng.normal(0.0, cfg.sim.gnss_std, size=3) * np.array([1.0, 1.0, 0.5])
        scan = None
        if t >= next_lidar:
            next_lidar += 1.0 / cfg.lidar.rate_hz
            pts, top = lidar.scan(true_pos, scenario.obstacles, rng)
            scan = (pts + (est - true_pos), top)
        rain = scenario.active_rain(t)
        sp = ctrl.update(t, est, scan, rain)
        phase_times.setdefault(sp.phase.value, t)

        if true_pos[2] > 0.5 and t >= next_cam:
            next_cam += 1.0 / cfg.camera.rate_hz
            iy, ix = visible_cells(true_pos, true_h, spec, cfg.camera)
            mask = np.zeros(spec.shape, dtype=bool)
            mask[iy, ix] = True
            truth_cov.mark(mask, t)
            for tgt in scenario.targets:
                if tgt.name in reports:
                    continue
                ty, tx = spec.index(tgt.x, tgt.y)
                if 0 <= ty < spec.ny and 0 <= tx < spec.nx and mask[ty, tx]:
                    err = rng.normal(0.0, cfg.sim.detection_error_std, size=2)
                    reports[tgt.name] = TargetReport(tgt.name, tgt.cls, (tgt.x, tgt.y),
                                                     (tgt.x + float(err[0]), tgt.y + float(err[1])), t)

        drone.step((sp.x, sp.y, sp.z), dt)
        path_len += float(np.linalg.norm(drone.pos - true_pos))
        if drone.pos[2] > 0.2:
            for obs in scenario.obstacles:
                if math.hypot(obs.x - drone.pos[0], obs.y - drone.pos[1]) - obs.bounding_radius > 5.0:
                    continue
                d = distance_to_shape(obs, drone.pos)
                min_clear = min(min_clear, d)
                if d < cfg.flight.drone_radius and (not collisions or t - collisions[-1][0] > 1.0):
                    collisions.append((round(t, 2), obs.name or obs.kind))
            dn = net_distance(drone.pos[0], drone.pos[1], cfg)
            min_net = min(min_net, dn)
            if dn < cfg.flight.drone_radius and (not collisions or t - collisions[-1][0] > 1.0):
                collisions.append((round(t, 2), "net"))
            if any(math.hypot(z.x - drone.pos[0], z.y - drone.pos[1]) <= z.radius for z in rain):
                rain_exposure += dt
        if step % record_every == 0:
            traj.append([t, *drone.pos.tolist(), float(list(Phase).index(sp.phase))])
        t += dt
        step += 1

    landed = math.hypot(drone.pos[0] - scenario.start[0], drone.pos[1] - scenario.start[1]) < 1.0 \
        and drone.pos[2] < 0.3
    bonus = cfg.mission.bonus_window
    obs_targets = [tg for tg in scenario.targets if observable[spec.index(tg.x, tg.y)]]
    found = list(reports.values())
    metrics: dict[str, Any] = {
        "scenario": scenario.name,
        "mode": "prior" if use_prior else "explore",
        "mission_time": round(t, 1),
        "completed": ctrl.done,
        "landed_home": landed,
        "collisions": len(collisions),
        "min_clearance": round(min_clear, 3) if math.isfinite(min_clear) else None,
        "min_net_distance": round(min_net, 3) if math.isfinite(min_net) else None,
        "coverage": round(truth_cov.fraction(observable), 4),
        "coverage_bonus": round(truth_cov.fraction_at(observable, bonus), 4),
        "coverage_time_budget": round(truth_cov.fraction_at(observable, cfg.mission.time_budget), 4),
        "observable_cells": int(np.count_nonzero(observable)),
        "targets": len(scenario.targets),
        "targets_observable": len(obs_targets),
        "targets_found": len(found),
        "targets_found_bonus": sum(1 for r in found if r.t_found <= bonus),
        "targets_within_2m": sum(1 for r in found if r.error <= DETECTION_RADIUS_M),
        "path_length": round(path_len, 1),
        "replans": ctrl.replans,
        "rain_exposure_s": round(rain_exposure, 1),
        "phase_start": {k: round(v, 1) for k, v in phase_times.items()},
    }
    return RunResult(scenario, metrics, np.array(traj), truth_cov, observable, found, ctrl, collisions)
