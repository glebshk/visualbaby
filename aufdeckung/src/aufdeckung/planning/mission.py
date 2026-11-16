"""Mission controller: the same state machine runs in the Python simulator and the ROS 2 node.

Phases: TAKEOFF -> COVER <-> WAIT -> RETURN -> LAND -> DONE (HOLD on unrecoverable errors).

Every update fuses lidar hits into the height map, books camera coverage and
re-validates the remaining route. The route is replanned when the map or the
active rain zones make it unsafe, when it runs out while unseen cells remain,
or the drone returns home when the time budget requires it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from aufdeckung.config import Config
from aufdeckung.geometry import BoolArray, FloatArray, Shape
from aufdeckung.mapping.coverage import CoverageMap
from aufdeckung.mapping.grid import GridSpec, HeightMap, cage_mask
from aufdeckung.planning.coverage_planner import (
    PlanningError,
    Waypoint,
    plan_coverage,
    plan_transit,
)
from aufdeckung.planning.flightmap import FlightMap, build_flight_map, rain_mask
from aufdeckung.scenario.model import RainZone
from aufdeckung.sensors.camera import visible_cells


class Phase(str, Enum):
    TAKEOFF = "takeoff"
    COVER = "cover"
    WAIT = "wait"
    RETURN = "return"
    LAND = "land"
    DONE = "done"
    HOLD = "hold"


@dataclass(frozen=True)
class Setpoint:
    x: float
    y: float
    z: float
    yaw: float
    phase: Phase


@dataclass
class MissionEvent:
    t: float
    kind: str
    detail: str = ""


@dataclass
class MissionController:
    cfg: Config
    start: tuple[float, float]
    prior: tuple[Shape, ...] | None = None
    spec: GridSpec = field(init=False)
    hm: HeightMap = field(init=False)
    coverage: CoverageMap = field(init=False)
    need: BoolArray = field(init=False)
    fm: FlightMap = field(init=False)
    rescue_fm: FlightMap = field(init=False)
    phase: Phase = field(init=False, default=Phase.TAKEOFF)
    route: list[Waypoint] = field(init=False, default_factory=list)
    events: list[MissionEvent] = field(init=False, default_factory=list)
    replans: int = field(init=False, default=0)

    def __post_init__(self) -> None:
        self.spec = GridSpec.for_arena(self.cfg.arena, self.cfg.planner.grid_res)
        if self.prior is not None:
            self.hm = HeightMap.from_prior(self.spec, self.cfg.arena, self.prior)
        elif self.cfg.mission.explore:
            self.hm = HeightMap(self.spec, self.cfg.arena)
        else:
            raise ValueError("explore=False needs a prior map")
        self.coverage = CoverageMap(self.spec)
        self.need = cage_mask(self.spec, self.cfg.arena)
        fl = self.cfg.flight
        # Reduced margins, only for getting out when the map closes in around the drone.
        self._rescue_cfg = self.cfg.with_overrides({"flight": {
            "clearance": fl.drone_radius + 0.3,
            "vertical_clearance": min(fl.vertical_clearance, 0.75),
            "min_alt": min(fl.min_alt, fl.cruise_alt),
        }})
        self.hopeless = np.zeros(self.spec.shape, dtype=bool)
        self._hopeless_retry_version = -1
        self._rain: tuple[RainZone, ...] = ()
        self._dirty = True
        self._last_check = -math.inf
        self._last_cam = -math.inf
        self._last_replan = -math.inf
        self._hold: tuple[float, float, float] | None = None
        self._last_plan_state: tuple[int, int] = (-1, -1)
        self._axis: str | None = None
        self._rebuild()

    # ----------------------------------------------------------------- public API

    def update(
        self,
        t: float,
        pos: FloatArray,
        scan: tuple[FloatArray, BoolArray] | None = None,
        rain_active: tuple[RainZone, ...] = (),
    ) -> Setpoint:
        p = (float(pos[0]), float(pos[1]), float(pos[2]))
        if scan is not None and self.hm.integrate_hits(scan[0], p[2], scan[1], origin=p):
            self._dirty = True
        if tuple(rain_active) != self._rain:
            self._rain = tuple(rain_active)
            self._dirty = True
            self._log(t, "rain", f"{len(self._rain)} zone(s) active")
            if self.phase == Phase.WAIT:
                self._set_phase(t, Phase.COVER)
                self.route = []
        if t - self._last_cam >= 1.0 / self.cfg.camera.rate_hz and p[2] > 0.5:
            self._last_cam = t
            iy, ix = visible_cells(p, self.hm.height, self.spec, self.cfg.camera)
            mask = np.zeros(self.spec.shape, dtype=bool)
            mask[iy, ix] = True
            self.coverage.mark(mask, t)

        if self._dirty and t - self._last_check >= self.cfg.planner.replan_min_interval:
            self._last_check = t
            self._dirty = False
            self._rebuild()
            if self.route and not self._route_valid(p):
                self._log(t, "route_invalid", self.phase.value)
                self._replan(t, p)

        self._check_time(t, p)
        return self._step(t, p)

    @property
    def done(self) -> bool:
        return self.phase == Phase.DONE

    # ----------------------------------------------------------------- internals

    def _step(self, t: float, p: tuple[float, float, float]) -> Setpoint:
        tol = self.cfg.flight.waypoint_tolerance
        while self.route and _dist(p, self.route[0].xyz) < tol:
            self.route.pop(0)
        if not self.route:
            self._on_route_end(t, p)
        if self.phase == Phase.DONE:
            return Setpoint(self.start[0], self.start[1], 0.0, 0.0, self.phase)
        if self.phase in (Phase.WAIT, Phase.HOLD) or not self.route:
            if self._hold is None:
                self._hold = p
            return Setpoint(*self._hold, 0.0, self.phase)
        self._hold = None
        if self.phase == Phase.TAKEOFF and p[2] >= self.cfg.flight.min_alt - tol:
            self._set_phase(t, Phase.COVER)
        if self.phase == Phase.RETURN and len(self.route) == 1:
            self._set_phase(t, Phase.LAND)
        wp = self.route[0]
        return Setpoint(wp.x, wp.y, wp.z, 0.0, self.phase)

    def _on_route_end(self, t: float, p: tuple[float, float, float]) -> None:
        if self.phase in (Phase.LAND, Phase.RETURN):
            if math.hypot(p[0] - self.start[0], p[1] - self.start[1]) < 1.0 and p[2] < 0.3:
                self._set_phase(t, Phase.DONE)
            else:
                self._start_return(t, p)
            return
        if self.phase in (Phase.TAKEOFF, Phase.COVER):
            self._replan(t, p, route_ended=True)
            if not self.route and self.hopeless.any() and self._hopeless_retry_version != self.hm.version:
                # "Hopeless" is judged on the map known at that moment, which early in the flight
                # is coarse and pessimistic. Before giving up, retry those cells once with the
                # current map (again after each map change); the stall check stops no-progress loops.
                self._hopeless_retry_version = self.hm.version
                self._log(t, "hopeless_retry", f"{int(np.count_nonzero(self.hopeless))} cells")
                self.hopeless[:] = False
                self._replan(t, p, route_ended=True)
            if not self.route:
                if self._rain_blocked_need():
                    self._set_phase(t, Phase.WAIT)
                else:
                    self._start_return(t, p)

    def _replan(self, t: float, p: tuple[float, float, float], route_ended: bool = False) -> None:
        if self.phase in (Phase.RETURN, Phase.LAND):
            self._start_return(t, p)
            return
        if t - self._last_replan < 1e-9 and self.route:
            return
        self._last_replan = t
        self.replans += 1
        need = self.need & ~self.coverage.covered & ~self.hopeless
        try:
            axes = (self._axis,) if self._axis else ("x", "y")
            plan = plan_coverage(p, self.hm, self.fm, need, self.cfg, axes=axes,
                                 rescue=self.rescue_fm, anchor=self.start)
            if plan.axis in ("x", "y"):
                self._axis = plan.axis
        except PlanningError as exc:
            self._log(t, "planning_error", str(exc))
            self.route = []
            self._start_return(t, p)
            return
        if plan.hopeless is not None:
            # Cells no reachable viewpoint can image (inside tall buildings, under
            # tree crowns, ...) are dropped for good, except near rain zones.
            soft = rain_mask(self.spec, self._rain, self.cfg.flight.clearance + 6.0)
            self.hopeless |= plan.hopeless & ~soft
        useful = int(np.count_nonzero(plan.predicted & need))
        covered = int(np.count_nonzero(self.coverage.covered))
        # A finished route that taught us nothing new would loop forever: stop instead.
        stalled = route_ended and (covered, useful) == self._last_plan_state
        if route_ended:
            self._last_plan_state = (covered, useful)
        if useful < self.cfg.planner.gapfill_min_gain or stalled:
            self.route = []
        else:
            self.route = plan.waypoints
        self._log(t, "replan", f"#{self.replans} {len(self.route)} wps, +{useful} cells, "
                               f"est {plan.est_time:.0f}s, axis {plan.axis}")

    def _start_return(self, t: float, p: tuple[float, float, float]) -> None:
        try:
            wps = plan_transit(p, self.start, self.fm, rescue=self.rescue_fm)
        except PlanningError as exc:
            self._log(t, "return_failed", str(exc))
            self.route = []
            self._set_phase(t, Phase.HOLD)
            return
        self.route = [*wps, Waypoint(self.start[0], self.start[1], 0.0, "land")]
        if self.phase not in (Phase.RETURN, Phase.LAND):
            self._set_phase(t, Phase.RETURN)

    def _check_time(self, t: float, p: tuple[float, float, float]) -> None:
        if self.phase not in (Phase.TAKEOFF, Phase.COVER, Phase.WAIT):
            return
        fl = self.cfg.flight
        home = math.hypot(p[0] - self.start[0], p[1] - self.start[1])
        needed = 1.5 * home / fl.max_speed + (fl.max_alt + p[2]) / fl.max_climb
        if t + needed + self.cfg.mission.return_reserve >= self.cfg.mission.time_budget:
            self._log(t, "time_budget", f"{needed:.0f}s needed to get home")
            self._start_return(t, p)

    def _route_valid(self, p: tuple[float, float, float]) -> bool:
        prev = p
        for i, wp in enumerate(self.route):
            soft = wp.kind == "escape"
            horiz = math.hypot(wp.x - prev[0], wp.y - prev[1])
            if horiz < 0.5:
                iy, ix = self.fm.cell(wp.x, wp.y)
                ok = wp.kind == "land" or (
                    bool((self.fm.hard_ok if soft else self.fm.flyable)[iy, ix])
                    and self.fm.required_alt[iy, ix] <= wp.z + 0.05
                )
            else:
                flat, peak = self.fm.segment_profile((prev[0], prev[1]), (wp.x, wp.y), soft=soft)
                floor = wp.z if i == 0 else min(prev[2], wp.z)
                ok = flat and peak <= floor + 0.05
            if not ok:
                return False
            prev = wp.xyz
        return True

    def _rain_blocked_need(self) -> bool:
        if not self._rain:
            return False
        blocked = rain_mask(self.spec, self._rain, self.cfg.flight.clearance + 6.0)
        return bool((blocked & self.need & ~self.coverage.covered & ~self.hopeless).any())

    def _rebuild(self) -> None:
        unknown = self.prior is None
        self.fm = build_flight_map(self.hm, self.cfg, self._rain, assume_unknown_below_cruise=unknown)
        self.rescue_fm = build_flight_map(self.hm, self._rescue_cfg, self._rain,
                                          assume_unknown_below_cruise=unknown)

    def _set_phase(self, t: float, phase: Phase) -> None:
        if phase != self.phase:
            self._log(t, "phase", f"{self.phase.value} -> {phase.value}")
            self.phase = phase

    def _log(self, t: float, kind: str, detail: str = "") -> None:
        self.events.append(MissionEvent(round(t, 2), kind, detail))


def _dist(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.dist(a, b)
