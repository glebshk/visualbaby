"""Point-mass multicopter with speed/climb/acceleration limits and wind gusts.

A stand-in for PX4 position control: it flies straight at the setpoint and
brakes so it can stop on it. Use PX4 SITL in Gazebo for real dynamics.
"""

from __future__ import annotations

import math

import numpy as np

from aufdeckung.config import FlightConfig
from aufdeckung.geometry import FloatArray


class KinematicDrone:
    def __init__(self, fl: FlightConfig, pos: tuple[float, float, float], wind_std: float = 0.0,
                 rng: np.random.Generator | None = None) -> None:
        self.fl = fl
        self.pos: FloatArray = np.array(pos, dtype=float)
        self.vel: FloatArray = np.zeros(3)
        self.wind: FloatArray = np.zeros(3)
        self.wind_std = wind_std
        self.rng = rng or np.random.default_rng(0)

    def step(self, target: tuple[float, float, float], dt: float) -> None:
        fl = self.fl
        err = np.asarray(target, dtype=float) - self.pos
        dh = math.hypot(err[0], err[1])
        v_des = np.zeros(3)
        if dh > 1e-6:
            speed = min(fl.max_speed, math.sqrt(2 * fl.max_accel * dh) * 0.9, dh / dt)
            v_des[:2] = err[:2] / dh * speed
        dz = abs(err[2])
        if dz > 1e-6:
            v_des[2] = math.copysign(min(fl.max_climb, math.sqrt(2 * fl.max_accel * dz) * 0.9, dz / dt),
                                     err[2])
        dv = v_des - self.vel
        n = float(np.linalg.norm(dv))
        if n > fl.max_accel * dt:
            dv *= fl.max_accel * dt / n
        self.vel = self.vel + dv
        if self.wind_std > 0:
            # Ornstein-Uhlenbeck gusts with ~2 s correlation time.
            tau = 2.0
            self.wind += (-self.wind / tau) * dt + self.wind_std * math.sqrt(2 * dt / tau) * \
                self.rng.normal(size=3) * np.array([1.0, 1.0, 0.3])
        self.pos = self.pos + (self.vel + self.wind) * dt
        if self.pos[2] < 0.0:
            self.pos[2] = 0.0
            self.vel[2] = max(self.vel[2], 0.0)
