"""Simulated spinning 3D lidar (ray cast against the true scenario geometry)."""

from __future__ import annotations

import math

import numpy as np

from aufdeckung.config import LidarConfig
from aufdeckung.geometry import BoolArray, FloatArray, Shape, ray_cast


class LidarSensor:
    def __init__(self, config: LidarConfig) -> None:
        self.config = config
        az = np.linspace(0.0, 2 * math.pi, config.h_samples, endpoint=False)
        el = np.radians(np.linspace(config.v_min_deg, config.v_max_deg, config.v_samples))
        a, e = np.meshgrid(az, el)
        self.dirs = np.stack(
            [np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)], axis=-1
        ).reshape(-1, 3)
        self.top_beam = (np.arange(config.v_samples)[:, None] == config.v_samples - 1).repeat(
            config.h_samples, axis=1
        ).ravel()

    def scan(
        self,
        pos: FloatArray,
        shapes: list[Shape] | tuple[Shape, ...],
        rng: np.random.Generator | None = None,
    ) -> tuple[FloatArray, BoolArray]:
        """Return world-frame hit points (N, 3) and a mask of which came from the top beam."""
        t = ray_cast(pos, self.dirs, list(shapes), self.config.max_range)
        ok = np.isfinite(t) & (t >= self.config.min_range)
        t = t[ok]
        if rng is not None and self.config.noise_std > 0:
            t = t + rng.normal(0.0, self.config.noise_std, size=t.shape)
        points: FloatArray = pos[None, :] + self.dirs[ok] * t[:, None]
        top: BoolArray = self.top_beam[ok]
        return points, top
