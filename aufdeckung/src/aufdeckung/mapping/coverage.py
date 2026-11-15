"""Camera coverage bookkeeping."""

from __future__ import annotations

import numpy as np

from aufdeckung.geometry import BoolArray
from aufdeckung.mapping.grid import GridSpec


class CoverageMap:
    """Which cells have been imaged (first time stamp kept per cell)."""

    def __init__(self, spec: GridSpec) -> None:
        self.spec = spec
        self.first_seen = np.full(spec.shape, np.inf)

    @property
    def covered(self) -> BoolArray:
        result: BoolArray = np.isfinite(self.first_seen)
        return result

    def mark(self, mask: BoolArray, t: float) -> int:
        new = mask & ~self.covered
        self.first_seen[new] = t
        return int(np.count_nonzero(new))

    def fraction(self, of: BoolArray) -> float:
        total = int(np.count_nonzero(of))
        return 1.0 if total == 0 else float(np.count_nonzero(self.covered & of)) / total

    def fraction_at(self, of: BoolArray, t: float) -> float:
        total = int(np.count_nonzero(of))
        return 1.0 if total == 0 else float(np.count_nonzero((self.first_seen <= t) & of)) / total
