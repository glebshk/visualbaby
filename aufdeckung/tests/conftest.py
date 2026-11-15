from __future__ import annotations

import pytest

from aufdeckung.config import Config
from aufdeckung.geometry import Box, Cylinder
from aufdeckung.mapping.grid import GridSpec
from aufdeckung.scenario.model import Scenario, Target


@pytest.fixture
def cfg() -> Config:
    return Config()


@pytest.fixture
def spec(cfg: Config) -> GridSpec:
    return GridSpec.for_arena(cfg.arena, cfg.planner.grid_res)


@pytest.fixture
def simple_scenario() -> Scenario:
    """Hand-made arena: one low building (fly over), one ceiling tower (go around), a tree."""
    return Scenario(
        name="simple",
        obstacles=(
            Box(-8.0, 4.0, 6.0, 6.0, 6.0, name="low_building"),
            Box(8.0, 2.0, 5.0, 5.0, 13.0, name="tower"),
            Cylinder(0.0, 10.0, 0.2, 2.5, kind="tree", name="tree_trunk"),
            Cylinder(0.0, 10.0, 1.5, 3.0, z0=2.5, kind="tree", name="tree_crown"),
        ),
        targets=(
            Target(-14.0, -6.0, "class1", "t1"),
            Target(14.0, -8.0, "class2", "t2"),
            Target(3.0, 15.0, "special", "t3"),
        ),
        start=(0.0, -18.0),
    )
