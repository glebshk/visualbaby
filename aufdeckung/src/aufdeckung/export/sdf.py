"""Scenario -> Gazebo Harmonic SDF world.

The world mirrors what the Python simulator uses: the same cage polygon,
obstacle primitives, targets and (visual-only) rain curtains. The header
(physics, PX4-required system plugins, NavSat origin) follows the visualbaby
worlds so PX4 SITL can attach to it with ``PX4_GZ_WORLD=<world name>``.
"""

from __future__ import annotations

import hashlib
import math
import xml.etree.ElementTree as ET
from pathlib import Path

from aufdeckung.config import ArenaConfig
from aufdeckung.geometry import Box, Cylinder, Shape, regular_polygon
from aufdeckung.scenario.model import Scenario

GROUND_COLORS = {
    "grass": (0.30, 0.50, 0.22),
    "asphalt": (0.22, 0.22, 0.24),
    "gravel": (0.55, 0.53, 0.50),
    "sand": (0.76, 0.70, 0.50),
}
FACADES = [(0.80, 0.76, 0.70), (0.62, 0.55, 0.48), (0.55, 0.60, 0.66), (0.85, 0.82, 0.78),
           (0.48, 0.40, 0.36), (0.70, 0.66, 0.58), (0.40, 0.44, 0.50)]
KIND_COLORS = {
    "vehicle": (0.15, 0.25, 0.55),
    "tree": (0.18, 0.42, 0.16),
    "pole": (0.30, 0.30, 0.32),
    "furniture": (0.45, 0.35, 0.25),
    "feature": (0.55, 0.75, 0.95),
}
TARGET_COLORS = {"class1": (0.85, 0.10, 0.10), "class2": (0.10, 0.35, 0.85), "special": (1.0, 0.55, 0.0)}
PX4_SYSTEMS = [
    ("gz::sim::systems::Physics", "gz-sim-physics-system"),
    ("gz::sim::systems::UserCommands", "gz-sim-user-commands-system"),
    ("gz::sim::systems::SceneBroadcaster", "gz-sim-scene-broadcaster-system"),
    ("gz::sim::systems::Contact", "gz-sim-contact-system"),
    ("gz::sim::systems::Imu", "gz-sim-imu-system"),
    ("gz::sim::systems::AirPressure", "gz-sim-air-pressure-system"),
    ("gz::sim::systems::Magnetometer", "gz-sim-magnetometer-system"),
    ("gz::sim::systems::ApplyLinkWrench", "gz-sim-apply-link-wrench-system"),
    ("gz::sim::systems::NavSat", "gz-sim-navsat-system"),
]


def _sub(parent: ET.Element, tag: str, text: str | None = None, **attrib: str) -> ET.Element:
    el = ET.SubElement(parent, tag, attrib)
    if text is not None:
        el.text = text
    return el


def _f(v: float) -> str:
    """Compact number formatting: 4 decimals, trailing zeros dropped, never "-0"."""
    text = f"{v:.4f}".rstrip("0").rstrip(".")
    return "0" if text in ("", "-0") else text


def _pose(x: float, y: float, z: float, yaw: float = 0.0) -> str:
    return f"{_f(x)} {_f(y)} {_f(z)} 0 0 {_f(yaw)}"


def _material(parent: ET.Element, rgb: tuple[float, float, float], alpha: float = 1.0) -> None:
    mat = _sub(parent, "material")
    col = " ".join(_f(c) for c in rgb) + f" {_f(alpha)}"
    _sub(mat, "ambient", col)
    _sub(mat, "diffuse", col)
    _sub(mat, "specular", "0.1 0.1 0.1 1")


def _geometry(parent: ET.Element, shape: Shape) -> None:
    geo = _sub(parent, "geometry")
    if isinstance(shape, Box):
        b = _sub(geo, "box")
        _sub(b, "size", f"{_f(shape.sx)} {_f(shape.sy)} {_f(shape.height)}")
    else:
        c = _sub(geo, "cylinder")
        _sub(c, "radius", _f(shape.radius))
        _sub(c, "length", _f(shape.height))


def _color_for(shape: Shape) -> tuple[float, float, float]:
    if shape.kind == "building":
        digest = int(hashlib.md5(shape.name.encode()).hexdigest(), 16)
        return FACADES[digest % len(FACADES)]
    return KIND_COLORS.get(shape.kind, (0.6, 0.6, 0.6))


def _static_model(world: ET.Element, name: str, shape: Shape, rgb: tuple[float, float, float],
                  collide: bool = True, alpha: float = 1.0) -> None:
    model = _sub(world, "model", name=name)
    _sub(model, "static", "true")
    yaw = shape.yaw if isinstance(shape, Box) else 0.0
    _sub(model, "pose", _pose(shape.x, shape.y, shape.z0 + shape.height / 2, yaw))
    link = _sub(model, "link", name="link")
    if collide:
        _geometry(_sub(link, "collision", name="collision"), shape)
    vis = _sub(link, "visual", name="visual")
    _geometry(vis, shape)
    _material(vis, rgb, alpha)
    if alpha < 1.0:
        _sub(vis, "transparency", _f(1.0 - alpha))


def scenario_to_sdf(sc: Scenario, arena: ArenaConfig, world_name: str = "aufdeckung") -> str:
    root = ET.Element("sdf", version="1.10")
    world = _sub(root, "world", name=world_name)
    physics = _sub(world, "physics", type="ode")
    _sub(physics, "max_step_size", "0.004")
    _sub(physics, "real_time_factor", "1.0")
    _sub(physics, "real_time_update_rate", "250")
    for name, filename in PX4_SYSTEMS:
        _sub(world, "plugin", name=name, filename=filename)
    sensors = _sub(world, "plugin", name="gz::sim::systems::Sensors", filename="gz-sim-sensors-system")
    _sub(sensors, "render_engine", "ogre2")
    _sub(world, "gravity", "0 0 -9.8")
    _sub(world, "magnetic_field", "6e-06 2.3e-05 -4.2e-05")
    _sub(world, "atmosphere", type="adiabatic")
    scene = _sub(world, "scene")
    _sub(scene, "grid", "false")
    _sub(scene, "ambient", "0.4 0.4 0.4 1")
    _sub(scene, "background", "0.7 0.8 0.9 1")
    _sub(scene, "shadows", "true")
    sph = _sub(world, "spherical_coordinates")
    _sub(sph, "surface_model", "EARTH_WGS84")
    _sub(sph, "world_frame_orientation", "ENU")
    _sub(sph, "latitude_deg", repr(arena.origin_lat))
    _sub(sph, "longitude_deg", repr(arena.origin_lon))
    _sub(sph, "elevation", _f(arena.origin_alt))
    _sub(sph, "heading_deg", "0")

    sun = _sub(world, "light", type="directional", name="sun")
    _sub(sun, "cast_shadows", "true")
    _sub(sun, "pose", "0 0 50 0 0 0")
    _sub(sun, "diffuse", "0.9 0.9 0.9 1")
    _sub(sun, "specular", "0.2 0.2 0.2 1")
    _sub(sun, "direction", "-0.4 0.3 -0.9")

    ground = _sub(world, "model", name="ground_plane")
    _sub(ground, "static", "true")
    link = _sub(ground, "link", name="link")
    size = _f(arena.diameter * 2)
    col = _sub(_sub(_sub(link, "collision", name="collision"), "geometry"), "plane")
    _sub(col, "normal", "0 0 1")
    _sub(col, "size", f"{size} {size}")
    vis = _sub(link, "visual", name="visual")
    plane = _sub(_sub(vis, "geometry"), "plane")
    _sub(plane, "normal", "0 0 1")
    _sub(plane, "size", f"{size} {size}")
    _material(vis, GROUND_COLORS.get(sc.ground, GROUND_COLORS["grass"]))

    _add_cage(world, arena)
    for i, shape in enumerate(sc.obstacles):
        _static_model(world, shape.name or f"{shape.kind}_{i}", shape, _color_for(shape))
    for i, tgt in enumerate(sc.targets):
        h = max(tgt.size * 0.6, 0.15)
        box = Box(tgt.x, tgt.y, tgt.size, tgt.size, h, kind="target", name=tgt.name)
        _static_model(world, f"target_{tgt.name or i}", box, TARGET_COLORS[tgt.cls])
    for i, zone in enumerate(sc.rain_zones):
        cyl = Cylinder(zone.x, zone.y, zone.radius, arena.height, kind="rain", name=f"rain_{i}")
        _static_model(world, f"rain_zone_{i}", cyl, (0.4, 0.6, 1.0), collide=False, alpha=0.25)
    comment = ET.Comment(
        f" generated by aufdeckung from scenario {sc.name!r} (preset {sc.preset}, seed {sc.seed}); "
        f"rain zones are visual only: active windows "
        + ", ".join(f"#{i} {z.t_on:.0f}-{z.t_off:.0f}s" for i, z in enumerate(sc.rain_zones))
        + " "
    )
    world.insert(0, comment)
    ET.indent(root)
    return '<?xml version="1.0" ?>\n' + ET.tostring(root, encoding="unicode") + "\n"


def _add_cage(world: ET.Element, arena: ArenaConfig) -> None:
    poly = regular_polygon(arena.diameter, arena.n_sides)
    n = len(poly)
    for i in range(n):
        x, y = poly[i]
        pole = Cylinder(float(x), float(y), 0.15, arena.height, kind="cage", name=f"cage_pole_{i}")
        _static_model(world, f"cage_pole_{i}", pole, (0.85, 0.85, 0.88))
        x2, y2 = poly[(i + 1) % n]
        length = math.hypot(x2 - x, y2 - y)
        net = Box(float((x + x2) / 2), float((y + y2) / 2), length, 0.02, arena.height,
                  yaw=math.atan2(y2 - y, x2 - x), kind="net", name=f"cage_net_{i}")
        _static_model(world, f"cage_net_{i}", net, (0.2, 0.2, 0.2), alpha=0.15)
    ceiling = Cylinder(0.0, 0.0, arena.diameter / 2, 0.02, z0=arena.height, kind="net", name="ceiling")
    _static_model(world, "cage_ceiling", ceiling, (0.2, 0.2, 0.2), alpha=0.1)


def write_world(sc: Scenario, arena: ArenaConfig, path: str | Path, world_name: str = "aufdeckung"
                ) -> Path:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(scenario_to_sdf(sc, arena, world_name), encoding="utf-8")
    return out
