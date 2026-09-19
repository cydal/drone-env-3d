"""Reusable, parameterised environment assets. All functions return one <model> string.

Conventions: poses are model poses in the world (x y z yaw); every visual has a matching
collision unless marked `visual_only` (thin ground markings/roads use a slightly raised
visual and no collision so they never trip contact sensors).
"""
from __future__ import annotations

import math

from .palette import material


def _model(name: str, x: float, y: float, z: float, yaw: float, body: str, *, static: bool = True) -> str:
    return f"""
    <model name="{name}">
      <static>{"true" if static else "false"}</static>
      <pose>{x} {y} {z} 0 0 {yaw}</pose>
      <link name="link">{body}
      </link>
    </model>"""


def _box(name: str, sx: float, sy: float, sz: float, mat: str, *, pose="0 0 0 0 0 0", collide: bool = True, emissive=None) -> str:
    geo = f"<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>"
    out = f"""
        <visual name="{name}"><pose>{pose}</pose>{geo}{material(mat, emissive=emissive)}</visual>"""
    if collide:
        out += f"""
        <collision name="{name}_c"><pose>{pose}</pose>{geo}</collision>"""
    return out


def _cyl(name: str, r: float, h: float, mat: str, *, pose="0 0 0 0 0 0", collide: bool = True, emissive=None) -> str:
    geo = f"<geometry><cylinder><radius>{r:.3f}</radius><length>{h:.3f}</length></cylinder></geometry>"
    out = f"""
        <visual name="{name}"><pose>{pose}</pose>{geo}{material(mat, emissive=emissive)}</visual>"""
    if collide:
        out += f"""
        <collision name="{name}_c"><pose>{pose}</pose>{geo}</collision>"""
    return out


# ---------------------------------------------------------------- buildings & structures

def building(name: str, x: float, y: float, w: float, d: float, h: float, yaw: float = 0.0, *,
             style: str = "facade", bands: bool = True, helipad: bool = False) -> str:
    """Rectangular building; optional dark glass bands every 4 m and a rooftop helipad."""
    body = _box("core", w, d, h, style, pose=f"0 0 {h / 2} 0 0 0")
    if bands:
        for k in range(1, int(h // 4)):
            body += _box(f"band{k}", w + 0.06, d + 0.06, 0.6, "facade_dk", pose=f"0 0 {k * 4:.2f} 0 0 0", collide=False)
    body += _box("roof", w - 0.4, d - 0.4, 0.3, "roof", pose=f"0 0 {h + 0.15} 0 0 0")
    if helipad:
        r = min(w, d) * 0.35
        body += _cyl("helipad", r, 0.05, "concrete_dk", pose=f"0 0 {h + 0.33} 0 0 0", collide=False)
        body += _cyl("helipad_ring", r * 0.85, 0.06, "accent", pose=f"0 0 {h + 0.34} 0 0 0", collide=False)
        body += _cyl("helipad_core", r * 0.7, 0.07, "concrete_dk", pose=f"0 0 {h + 0.35} 0 0 0", collide=False)
    return _model(name, x, y, 0, yaw, body)


def warehouse(name: str, x: float, y: float, w: float, d: float, h: float, yaw: float = 0.0) -> str:
    """Long low industrial hall with a ridge roof strip and a loading dock."""
    body = _box("hall", w, d, h, "steel", pose=f"0 0 {h / 2} 0 0 0")
    body += _box("ridge", w * 0.9, 1.2, 0.8, "steel_dk", pose=f"0 0 {h + 0.4} 0 0 0")
    body += _box("stripe", w + 0.05, d + 0.05, 0.5, "accent", pose=f"0 0 {h * 0.75} 0 0 0", collide=False)
    body += _box("dock", w * 0.6, 3.0, 1.2, "concrete_dk", pose=f"0 {-(d / 2 + 1.5)} 0.6 0 0 0")
    return _model(name, x, y, 0, yaw, body)


def tower(name: str, x: float, y: float, h: float, r: float = 1.5, *, cabin: bool = True) -> str:
    """Control / comms tower: mast + cabin + beacon."""
    body = _cyl("mast", r, h, "concrete_dk", pose=f"0 0 {h / 2} 0 0 0")
    if cabin:
        body += _box("cabin", r * 5, r * 5, 3.0, "glass", pose=f"0 0 {h + 1.5} 0 0 0")
        body += _box("cabin_roof", r * 5.4, r * 5.4, 0.3, "roof", pose=f"0 0 {h + 3.15} 0 0 0")
    body += _cyl("beacon", 0.25, 0.5, "beacon", pose=f"0 0 {h + (3.6 if cabin else 0.3)} 0 0 0", collide=False, emissive="0.6 0.1 0.1 1")
    return _model(name, x, y, 0, 0, body)


def bridge(name: str, x: float, y: float, span: float, height: float, width: float = 6.0, yaw: float = 0.0) -> str:
    """Deck on two piers; the space below stays clear for traffic."""
    body = _box("deck", span, width, 0.6, "concrete", pose=f"0 0 {height} 0 0 0")
    body += _box("rail_l", span, 0.15, 1.0, "steel_dk", pose=f"0 {width / 2 - 0.1} {height + 0.8} 0 0 0")
    body += _box("rail_r", span, 0.15, 1.0, "steel_dk", pose=f"0 {-width / 2 + 0.1} {height + 0.8} 0 0 0")
    for i, px in enumerate((-span / 2 + 3, span / 2 - 3)):
        body += _box(f"pier{i}", 2.0, width * 0.6, height - 0.3, "concrete_dk", pose=f"{px} 0 {(height - 0.3) / 2} 0 0 0")
    return _model(name, x, y, 0, yaw, body)


def platform_static(name: str, x: float, y: float, w: float, d: float, h: float, yaw: float = 0.0) -> str:
    """Elevated platform on legs (rooftop-like) with an edge marking."""
    body = _box("deck", w, d, 0.4, "concrete", pose=f"0 0 {h} 0 0 0")
    body += _box("edge", w + 0.1, d + 0.1, 0.1, "accent", pose=f"0 0 {h + 0.25} 0 0 0", collide=False)
    for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        body += _box(f"leg{i}", 0.6, 0.6, h, "steel_dk", pose=f"{sx * (w / 2 - 0.6)} {sy * (d / 2 - 0.6)} {h / 2} 0 0 0")
    return _model(name, x, y, 0, yaw, body)


def wall(name: str, x: float, y: float, length: float, height: float, yaw: float = 0.0, thickness: float = 0.5) -> str:
    return _model(name, x, y, 0, yaw, _box("wall", length, thickness, height, "concrete", pose=f"0 0 {height / 2} 0 0 0"))


def fence(name: str, x: float, y: float, length: float, yaw: float = 0.0, height: float = 2.4) -> str:
    """Semi-transparent security fence with posts (collides)."""
    body = _box("mesh", length, 0.08, height, "fence", pose=f"0 0 {height / 2} 0 0 0")
    n = max(2, int(length // 4))
    for i in range(n + 1):
        px = -length / 2 + i * length / n
        body += _box(f"post{i}", 0.15, 0.15, height + 0.3, "steel_dk", pose=f"{px:.2f} 0 {(height + 0.3) / 2} 0 0 0", collide=False)
    return _model(name, x, y, 0, yaw, body)


def container_stack(name: str, x: float, y: float, yaw: float = 0.0, layout=((0, 0, "container_a"), (1, 0, "container_b"), (0, 1, "container_c"))) -> str:
    """Shipping containers 6.1 x 2.44 x 2.6; layout entries are (row, level, colour)."""
    body = ""
    for i, (row, level, col) in enumerate(layout):
        body += _box(f"c{i}", 6.1, 2.44, 2.6, col, pose=f"0 {row * 2.6:.2f} {1.3 + level * 2.6:.2f} 0 0 0")
    return _model(name, x, y, 0, yaw, body)


def landing_pad(name: str, x: float, y: float, r: float = 4.0, *, label_color: str = "accent") -> str:
    """Raised pad with ring + H marking. Collides (drones land on it)."""
    body = _cyl("pad", r, 0.12, "concrete_dk", pose="0 0 0.06 0 0 0")
    body += _cyl("ring", r * 0.9, 0.02, label_color, pose="0 0 0.13 0 0 0", collide=False)
    body += _cyl("inner", r * 0.78, 0.03, "concrete_dk", pose="0 0 0.135 0 0 0", collide=False)
    body += _box("h1", r * 0.12, r * 0.7, 0.02, "marking", pose=f"{-r * 0.25:.2f} 0 0.15 0 0 0", collide=False)
    body += _box("h2", r * 0.12, r * 0.7, 0.02, "marking", pose=f"{r * 0.25:.2f} 0 0.15 0 0 0", collide=False)
    body += _box("h3", r * 0.5, r * 0.12, 0.02, "marking", pose="0 0 0.15 0 0 0", collide=False)
    for i in range(4):
        a = i * math.pi / 2 + math.pi / 4
        body += _cyl(f"lamp{i}", 0.12, 0.3, "accent2", pose=f"{r * 1.05 * math.cos(a):.2f} {r * 1.05 * math.sin(a):.2f} 0.15 0 0 0", emissive="0.1 0.4 0.6 1")
    return _model(name, x, y, 0, 0, body)


def road(name: str, x: float, y: float, length: float, yaw: float = 0.0, width: float = 8.0, *, dashed: bool = True) -> str:
    """Visual-only road strip (no collision: it must not register as an obstacle)."""
    body = _box("asphalt", length, width, 0.04, "asphalt", pose="0 0 0.02 0 0 0", collide=False)
    if dashed:
        n = int(length // 6)
        for i in range(n):
            px = -length / 2 + 3 + i * 6
            body += _box(f"dash{i}", 3.0, 0.2, 0.01, "marking", pose=f"{px:.2f} 0 0.045 0 0 0", collide=False)
    body += _box("edge_l", length, 0.25, 0.01, "marking", pose=f"0 {width / 2 - 0.2:.2f} 0.045 0 0 0", collide=False)
    body += _box("edge_r", length, 0.25, 0.01, "marking", pose=f"0 {-width / 2 + 0.2:.2f} 0.045 0 0 0", collide=False)
    return _model(name, x, y, 0, yaw, body)


def apron(name: str, x: float, y: float, w: float, d: float, yaw: float = 0.0) -> str:
    """Concrete apron / staging surface (visual only, marked bays)."""
    body = _box("slab", w, d, 0.03, "apron", pose="0 0 0.015 0 0 0", collide=False)
    n = max(1, int(w // 8))
    for i in range(n + 1):
        px = -w / 2 + i * w / n
        body += _box(f"bay{i}", 0.2, d * 0.9, 0.01, "marking", pose=f"{px:.2f} 0 0.035 0 0 0", collide=False)
    return _model(name, x, y, 0, yaw, body)


def marking(name: str, x: float, y: float, w: float, d: float, yaw: float = 0.0, color: str = "accent") -> str:
    return _model(name, x, y, 0, yaw, _box("mark", w, d, 0.015, color, pose="0 0 0.05 0 0 0", collide=False))


def pole(name: str, x: float, y: float, h: float = 6.0, r: float = 0.12) -> str:
    return _model(name, x, y, 0, 0, _cyl("pole", r, h, "steel_dk", pose=f"0 0 {h / 2} 0 0 0"))


def mast_light(name: str, x: float, y: float, h: float = 12.0) -> str:
    body = _cyl("mast", 0.18, h, "steel_dk", pose=f"0 0 {h / 2} 0 0 0")
    body += _box("head", 1.6, 0.6, 0.3, "white", pose=f"0 0 {h + 0.15} 0 0 0", collide=False, emissive="0.5 0.5 0.45 1")
    return _model(name, x, y, 0, 0, body)


def barrier(name: str, x: float, y: float, length: float = 3.0, yaw: float = 0.0) -> str:
    """Jersey barrier segment (stripes)."""
    body = _box("base", length, 0.6, 0.9, "concrete", pose="0 0 0.45 0 0 0")
    body += _box("stripe", length + 0.02, 0.62, 0.15, "accent", pose="0 0 0.7 0 0 0", collide=False)
    return _model(name, x, y, 0, yaw, body)


def tank(name: str, x: float, y: float, r: float = 4.0, h: float = 8.0) -> str:
    body = _cyl("tank", r, h, "tank", pose=f"0 0 {h / 2} 0 0 0")
    body += _cyl("cap", r * 0.6, 0.6, "steel_dk", pose=f"0 0 {h + 0.3} 0 0 0")
    body += _cyl("band", r + 0.03, 0.4, "accent2", pose=f"0 0 {h * 0.6} 0 0 0", collide=False)
    return _model(name, x, y, 0, 0, body)


# ---------------------------------------------------------------- terrain (collision == visual)

def ramp(name: str, x: float, y: float, length: float, width: float, rise: float, yaw: float = 0.0) -> str:
    """Sloped slab: an embankment / ramp. Tilted box, so physics and visuals agree exactly."""
    pitch = -math.atan2(rise, length)
    hyp = math.hypot(length, rise)
    body = _box("slab", hyp, width, 0.5, "hill", pose=f"0 0 -0.25 0 {pitch:.4f} 0")
    return _model(name, x, y, rise / 2, yaw, body)


def terrace(name: str, x: float, y: float, radii: tuple[float, ...] = (30.0, 22.0, 14.0), step: float = 3.0) -> str:
    """A hill made of stacked plateaus (each level flat and walkable/landable)."""
    body = ""
    for i, r in enumerate(radii):
        body += _cyl(f"level{i}", r, step, "hill", pose=f"0 0 {step * (i + 0.5):.2f} 0 0 0")
    body += _cyl("top_mark", radii[-1] * 0.25, 0.02, "marking", pose=f"0 0 {step * len(radii) + 0.01:.2f} 0 0 0", collide=False)
    return _model(name, x, y, 0, 0, body)
