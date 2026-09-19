"""Parameterised world assets (SDF model generators) and the visual design language.

Every generator returns a <model> string with collision geometry that matches the
visual geometry, so what you see is what the physics simulates. Colours come from a
single palette so the world reads as one place (brief 3b §5).
"""
from .palette import PALETTE, material
from .library import (building, warehouse, tower, bridge, platform_static, wall, fence, container_stack,
                      landing_pad, road, pole, barrier, ramp, terrace, tank, apron, marking, mast_light)

__all__ = ["PALETTE", "material", "building", "warehouse", "tower", "bridge", "platform_static", "wall", "fence",
           "container_stack", "landing_pad", "road", "pole", "barrier", "ramp", "terrace", "tank", "apron", "marking",
           "mast_light"]
