"""Unit handling for the DH Lab kinematics core.

Internal storage and computation is always SI (metres, radians). The two
unit toggles below only affect how *bare numeric literals* in student-
entered DH cell expressions are interpreted, and how values are displayed
in the GUI -- never how ``q1..qN`` (always radians internally) or ``pi``
(always the dimensionless mathematical constant) are treated. See
:mod:`.expressions` and :mod:`.dh_table` for where that rule is applied.
"""
from __future__ import annotations

import math
from enum import Enum


class AngleUnit(Enum):
    RADIANS = "rad"
    DEGREES = "deg"


class LengthUnit(Enum):
    METRES = "m"
    MILLIMETRES = "mm"


def angle_literal_to_si(value: float, unit: AngleUnit) -> float:
    """Convert a bare numeric literal from an angle-typed DH cell
    (alpha or theta) to radians, per the current angle-unit toggle."""
    if unit is AngleUnit.DEGREES:
        return math.radians(value)
    return value


def length_literal_to_si(value: float, unit: LengthUnit) -> float:
    """Convert a bare numeric literal from a length-typed DH cell
    (a or d) to metres, per the current length-unit toggle."""
    if unit is LengthUnit.MILLIMETRES:
        return value / 1000.0
    return value


def si_angle_to_display(value_rad: float, unit: AngleUnit) -> float:
    """Convert an internal SI angle (radians) to the unit shown in the GUI."""
    if unit is AngleUnit.DEGREES:
        return math.degrees(value_rad)
    return value_rad


def si_length_to_display(value_m: float, unit: LengthUnit) -> float:
    """Convert an internal SI length (metres) to the unit shown in the GUI."""
    if unit is LengthUnit.MILLIMETRES:
        return value_m * 1000.0
    return value_m
