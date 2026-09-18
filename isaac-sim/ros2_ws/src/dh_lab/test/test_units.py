import math

import pytest

from dh_lab.kinematics.units import (
    AngleUnit,
    LengthUnit,
    angle_literal_to_si,
    length_literal_to_si,
    si_angle_to_display,
    si_length_to_display,
)


def test_degree_literal_to_radians():
    # "a theta literal of 90 in degree mode equals pi/2 internally"
    assert angle_literal_to_si(90.0, AngleUnit.DEGREES) == pytest.approx(math.pi / 2)


def test_radian_literal_passthrough():
    assert angle_literal_to_si(1.5708, AngleUnit.RADIANS) == pytest.approx(1.5708)


def test_millimetre_literal_to_metres():
    assert length_literal_to_si(1000.0, LengthUnit.MILLIMETRES) == pytest.approx(1.0)


def test_metre_literal_passthrough():
    assert length_literal_to_si(0.5, LengthUnit.METRES) == pytest.approx(0.5)


def test_display_roundtrip_degrees():
    original = math.pi / 3
    literal = si_angle_to_display(original, AngleUnit.DEGREES)
    assert angle_literal_to_si(literal, AngleUnit.DEGREES) == pytest.approx(original)


def test_display_roundtrip_millimetres():
    original = 0.238
    literal = si_length_to_display(original, LengthUnit.MILLIMETRES)
    assert length_literal_to_si(literal, LengthUnit.MILLIMETRES) == pytest.approx(original)
