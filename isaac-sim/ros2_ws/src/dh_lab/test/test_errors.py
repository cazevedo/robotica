import math

import numpy as np
import pytest

from dh_lab.kinematics.errors import orientation_error, position_error
from dh_lab.kinematics.transforms import rot_z, trans_x


def test_zero_error_identical_poses():
    m = trans_x(0.3) @ rot_z(math.radians(40))
    assert position_error(m, m).total_mm == pytest.approx(0.0, abs=1e-9)
    assert orientation_error(m, m).angle_deg == pytest.approx(0.0, abs=1e-9)


def test_known_90_degree_rotation():
    predicted = rot_z(math.pi / 2)
    ground_truth = np.eye(4)
    assert orientation_error(predicted, ground_truth).angle_deg == pytest.approx(90.0, abs=1e-9)


def test_position_error_per_axis_and_total():
    predicted = np.eye(4)
    predicted[:3, 3] = [0.001, 0.002, -0.003]  # metres
    ground_truth = np.eye(4)
    pos = position_error(predicted, ground_truth)
    assert pos.x_mm == pytest.approx(1.0)
    assert pos.y_mm == pytest.approx(2.0)
    assert pos.z_mm == pytest.approx(-3.0)
    assert pos.total_mm == pytest.approx(math.sqrt(1 + 4 + 9))


def test_orientation_stability_at_tiny_angle():
    # 1e-9 rad must not come back as NaN or a false 0 -- this is exactly
    # why atan2-on-quaternion is used instead of arccos(trace).
    tiny = 1e-9
    ori = orientation_error(rot_z(tiny), np.eye(4))
    assert math.isfinite(ori.angle_deg)
    assert ori.angle_deg > 0.0
    assert ori.angle_deg == pytest.approx(math.degrees(tiny), rel=1e-3)


def test_orientation_at_180_degrees():
    ori = orientation_error(rot_z(math.pi), np.eye(4))
    assert ori.angle_deg == pytest.approx(180.0, abs=1e-6)


def test_orientation_error_is_symmetric_in_magnitude():
    predicted = rot_z(math.radians(12.3))
    ground_truth = np.eye(4)
    forward = orientation_error(predicted, ground_truth).angle_deg
    backward = orientation_error(ground_truth, predicted).angle_deg
    assert forward == pytest.approx(backward, abs=1e-9)
