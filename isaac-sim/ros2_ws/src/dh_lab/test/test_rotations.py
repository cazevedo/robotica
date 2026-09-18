import math

import numpy as np
import pytest

from dh_lab.kinematics.rotations import quaternion_to_rotation_matrix, rotation_matrix_to_quaternion
from dh_lab.kinematics.transforms import rot_x, rot_z


@pytest.mark.parametrize(
    "angle,axis_matrix",
    [
        (0.0, lambda a: np.eye(3)),
        (math.pi / 2, lambda a: rot_z(a)[:3, :3]),
        (math.pi, lambda a: rot_z(a)[:3, :3]),
        (math.radians(12.3), lambda a: rot_x(a)[:3, :3]),
    ],
)
def test_matrix_quaternion_roundtrip(angle, axis_matrix):
    r = axis_matrix(angle)
    q = rotation_matrix_to_quaternion(r)
    back = quaternion_to_rotation_matrix(q)
    np.testing.assert_allclose(back, r, atol=1e-9)


def test_identity_quaternion_is_w_one():
    q = rotation_matrix_to_quaternion(np.eye(3))
    np.testing.assert_allclose(q, [1.0, 0.0, 0.0, 0.0], atol=1e-12)


def test_unnormalized_quaternion_input_is_normalized():
    q = np.array([2.0, 0.0, 0.0, 0.0])  # unnormalised "identity"
    r = quaternion_to_rotation_matrix(q)
    np.testing.assert_allclose(r, np.eye(3), atol=1e-12)
