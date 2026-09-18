import math

import numpy as np
import pytest

from dh_lab.kinematics.dh_table import DHTable
from dh_lab.kinematics.units import AngleUnit, LengthUnit
from dh_lab.kinematics.transforms import (
    craig_dh_matrix,
    forward_kinematics,
    rot_z,
    trans_x,
    trans_z,
)


def _closed_form(alpha, a, d, theta):
    """The DH matrix spelled out exactly as the closed form in the
    module/spec docstring -- independent of craig_dh_matrix's own
    Rot_x@Trans_x@Rot_z@Trans_z construction, so comparing the two is a
    real cross-check, not a tautology."""
    c, s = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [c, -s, 0.0, a],
        [s * ca, c * ca, -sa, -sa * d],
        [s * sa, c * sa, ca, ca * d],
        [0.0, 0.0, 0.0, 1.0],
    ])


class TestCraigDHMatrix:
    def test_identity_row(self):
        np.testing.assert_allclose(craig_dh_matrix(0, 0, 0, 0), np.eye(4), atol=1e-12)

    def test_pure_rotation(self):
        np.testing.assert_allclose(
            craig_dh_matrix(0, 0, 0, math.pi / 2), rot_z(math.pi / 2), atol=1e-12
        )

    def test_pure_translation(self):
        np.testing.assert_allclose(
            craig_dh_matrix(0, 0.3, 0.2, 0), trans_x(0.3) @ trans_z(0.2), atol=1e-12
        )

    def test_nonzero_alpha_against_closed_form(self):
        alpha, a, d, theta = math.radians(37.0), 0.12, -0.05, math.radians(64.0)
        np.testing.assert_allclose(
            craig_dh_matrix(alpha, a, d, theta), _closed_form(alpha, a, d, theta), atol=1e-12
        )

    @pytest.mark.parametrize("alpha", [math.pi / 2, -math.pi / 2])
    def test_alpha_plus_minus_90_exactly(self, alpha):
        a, d, theta = 0.1, 0.2, math.radians(30)
        np.testing.assert_allclose(
            craig_dh_matrix(alpha, a, d, theta), _closed_form(alpha, a, d, theta), atol=1e-12
        )


class TestForwardKinematics:
    def test_identity_table_gives_identity_tool_pose(self):
        """All DH params hard-coded to zero (ignoring q) must give an
        identity tool pose regardless of the configuration commanded."""
        table = DHTable(joint_count=3)
        for i in range(3):
            table.set_cell(i, "theta", "0")
        rows, base, tool_t = table.evaluate({1: 1.7, 2: -0.4, 3: 2.9})
        result = forward_kinematics(rows, base, tool_t)
        np.testing.assert_allclose(result.tool.matrix, np.eye(4), atol=1e-12)

    def test_q_equals_zero(self):
        """The default table (theta_i = q_i, everything else 0) at q=0
        is identity -- and many real mistakes are invisible here, which
        is itself one of the tool's lessons (see the "zero test", §6)."""
        table = DHTable(joint_count=6)
        rows, base, tool_t = table.evaluate({i: 0.0 for i in range(1, 7)})
        result = forward_kinematics(rows, base, tool_t)
        np.testing.assert_allclose(result.tool.matrix, np.eye(4), atol=1e-12)

    def test_single_joint_pure_rotation(self):
        table = DHTable(joint_count=1)
        table.set_cell(0, "theta", "q1")
        rows, base, tool_t = table.evaluate({1: math.pi / 2})
        result = forward_kinematics(rows, base, tool_t)
        np.testing.assert_allclose(result.tool.matrix, rot_z(math.pi / 2), atol=1e-12)

    def test_single_joint_pure_translation(self):
        table = DHTable(joint_count=1)  # default length unit is mm
        table.set_cell(0, "d", "250")
        table.set_cell(0, "theta", "0")
        rows, base, tool_t = table.evaluate({1: 0.0})
        result = forward_kinematics(rows, base, tool_t)
        np.testing.assert_allclose(result.tool.matrix, trans_z(0.25), atol=1e-12)

    def test_two_link_planar_arm_against_closed_form(self):
        # Classic 2R planar arm, both joints rotating about the same Z:
        #   x = L1 cos(q1) + L2 cos(q1+q2), y = L1 sin(q1) + L2 sin(q1+q2)
        #
        # Craig's modified DH is "look-behind": row i's `a` is a_{i-1}
        # (the *previous* link's length), applied via Trans_x *before*
        # that row's Rot_z(theta_i). So L1 (link 1's length) belongs on
        # row 2's `a`, not row 1's -- row 1 has nothing before it (a=0).
        # L2 has no row of its own (there is no "row 3"); it's the fixed
        # tool transform, ^2T_tool = Trans_x(L2).
        table = DHTable(joint_count=2)
        table.set_units(AngleUnit.RADIANS, LengthUnit.METRES)
        table.set_cell(0, "theta", "q1")
        table.set_cell(1, "a", "1")  # L1, look-behind onto row 2
        table.set_cell(1, "theta", "q2")
        table.set_fixed_cell("tool", "x", "1")  # L2
        q1, q2 = math.radians(30), math.radians(45)
        rows, base, tool_t = table.evaluate({1: q1, 2: q2})
        result = forward_kinematics(rows, base, tool_t)
        expected_x = math.cos(q1) + math.cos(q1 + q2)
        expected_y = math.sin(q1) + math.sin(q1 + q2)
        assert result.tool.matrix[0, 3] == pytest.approx(expected_x, abs=1e-9)
        assert result.tool.matrix[1, 3] == pytest.approx(expected_y, abs=1e-9)

    def test_returns_every_intermediate_transform_not_just_tool(self):
        table = DHTable(joint_count=3)
        rows, base, tool_t = table.evaluate({1: 0.3, 2: -0.1, 3: 0.2})
        result = forward_kinematics(rows, base, tool_t)
        assert set(result.transforms) == {"0", "1", "2", "3", "tool"}
        # Every intermediate transform is anchored at the same base frame.
        assert all(t.frame_from == "base" for t in result.transforms.values())

    def test_joint_limits_at_boundary_stay_finite(self):
        table = DHTable(joint_count=1)
        table.set_cell(0, "theta", "q1")
        for q in (-math.pi, math.pi):
            rows, base, tool_t = table.evaluate({1: q})
            result = forward_kinematics(rows, base, tool_t)
            assert np.all(np.isfinite(result.tool.matrix))
