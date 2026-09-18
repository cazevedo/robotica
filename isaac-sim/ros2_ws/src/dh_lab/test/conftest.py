"""Shared fixtures: a synthetic 3-joint reference DH table and a
*matching* ChainDescription built with exactly equivalent geometry (so
the reference table has ~zero error against the chain for any q). Used
by the sweep and batch diagnostic tests, which inject faults into a
known-correct table and need a ground truth that agrees with the
unperturbed version.

Geometry is deliberately chosen so every one of the four sweep-shape
signatures in the spec has something to come from:
  row1: alpha0=0,       a0=0      -- plain
  row2: alpha1=90 deg,  a1=0.2 m  -- nonzero alpha, upstream of joint2's own rotation
  row3: alpha2=0,       a2=0.25 m -- nonzero a, for the "wrong a/d" case
row2's theta also carries a constant offset (q2 + a fixed angle) so a
"sign error on joint direction" (negating only the q2 term) is
distinguishable from negating the whole expression.
"""
import math

import pytest

from dh_lab.config.robot_config import RobotConfig
from dh_lab.ground_truth.chain import ChainDescription, FixedOffset, JointSpec
from dh_lab.kinematics.dh_table import DHTable
from dh_lab.kinematics.rotations import rotation_matrix_to_quaternion
from dh_lab.kinematics.transforms import rot_x, rot_z, trans_x
from dh_lab.kinematics.units import AngleUnit, LengthUnit

ALPHA_PREV = [0.0, math.pi / 2, 0.0]
A_PREV = [0.0, 0.2, 0.25]
THETA2_OFFSET_RAD = -math.pi / 4  # row2: theta2 = q2 + THETA2_OFFSET_RAD
THETA_OFFSET_RAD = [0.0, THETA2_OFFSET_RAD, 0.0]

def build_table(alpha_prev=None, a_prev=None, theta2_sign=1.0) -> DHTable:
    """The reference table by default; pass overrides to inject a fault
    into exactly one parameter while leaving the rest matching the
    ground-truth chain built by :func:`build_chain`."""
    alpha_prev = ALPHA_PREV if alpha_prev is None else alpha_prev
    a_prev = A_PREV if a_prev is None else a_prev
    table = DHTable(joint_count=3)
    table.set_units(AngleUnit.RADIANS, LengthUnit.METRES)
    for i in range(3):
        table.set_cell(i, "alpha", repr(alpha_prev[i]))
        table.set_cell(i, "a", repr(a_prev[i]))
        table.set_cell(i, "d", "0")
        if i == 1:  # row2 / joint2: theta2 = (+/-)q2 + offset
            sign = "" if theta2_sign > 0 else "-"
            table.set_cell(i, "theta", f"{sign}q2 + ({THETA2_OFFSET_RAD!r})")
        else:
            table.set_cell(i, "theta", f"q{i + 1}")
    assert table.is_valid, [
        (name, getattr(row, name).error)
        for row in table.rows for name in ("alpha", "a", "d", "theta")
        if not getattr(row, name).is_valid
    ]
    return table


def build_chain(alpha_prev=None, a_prev=None) -> ChainDescription:
    """Ground truth matching :func:`build_table`'s *default* (fault-free)
    geometry -- always built from the nominal ALPHA_PREV/A_PREV, since
    faults are injected into the table under test, never into the truth
    it's compared against."""
    alpha_prev = ALPHA_PREV if alpha_prev is None else alpha_prev
    a_prev = A_PREV if a_prev is None else a_prev
    joints = []
    prev_link = "base"
    for i in range(3):
        m = rot_x(alpha_prev[i]) @ trans_x(a_prev[i])
        parent_frame = FixedOffset(
            translation=tuple(float(c) for c in m[:3, 3]),
            quaternion_wxyz=tuple(float(c) for c in rotation_matrix_to_quaternion(m[:3, :3])),
        )
        # A constant theta offset ("theta_i = q_i + offset") is encoded
        # via the CHILD frame's own fixed rotation, not a separate field:
        # _joint_transform computes parent_frame @ Rot(axis,q) @
        # inverse(child_frame), so child_frame = Rot(axis,-offset) makes
        # the effective rotation Rot(axis, q+offset). This is also a
        # real exercise of the two-frame joint math (see analytic_fk.py)
        # rather than a test-only shortcut.
        child_rotation = rot_z(-THETA_OFFSET_RAD[i])[:3, :3]
        child_frame = FixedOffset(
            translation=(0.0, 0.0, 0.0),
            quaternion_wxyz=tuple(float(c) for c in rotation_matrix_to_quaternion(child_rotation)),
        )
        child = f"link{i + 1}"
        joints.append(JointSpec(
            name=f"joint{i + 1}", parent_link=prev_link, child_link=child,
            joint_type="revolute", axis=(0.0, 0.0, 1.0),
            lower_limit=-math.pi, upper_limit=math.pi,
            parent_frame=parent_frame, child_frame=child_frame,
        ))
        prev_link = child
    return ChainDescription(robot_prim_path="/World/test", base_link="base", tool_link="link3", joints=joints)


@pytest.fixture
def robot_config() -> RobotConfig:
    return RobotConfig(joint_names=["joint1", "joint2", "joint3"])


@pytest.fixture
def joint_limits() -> dict:
    return {"joint1": (-math.pi, math.pi), "joint2": (-math.pi, math.pi), "joint3": (-math.pi, math.pi)}


@pytest.fixture
def reference_setup(robot_config):
    """(table, chain, config), matching by construction."""
    return build_table(), build_chain(), robot_config
