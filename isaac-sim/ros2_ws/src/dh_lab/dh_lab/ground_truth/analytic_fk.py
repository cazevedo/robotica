"""Analytic forward kinematics evaluated directly from an extracted
:class:`~dh_lab.ground_truth.chain.ChainDescription` -- this is the
tool's ground truth, built independently of any DH table (Craig or
otherwise). Pure Python + NumPy: no pxr, no Isaac Sim, no ROS, no
network, since the chain was already extracted and cached to JSON once
(see :mod:`.usd_walk`). That independence is what lets this be evaluated
at any configuration, including ones the robot has never been commanded
to, instantly and without moving anything -- which is what makes the
sweep/batch diagnostics (:mod:`dh_lab.diagnostics`) possible.

URDF/USD link frames do not in general coincide with DH frames, so only
this module's base-to-tool transform is convention-independent and
directly comparable with the student's predicted tool pose; comparing
intermediate frames is not meaningful in general (see the instructor-only
per-row diagnostic instead, which derives its own reference DH table).
"""
from __future__ import annotations

import numpy as np

from ..kinematics.rotations import quaternion_to_rotation_matrix
from ..kinematics.transforms import FKResult, Transform
from .chain import ChainDescription, JointSpec

GROUND_TRUTH_CONVENTION = "USD articulation chain (extracted, convention-independent)"


def _axis_rotation_matrix(axis, angle: float) -> np.ndarray:
    """3x3 rotation by ``angle`` radians about a unit vector ``axis``
    (Rodrigues' formula) -- a joint's axis need not be a coordinate axis
    in general, unlike the Craig-DH primitives it's compared against."""
    ax = np.asarray(axis, dtype=float)
    ax = ax / np.linalg.norm(ax)
    c, s = np.cos(angle), np.sin(angle)
    k = np.array([
        [0, -ax[2], ax[1]],
        [ax[2], 0, -ax[0]],
        [-ax[1], ax[0], 0],
    ])
    return np.eye(3) + s * k + (1 - c) * (k @ k)


def _frame_matrix(offset) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = quaternion_to_rotation_matrix(offset.quaternion_wxyz)
    m[:3, 3] = offset.translation
    return m


def _joint_transform(joint: JointSpec, q_value: float) -> np.ndarray:
    """4x4 ``parent_link -> child_link`` transform at joint value
    ``q_value`` (ignored for a fixed joint).

    A USD Physics joint is authored as two local frames, one per body;
    they coincide in world space exactly when ``child_link`` is rotated
    about the joint's axis by the current joint angle relative to
    ``parent_link`` -- i.e. ``parent_frame @ Rot(axis, q) @
    inverse(child_frame)``, *not* simply "offset then rotate" (that
    would silently assume ``child_frame`` is identity, which the schema
    does not guarantee).
    """
    parent_frame = _frame_matrix(joint.parent_frame)
    child_frame = _frame_matrix(joint.child_frame)
    rot = np.eye(4)
    if joint.joint_type == "revolute":
        rot[:3, :3] = _axis_rotation_matrix(joint.axis, q_value)
    return parent_frame @ rot @ np.linalg.inv(child_frame)


def forward_kinematics(chain: ChainDescription, q_by_joint_name: dict) -> FKResult:
    """Evaluate the extracted chain at ``q_by_joint_name`` (revolute
    joint name -> radians; fixed joints and any joint missing from the
    dict are treated as 0). Returns every intermediate
    ``^{base_link}T_{link}`` plus the tool pose, keyed by *link* name, in
    the same :class:`~dh_lab.kinematics.transforms.FKResult` shape the
    student's DH-table FK returns (keyed by frame name/index there) --
    so the two tool poses are directly comparable, per this module's
    docstring.
    """
    running = Transform(np.eye(4), chain.base_link, chain.base_link, GROUND_TRUTH_CONVENTION)
    transforms = {chain.base_link: running}
    for joint in chain.joints:
        q_value = q_by_joint_name.get(joint.name, 0.0)
        step = Transform(
            _joint_transform(joint, q_value), joint.parent_link, joint.child_link, GROUND_TRUTH_CONVENTION
        )
        running = running.then(step)
        transforms[joint.child_link] = running
    tool = transforms[chain.tool_link]
    return FKResult(transforms=transforms, tool=tool)
