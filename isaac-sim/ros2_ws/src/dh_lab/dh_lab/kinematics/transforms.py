"""Forward kinematics core: builds every intermediate transform of a
Craig (modified) Denavit-Hartenberg chain from a numerically-evaluated
:class:`~dh_lab.kinematics.dh_table.EvaluatedRow` list. Pure Python +
NumPy, no Isaac Sim or ROS 2 imports -- importable and testable
standalone (see ``../../test/test_fk.py``).

Craig / modified DH, frame ``i-1`` to ``i``::

    ^{i-1}T_i = Rot_x(alpha_{i-1}) . Trans_x(a_{i-1}) . Rot_z(theta_i) . Trans_z(d_i)

        [ c(th)        -s(th)          0        a       ]
        [ s(th)c(al)    c(th)c(al)    -s(al)   -s(al)d   ]
        [ s(th)s(al)    c(th)s(al)     c(al)    c(al)d   ]
        [ 0             0              0        1        ]

``craig_dh_matrix`` builds this by composing the four named primitives
below (``rot_x``, ``trans_x``, ``rot_z``, ``trans_z``); the test suite
checks the result against the closed form above, built independently, so
the two aren't just checking the same arithmetic against itself.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .dh_table import EvaluatedFixedTransform, EvaluatedRow

CONVENTION = "Craig modified DH"


@dataclass(frozen=True)
class Transform:
    """A single rigid-body transform between two named frames.

    ``matrix`` is the 4x4 homogeneous transform ``^{frame_from}T_{frame_to}``
    (Craig's own notation): it converts a point's homogeneous coordinates
    expressed *in* ``frame_to`` into coordinates expressed in
    ``frame_from``, i.e. ``p_from = matrix @ p_to``.
    """
    matrix: np.ndarray
    frame_from: str
    frame_to: str
    convention: str = CONVENTION

    def __repr__(self) -> str:
        return f"<Transform [{self.convention}] {self.frame_from} -> {self.frame_to}>"

    def then(self, other: "Transform") -> "Transform":
        """Compose with a transform starting where this one ends:
        ``self.then(other)`` is ``^{self.frame_from}T_{other.frame_to}``."""
        if self.frame_to != other.frame_from:
            raise ValueError(
                f"cannot compose {self.frame_from}->{self.frame_to} with "
                f"{other.frame_from}->{other.frame_to}: frames don't meet"
            )
        return Transform(self.matrix @ other.matrix, self.frame_from, other.frame_to, self.convention)


def rot_x(angle: float) -> np.ndarray:
    """4x4 homogeneous rotation about the local X axis by ``angle`` radians."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, c, -s, 0.0],
        [0.0, s, c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def rot_z(angle: float) -> np.ndarray:
    """4x4 homogeneous rotation about the local Z axis by ``angle`` radians."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([
        [c, -s, 0.0, 0.0],
        [s, c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def trans_x(distance: float) -> np.ndarray:
    """4x4 homogeneous translation of ``distance`` metres along local X."""
    m = np.eye(4)
    m[0, 3] = distance
    return m


def trans_z(distance: float) -> np.ndarray:
    """4x4 homogeneous translation of ``distance`` metres along local Z."""
    m = np.eye(4)
    m[2, 3] = distance
    return m


def craig_dh_matrix(alpha_prev: float, a_prev: float, d: float, theta: float) -> np.ndarray:
    """The 4x4 ``^{i-1}T_i`` matrix for one Craig modified-DH row.

    ``alpha_prev``/``a_prev`` are this row's ``alpha_{i-1}``/``a_{i-1}``
    (twist and length of the *previous* link); ``d``/``theta`` are this
    row's ``d_i``/``theta_i``. All angles in radians, all lengths in
    metres (evaluate a :class:`~dh_lab.kinematics.dh_table.DHTable` first
    to get these from the student's symbolic/unit-aware cells).
    """
    return rot_x(alpha_prev) @ trans_x(a_prev) @ rot_z(theta) @ trans_z(d)


def fixed_transform_matrix(t: EvaluatedFixedTransform) -> np.ndarray:
    """4x4 matrix for a fixed base/tool transform: extrinsic XYZ Euler
    (roll about world/parent X, then pitch about Y, then yaw about Z)
    composed as ``Rz(yaw) @ Ry(pitch) @ Rx(roll)``, translation ``(x,y,z)``.
    This Euler convention is this tool's own choice for a 6-field GUI
    form, unrelated to the Craig DH convention used for the chain itself.
    """
    cr, sr = np.cos(t.roll), np.sin(t.roll)
    cp, sp = np.cos(t.pitch), np.sin(t.pitch)
    cy, sy = np.cos(t.yaw), np.sin(t.yaw)
    rot = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    m = np.eye(4)
    m[:3, :3] = rot
    m[:3, 3] = [t.x, t.y, t.z]
    return m


@dataclass(frozen=True)
class FKResult:
    """Every intermediate transform of one forward-kinematics evaluation.

    ``transforms["0"]`` is ``^{base}T_0`` (i.e. exactly the fixed base
    transform); ``transforms[str(i)]`` for ``i >= 1`` is the full chain
    ``^{base}T_i`` (base frame through DH frame i); ``transforms["tool"]``
    (same object as :attr:`tool`) is ``^{base}T_tool``. All are anchored
    at the same base frame so they can be compared or drawn directly,
    rather than only being available frame-to-frame.
    """
    transforms: dict[str, Transform]
    tool: Transform

    def matrix_to(self, frame: str) -> np.ndarray:
        return self.transforms[frame].matrix


def forward_kinematics(
    rows: list[EvaluatedRow],
    base_transform: EvaluatedFixedTransform,
    tool_transform: EvaluatedFixedTransform,
    base_frame_name: str = "base",
) -> FKResult:
    """Evaluate the full Craig modified-DH chain and return **every**
    intermediate transform (not just the final tool pose), each anchored
    at ``base_frame_name`` -- see :class:`FKResult`.

    ``rows`` are already-numeric (SI) DH parameters for one configuration
    ``q`` (see :meth:`dh_lab.kinematics.dh_table.DHTable.evaluate`).
    ``base_transform`` is ``^{base_frame_name}T_0``; ``tool_transform``
    is ``^{N}T_tool`` where N is the last DH row.
    """
    world_to_0 = Transform(fixed_transform_matrix(base_transform), base_frame_name, "0")
    transforms: dict[str, Transform] = {"0": world_to_0}

    running = world_to_0
    for i, row in enumerate(rows, start=1):
        step = Transform(craig_dh_matrix(row.alpha, row.a, row.d, row.theta), str(i - 1), str(i))
        running = running.then(step)
        transforms[str(i)] = running

    tool = running.then(Transform(fixed_transform_matrix(tool_transform), str(len(rows)), "tool"))
    transforms["tool"] = tool
    return FKResult(transforms=transforms, tool=tool)
