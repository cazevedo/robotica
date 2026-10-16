"""Labwork 1 - forward kinematics of the UFACTORY Lite 6.

THIS IS THE ONLY FILE YOU EDIT IN LABWORK 1.

Implement these three functions, replacing every ``raise NotImplementedError``:

    dh_table()                         the modified DH table, with joint offsets
    mdh_transform(a, alpha, d, theta)  one link transform, ^{i-1}_i T
    forward_kinematics(q, all_frames)  ^0_6 T, or all seven frames ^0_i T

``pose_from_transform(T)`` at the bottom is PROVIDED: already implemented, do
not edit it.

Keep the function names, arguments and return types exactly as they are: the
checker node (``fk_check``), the tests in ``test/`` and the instructor's grader
all call these functions by name.

Conventions - fixed, not yours to choose:

    * Craig's MODIFIED Denavit-Hartenberg convention, frames {0} ... {6}.
    * Frame {0} coincides with the ROS frame ``link_base``.
    * Frame {6} coincides with the ROS frame ``link_eef`` (the end-effector
      frame).
    * Joint angles in RADIANS, exactly as published on /joint_states
      (q = 0 is the controller's zero configuration).
    * Lengths in METRES.

Allowed imports: ``numpy`` and ``math``. Nothing else - no MoveIt, no
kinematics library, no URDF parser, no tf. ``numpy`` does the linear algebra
(``@`` for matrix products, ``np.cos``, ``np.eye``...); the robotics is yours.
``test/test_forward_kinematics.py`` checks the imports.
"""

import numpy as np


def dh_table() -> np.ndarray:
    """Modified DH parameters of the Lite 6.

    Returns:
        A (6, 4) float array. Row i-1 (i = 1..6) holds, in this order:

            [ a_{i-1} (m),  alpha_{i-1} (rad),  d_i (m),  theta_offset_i (rad) ]

        theta_offset_i is the constant added to the controller's joint angle
        q_i to obtain the DH joint angle:  theta_i = q_i + theta_offset_i.
    """
    # TODO (LW1, Part B): the table you derived in Part A.
    raise NotImplementedError('dh_table() is not implemented yet')


def mdh_transform(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    """Transform from frame {i-1} to frame {i}, ^{i-1}_i T, modified DH.

    Args:
        a:     a_{i-1}     (m)
        alpha: alpha_{i-1} (rad)
        d:     d_i         (m)
        theta: theta_i     (rad) - the DH angle, offset already included

    Returns:
        (4, 4) homogeneous transform.
    """
    # TODO (LW1, Part B)
    raise NotImplementedError('mdh_transform() is not implemented yet')


def forward_kinematics(q, all_frames: bool = False):
    """Pose of the end-effector, ^0_6 T, for joint vector q.

    Args:
        q: six joint angles (rad), controller convention, order joint1..joint6.
        all_frames: if True, return every frame instead of only the last one.

    Returns:
        all_frames=False: (4, 4) array ^0_6 T.
        all_frames=True:  list of seven (4, 4) arrays
                          [^0_0 T (identity), ^0_1 T, ^0_2 T, ..., ^0_6 T].

    Uses dh_table() and mdh_transform(), joint offsets applied.
    """
    # TODO (LW1, Part B): for each row of dh_table(), theta_i = q_i + theta_offset_i;
    # chain the transforms from mdh_transform(), starting from the identity.
    raise NotImplementedError('forward_kinematics() is not implemented yet')


# =============================================================================
# PROVIDED - DO NOT EDIT. Already implemented; nothing below is yours to change.
# =============================================================================
def pose_from_transform(T) -> np.ndarray:
    """End-effector pose as six numbers.

    Converts ^0_6 T into the representation used by the Lite 6 controller and
    by UFACTORY Studio. fk_check uses it to print your pose.

    Args:
        T: (4, 4) homogeneous transform.

    Returns:
        (6,) array [x, y, z, roll, pitch, yaw]: position in metres, orientation
        in radians as X-Y-Z FIXED angles (Craig, section 2.8):

            R = Rz(yaw) @ Ry(pitch) @ Rx(roll),   -pi/2 <= pitch <= pi/2

        (UFACTORY Studio shows the same three angles, in degrees.) At
        pitch = +-pi/2 exactly, roll and yaw cannot be told apart; this
        function takes the degenerate-case choice of Craig 2.8: yaw = 0.
    """
    T = np.asarray(T, dtype=float)
    r = T[:3, :3]
    pitch = np.arctan2(-r[2, 0], np.hypot(r[0, 0], r[1, 0]))
    cb = np.cos(pitch)
    if cb > 1e-12:
        yaw = np.arctan2(r[1, 0] / cb, r[0, 0] / cb)
        roll = np.arctan2(r[2, 1] / cb, r[2, 2] / cb)
    elif pitch > 0:            # pitch = +90 deg
        yaw, roll = 0.0, np.arctan2(r[0, 1], r[1, 1])
    else:                      # pitch = -90 deg
        yaw, roll = 0.0, -np.arctan2(r[0, 1], r[1, 1])
    return np.array([T[0, 3], T[1, 3], T[2, 3], roll, pitch, yaw])
