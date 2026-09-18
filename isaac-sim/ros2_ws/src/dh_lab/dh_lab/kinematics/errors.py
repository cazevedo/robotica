"""Error metrics between a predicted and a ground-truth tool pose. Pure
Python + NumPy, no Isaac Sim or ROS 2 imports.

Orientation error is the axis-angle magnitude of the *relative* rotation
between the two poses, extracted via ``angle = 2*atan2(||q_xyz||, |q_w|)``
on the equivalent quaternion -- not ``arccos`` of a rotation-matrix trace,
whose derivative vanishes near zero error (exactly the regime a correct
student answer sits in), which makes it numerically unstable there.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .rotations import rotation_matrix_to_quaternion


@dataclass(frozen=True)
class PositionError:
    """Position error between two tool poses, in millimetres, expressed
    in the base frame both poses were given in."""
    total_mm: float
    x_mm: float
    y_mm: float
    z_mm: float


@dataclass(frozen=True)
class OrientationError:
    """Orientation error between two tool poses: axis-angle magnitude of
    the relative rotation, in degrees."""
    angle_deg: float


def position_error(predicted: np.ndarray, ground_truth: np.ndarray) -> PositionError:
    """``predicted``/``ground_truth`` are 4x4 homogeneous transforms
    expressed in the same base frame."""
    delta_mm = (np.asarray(predicted)[:3, 3] - np.asarray(ground_truth)[:3, 3]) * 1000.0
    return PositionError(
        total_mm=float(np.linalg.norm(delta_mm)),
        x_mm=float(delta_mm[0]),
        y_mm=float(delta_mm[1]),
        z_mm=float(delta_mm[2]),
    )


def orientation_error(predicted: np.ndarray, ground_truth: np.ndarray) -> OrientationError:
    """``predicted``/``ground_truth`` are 4x4 homogeneous transforms (only
    the rotation block is used) expressed in the same base frame."""
    p_rot = np.asarray(predicted)[:3, :3]
    g_rot = np.asarray(ground_truth)[:3, :3]
    relative = g_rot.T @ p_rot
    q = rotation_matrix_to_quaternion(relative)
    w, xyz = q[0], q[1:]
    angle_rad = 2.0 * np.arctan2(np.linalg.norm(xyz), np.abs(w))
    return OrientationError(angle_deg=float(np.degrees(angle_rad)))
