"""LW1 public tests - PROVIDED, DO NOT EDIT.

    python3 -m pytest ~/dev_ws/src/lite6_kinematics/test -v

These check structure, units and internal consistency - not whether your DH
table is right. That is what fk_check (Gazebo, then the arm) is for.
"""

import ast
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from lite6_kinematics import forward_kinematics as fk  # noqa: E402

SOURCE = os.path.join(os.path.dirname(HERE), 'lite6_kinematics', 'forward_kinematics.py')
ALLOWED_IMPORTS = {'numpy', 'math', 'typing', '__future__'}

# Joint ranges of the xarm_description model (rad), used only to draw samples.
LOW = np.array([-3.11, -2.61, -0.06, -3.11, -2.16, -3.11])
HIGH = np.array([3.11, 2.61, 3.11, 3.11, 2.16, 3.11])
RNG = np.random.default_rng(2627)
SAMPLES = RNG.uniform(LOW, HIGH, size=(50, 6))


def _is_se3(T, tol=1e-9):
    T = np.asarray(T, dtype=float)
    R = T[:3, :3]
    return (T.shape == (4, 4)
            and np.allclose(T[3], [0, 0, 0, 1], atol=tol)
            and np.allclose(R.T @ R, np.eye(3), atol=1e-9)
            and abs(np.linalg.det(R) - 1.0) < 1e-9)


def _rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# --- rules --------------------------------------------------------------------
def test_only_numpy_and_math_are_imported():
    tree = ast.parse(open(SOURCE).read())
    used = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            used |= {a.name.split('.')[0] for a in node.names}
        elif isinstance(node, ast.ImportFrom):
            used.add((node.module or '').split('.')[0])
        elif isinstance(node, ast.Call) and getattr(node.func, 'id', '') in ('__import__', 'exec', 'eval', 'open'):
            used.add(node.func.id)
    assert used <= ALLOWED_IMPORTS, f'not allowed in forward_kinematics.py: {sorted(used - ALLOWED_IMPORTS)}'


# --- dh_table -------------------------------------------------------------------
def test_dh_table_shape_and_units():
    t = np.asarray(fk.dh_table(), dtype=float)
    assert t.shape == (6, 4), 'dh_table() must be (6, 4): [a_{i-1}, alpha_{i-1}, d_i, theta_offset_i]'
    assert np.all(np.isfinite(t))
    assert np.all(np.abs(t[:, [0, 2]]) < 0.5), 'lengths look like millimetres: use metres'
    assert np.all(np.abs(t[:, [1, 3]]) <= 2 * np.pi + 1e-9), 'angles look like degrees: use radians'
    assert np.any(t[:, [0, 2]] != 0), 'the table is still empty'


# --- mdh_transform ----------------------------------------------------------------
def test_mdh_zero_parameters_is_identity():
    assert np.allclose(fk.mdh_transform(0.0, 0.0, 0.0, 0.0), np.eye(4))


def test_mdh_is_a_rigid_transform():
    for a, alpha, d, theta in RNG.uniform(-3, 3, size=(50, 4)):
        assert _is_se3(fk.mdh_transform(a, alpha, d, theta))


def test_mdh_a_translates_along_x_of_previous_frame():
    T = fk.mdh_transform(0.3, 0.0, 0.0, 0.0)
    assert np.allclose(T[:3, :3], np.eye(3)) and np.allclose(T[:3, 3], [0.3, 0.0, 0.0])


def test_mdh_theta_rotates_about_z():
    T = fk.mdh_transform(0.0, 0.0, 0.0, 0.7)
    assert np.allclose(T[:3, :3], _rz(0.7)) and np.allclose(T[:3, 3], 0.0)


# --- link_transforms / forward_kinematics -------------------------------------------
def test_link_transforms_are_six_rigid_transforms():
    A = fk.link_transforms(SAMPLES[0])
    assert len(A) == 6 and all(_is_se3(a) for a in A)


def test_forward_kinematics_is_a_rigid_transform():
    for q in SAMPLES:
        assert _is_se3(fk.forward_kinematics(q))


def test_all_frames_structure():
    q = SAMPLES[1]
    frames = fk.forward_kinematics(q, all_frames=True)
    assert len(frames) == 7, 'all_frames=True returns [^0_0T, ^0_1T, ..., ^0_6T]'
    assert np.allclose(frames[0], np.eye(4))
    assert np.allclose(frames[-1], fk.forward_kinematics(q))


def test_frames_are_the_product_of_link_transforms():
    for q in SAMPLES[:10]:
        frames = fk.forward_kinematics(q, all_frames=True)
        A = fk.link_transforms(q)
        for i in range(1, 7):
            assert np.allclose(frames[i], frames[i - 1] @ A[i - 1], atol=1e-12)


def test_accepts_list_and_array():
    q = SAMPLES[2]
    assert np.allclose(fk.forward_kinematics(list(q)), fk.forward_kinematics(np.array(q)))


def test_reach_is_plausible():
    # Lite 6 reach is 440 mm; the end-effector frame is never farther than
    # ~0.75 m from {0}.
    for q in SAMPLES:
        reach = np.linalg.norm(fk.forward_kinematics(q)[:3, 3])
        assert reach < 0.75, 'end-effector too far from the base: units?'


# --- pose_from_transform (provided; these pass from day one) ------------------------------
# A regression test of the code you were given, not of your work: if one of
# these fails, pose_from_transform has been edited by mistake - restore it.
@pytest.mark.parametrize('roll,pitch,yaw', [
    (0.3, -0.4, 1.2), (np.pi - 0.1, 0.2, -2.5), (-1.0, 1.3, 0.4), (2.0, -1.2, -0.3)])
def test_pose_round_trip(roll, pitch, yaw):
    T = np.eye(4)
    T[:3, :3] = _rz(yaw) @ _ry(pitch) @ _rx(roll)
    T[:3, 3] = [0.1, -0.2, 0.3]
    p = np.asarray(fk.pose_from_transform(T), dtype=float)
    assert p.shape == (6,)
    assert np.allclose(p[:3], [0.1, -0.2, 0.3])
    assert -np.pi / 2 - 1e-12 <= p[4] <= np.pi / 2 + 1e-12
    assert np.allclose(_rz(p[5]) @ _ry(p[4]) @ _rx(p[3]), T[:3, :3], atol=1e-9)


@pytest.mark.parametrize('pitch', [np.pi / 2, -np.pi / 2])
def test_pose_degenerate_pitch(pitch):
    # At pitch = +-pi/2 the convention is yaw = 0 (Craig 2.8), and the returned
    # angles must still rebuild the same rotation.
    T = np.eye(4)
    T[:3, :3] = _rz(0.8) @ _ry(pitch) @ _rx(0.3)
    p = np.asarray(fk.pose_from_transform(T), dtype=float)
    assert p[5] == 0.0
    assert np.allclose(_rz(p[5]) @ _ry(p[4]) @ _rx(p[3]), T[:3, :3], atol=1e-9)
