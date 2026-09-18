"""Per-joint sweep diagnostic. Holds every joint but one at a fixed
configuration, sweeps that one joint across its full range, and records
position/orientation error against the ground truth at each sample --
all without moving the simulated robot, since the ground truth
(:mod:`dh_lab.ground_truth.analytic_fk`) is analytic and can be evaluated
at any configuration instantly.

The GUI reads the resulting curve's *shape* out loud (flat-and-nonzero
across every joint suggests a base/tool transform; zero at q_j=0 growing
away from it suggests a sign error on joint j; constant-but-nonzero
suggests a wrong a/d in that row; sinusoidal suggests a wrong alpha
upstream) -- that mapping is exercised and kept honest by
``test/test_sweep_diagnostics.py``'s fault-injection tests, not
hard-coded here as a classifier: this module only produces the curves.

Pure Python + NumPy: no Isaac Sim, no ROS.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..config.robot_config import RobotConfig
from ..ground_truth.analytic_fk import forward_kinematics as ground_truth_forward_kinematics
from ..ground_truth.chain import ChainDescription
from ..kinematics.dh_table import DHTable
from ..kinematics.errors import orientation_error, position_error
from ..kinematics.transforms import forward_kinematics as dh_forward_kinematics


@dataclass(frozen=True)
class SweepResult:
    joint_name: str
    q_values_rad: np.ndarray        # the swept joint's own value at each sample
    position_error_mm: np.ndarray
    orientation_error_deg: np.ndarray


def sweep_joint(
    table: DHTable,
    chain: ChainDescription,
    config: RobotConfig,
    q_index: int,
    hold_at: dict,
    lower_limit: float,
    upper_limit: float,
    num_samples: int = 200,
) -> SweepResult:
    """Sweep ``q_index`` (1-based) from ``lower_limit`` to
    ``upper_limit`` radians, holding every other joint at ``hold_at``
    (``{q_index: radians}``; any entry for ``q_index`` itself is
    overwritten at each sample).
    """
    joint_name = config.joint_name_for(q_index)
    q_values = np.linspace(lower_limit, upper_limit, num_samples)
    pos_err = np.empty(num_samples)
    ori_err = np.empty(num_samples)

    for i, q_swept in enumerate(q_values):
        q_rad = dict(hold_at)
        q_rad[q_index] = float(q_swept)

        rows, base, tool_t = table.evaluate(q_rad)
        predicted = dh_forward_kinematics(rows, base, tool_t).tool.matrix

        q_by_name = {config.joint_name_for(idx): value for idx, value in q_rad.items()}
        truth = ground_truth_forward_kinematics(chain, q_by_name).tool.matrix

        pos_err[i] = position_error(predicted, truth).total_mm
        ori_err[i] = orientation_error(predicted, truth).angle_deg

    return SweepResult(
        joint_name=joint_name, q_values_rad=q_values,
        position_error_mm=pos_err, orientation_error_deg=ori_err,
    )


def sweep_all_joints(
    table: DHTable,
    chain: ChainDescription,
    config: RobotConfig,
    joint_limits: dict,
    hold_at: dict,
    num_samples: int = 200,
) -> dict:
    """:func:`sweep_joint` for every joint in ``config``, holding the
    others at ``hold_at`` in turn. ``joint_limits`` is ``{joint_name:
    (lower, upper)}`` radians (typically read straight from the same
    :class:`~dh_lab.ground_truth.chain.ChainDescription`). Returns
    ``{joint_name: SweepResult}``.
    """
    results = {}
    for q_index in range(1, config.joint_count + 1):
        joint_name = config.joint_name_for(q_index)
        lower, upper = joint_limits[joint_name]
        results[joint_name] = sweep_joint(table, chain, config, q_index, hold_at, lower, upper, num_samples)
    return results
