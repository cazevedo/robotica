"""Random-batch and zero-configuration diagnostics (spec section 6).
Pure Python + NumPy: no Isaac Sim, no ROS.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from ..config.robot_config import RobotConfig
from ..ground_truth.analytic_fk import forward_kinematics as ground_truth_forward_kinematics
from ..ground_truth.chain import ChainDescription
from ..kinematics.dh_table import DHTable
from ..kinematics.errors import orientation_error, position_error
from ..kinematics.transforms import forward_kinematics as dh_forward_kinematics


@dataclass(frozen=True)
class BatchSample:
    q_rad: dict  # q_index -> radians, for this one sample
    position_error_mm: float
    orientation_error_deg: float


@dataclass(frozen=True)
class BatchStatistics:
    samples: list  # list[BatchSample], in sampled order
    mean_position_mm: float
    median_position_mm: float
    p95_position_mm: float
    worst_position_mm: float
    mean_orientation_deg: float
    median_orientation_deg: float
    p95_orientation_deg: float
    worst_orientation_deg: float
    worst_sample: BatchSample  # by position error -- "jump to the worst one"


def _evaluate_one(table: DHTable, chain: ChainDescription, config: RobotConfig, q_rad: dict) -> BatchSample:
    rows, base, tool_t = table.evaluate(q_rad)
    predicted = dh_forward_kinematics(rows, base, tool_t).tool.matrix
    q_by_name = {config.joint_name_for(idx): value for idx, value in q_rad.items()}
    truth = ground_truth_forward_kinematics(chain, q_by_name).tool.matrix
    return BatchSample(
        q_rad=dict(q_rad),
        position_error_mm=position_error(predicted, truth).total_mm,
        orientation_error_deg=orientation_error(predicted, truth).angle_deg,
    )


def zero_configuration_check(table: DHTable, chain: ChainDescription, config: RobotConfig) -> BatchSample:
    """The "zero test": q = 0 for every joint. Many errors are invisible
    here -- finding that out is itself one of the lessons (see the
    student worksheet)."""
    return _evaluate_one(table, chain, config, {i: 0.0 for i in range(1, config.joint_count + 1)})


def random_batch(
    table: DHTable,
    chain: ChainDescription,
    config: RobotConfig,
    joint_limits: dict,
    num_samples: int = 2000,
    rng: Optional[np.random.Generator] = None,
) -> BatchStatistics:
    """Sample ``num_samples`` uniformly random valid configurations
    (within ``joint_limits``, ``{joint_name: (lower, upper)}`` radians)
    and report error statistics -- no simulator involved, since the
    ground truth is analytic and evaluable at any configuration.
    """
    rng = rng if rng is not None else np.random.default_rng()
    limits_by_index = {idx: joint_limits[config.joint_name_for(idx)] for idx in range(1, config.joint_count + 1)}

    samples = []
    for _ in range(num_samples):
        q_rad = {idx: float(rng.uniform(lo, hi)) for idx, (lo, hi) in limits_by_index.items()}
        samples.append(_evaluate_one(table, chain, config, q_rad))

    pos = np.array([s.position_error_mm for s in samples])
    ori = np.array([s.orientation_error_deg for s in samples])
    worst_index = int(np.argmax(pos))
    return BatchStatistics(
        samples=samples,
        mean_position_mm=float(np.mean(pos)),
        median_position_mm=float(np.median(pos)),
        p95_position_mm=float(np.percentile(pos, 95)),
        worst_position_mm=float(np.max(pos)),
        mean_orientation_deg=float(np.mean(ori)),
        median_orientation_deg=float(np.median(ori)),
        p95_orientation_deg=float(np.percentile(ori, 95)),
        worst_orientation_deg=float(np.max(ori)),
        worst_sample=samples[worst_index],
    )
