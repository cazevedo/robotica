"""Headless grading entry point (spec section 7): takes a saved student
DH table (JSON) plus the instructor's robot config and extracted
ground-truth chain, and reports the same batch statistics the GUI's
sweep/batch panel would show -- no GUI, no Isaac Sim, no ROS needed, so
a whole class's submissions can be graded from plain files.

Console-script entry point registered in setup.py as ``dh_lab_grade``.
Exit code is 1 if the table failed to parse, or if the worst-case error
over the random batch falls outside amber tolerance (i.e. would show
red in the GUI's colour coding); 0 otherwise -- documented here since
it's this module's own convention, not implied by the GUI.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

from ..config.robot_config import RobotConfig
from ..diagnostics.batch import random_batch, zero_configuration_check
from ..diagnostics.sweep import sweep_all_joints
from ..ground_truth.chain import ChainDescription
from ..kinematics.dh_table import DHTable


def grade(
    table_path,
    config_path,
    chain_path,
    num_samples: int = 2000,
    seed: int = 0,
    sweep_samples: int = 100,
) -> dict:
    """Load everything from disk and compute the full report as a plain,
    JSON-serializable dict. The CLI entry point below just formats and
    prints this; call it directly to grade programmatically (e.g. from a
    batch script iterating over a whole class's submissions).
    """
    table = DHTable.load(table_path)
    config = RobotConfig.load(config_path)
    chain = ChainDescription.load(chain_path)

    if not table.is_valid:
        invalid_cells = [
            {"row": i, "column": name, "error": getattr(row, name).error}
            for i, row in enumerate(table.rows)
            for name in ("alpha", "a", "d", "theta")
            if not getattr(row, name).is_valid
        ]
        return {"valid": False, "invalid_cells": invalid_cells}

    joint_limits = {j.name: (j.lower_limit, j.upper_limit) for j in chain.joints if j.joint_type == "revolute"}

    zero = zero_configuration_check(table, chain, config)
    stats = random_batch(table, chain, config, joint_limits, num_samples=num_samples, rng=np.random.default_rng(seed))
    sweeps = sweep_all_joints(
        table, chain, config, joint_limits,
        hold_at={i: 0.0 for i in range(1, config.joint_count + 1)},
        num_samples=sweep_samples,
    )
    worst_color = config.color_for_error(stats.worst_position_mm, stats.worst_orientation_deg)

    return {
        "valid": True,
        "zero_test": {
            "position_error_mm": zero.position_error_mm,
            "orientation_error_deg": zero.orientation_error_deg,
        },
        "batch": {
            "num_samples": num_samples,
            "mean_position_mm": stats.mean_position_mm,
            "median_position_mm": stats.median_position_mm,
            "p95_position_mm": stats.p95_position_mm,
            "worst_position_mm": stats.worst_position_mm,
            "mean_orientation_deg": stats.mean_orientation_deg,
            "median_orientation_deg": stats.median_orientation_deg,
            "p95_orientation_deg": stats.p95_orientation_deg,
            "worst_orientation_deg": stats.worst_orientation_deg,
            "worst_q_rad": stats.worst_sample.q_rad,
        },
        "sweep_max_position_error_mm": {name: float(np.max(s.position_error_mm)) for name, s in sweeps.items()},
        "sweep_max_orientation_error_deg": {name: float(np.max(s.orientation_error_deg)) for name, s in sweeps.items()},
        "worst_case_color": worst_color,
    }


def format_report(report: dict) -> str:
    if not report["valid"]:
        lines = ["INVALID TABLE -- cannot grade:"]
        for cell in report["invalid_cells"]:
            lines.append(f"  row {cell['row']} [{cell['column']}]: {cell['error']}")
        return "\n".join(lines)

    z, b = report["zero_test"], report["batch"]
    lines = [
        f"Zero test:         position {z['position_error_mm']:.3f} mm, "
        f"orientation {z['orientation_error_deg']:.3f} deg",
        f"Batch ({b['num_samples']} samples):",
        f"  position (mm):    mean {b['mean_position_mm']:.3f}  median {b['median_position_mm']:.3f}  "
        f"p95 {b['p95_position_mm']:.3f}  worst {b['worst_position_mm']:.3f}",
        f"  orientation (deg): mean {b['mean_orientation_deg']:.3f}  median {b['median_orientation_deg']:.3f}  "
        f"p95 {b['p95_orientation_deg']:.3f}  worst {b['worst_orientation_deg']:.3f}",
        f"  worst q (rad):    {b['worst_q_rad']}",
        f"Overall: {report['worst_case_color'].upper()}",
    ]
    return "\n".join(lines)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Headless DH Lab grading (no GUI, no Isaac Sim, no ROS).")
    parser.add_argument("table", type=Path, help="student's saved DH table JSON")
    parser.add_argument("--config", type=Path, required=True, help="instructor RobotConfig JSON")
    parser.add_argument("--chain", type=Path, required=True, help="extracted ChainDescription JSON (lite6_chain.json)")
    parser.add_argument("--samples", type=int, default=2000)
    parser.add_argument("--sweep-samples", type=int, default=100)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args(argv)

    report = grade(args.table, args.config, args.chain, args.samples, args.seed, args.sweep_samples)
    print(format_report(report))
    return 1 if (not report["valid"] or report["worst_case_color"] == "red") else 0


if __name__ == "__main__":
    sys.exit(main())
