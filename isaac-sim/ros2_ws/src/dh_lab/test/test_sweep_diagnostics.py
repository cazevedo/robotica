"""Fault-injection tests for the per-joint sweep diagnostic: inject one
known fault at a time into an otherwise-correct table and assert the
sweep produces the qualitative signature the UI claims it does (spec
section 6 / section 10 -- "this test is what keeps the hints honest").
"""
import math

import numpy as np
import pytest

from dh_lab.diagnostics.sweep import sweep_joint
from dh_lab.kinematics.dh_table import DHTable
from dh_lab.kinematics.units import AngleUnit, LengthUnit

from conftest import build_chain, build_table


def _sinusoidal_r_squared(q: np.ndarray, y: np.ndarray) -> float:
    """Fraction of variance in ``y`` explained by fitting a first- and
    second-harmonic sinusoid (``A + B sin(q) + C cos(q) + D sin(2q) + E
    cos(2q)``) -- close to 1.0 for a genuinely sinusoidal signature, much
    lower for a monotonic or constant one.

    The second harmonic matters here specifically because ``y`` is a
    *magnitude* (never negative): a wrong alpha upstream produces a
    geometric discrepancy that is itself a plain sinusoid in the swept
    joint, but its norm is the *rectified* version of that sinusoid,
    which is dominated by double-frequency content (``|sin(x)|``'s own
    Fourier series has no fundamental-frequency term at all) -- fitting
    only ``sin(q)``/``cos(q)`` would wrongly fail a textbook-correct
    alpha fault.
    """
    basis = np.stack([np.ones_like(q), np.sin(q), np.cos(q), np.sin(2 * q), np.cos(2 * q)], axis=1)
    coeffs, *_ = np.linalg.lstsq(basis, y, rcond=None)
    fitted = basis @ coeffs
    ss_res = np.sum((y - fitted) ** 2)
    ss_tot = np.sum((y - np.mean(y)) ** 2)
    return 1.0 - ss_res / ss_tot if ss_tot > 0 else 1.0


class TestReferenceFixtureIsSelfConsistent:
    """Guards the fixture itself: if this fails, the fault-injection
    tests below aren't testing what they claim to."""

    def test_zero_error_at_several_configurations(self, robot_config):
        table, chain = build_table(), build_chain()
        for q1, q2, q3 in [(0, 0, 0), (0.3, -0.6, 1.1), (-1.0, 2.0, -2.5)]:
            result = sweep_joint(
                table, chain, robot_config, q_index=1,
                hold_at={2: q2, 3: q3}, lower_limit=q1, upper_limit=q1, num_samples=1,
            )
            assert result.position_error_mm[0] == pytest.approx(0.0, abs=1e-6)
            assert result.orientation_error_deg[0] == pytest.approx(0.0, abs=1e-6)


class TestBaseOrToolTransformSignature:
    """"error flat and non-zero across all joints suggests a base or
    tool transform" -- a translation-only base error is exactly constant
    in every joint (it commutes with every downstream rotation)."""

    def test_translation_only_base_fault_is_flat_across_the_sweep(self, robot_config, joint_limits):
        table = build_table()
        table.set_units(AngleUnit.RADIANS, LengthUnit.METRES)
        table.set_fixed_cell("base", "x", "0.05")  # 5 cm base-position fault
        chain = build_chain()

        result = sweep_joint(
            table, chain, robot_config, q_index=1,
            hold_at={2: 0.4, 3: -0.7}, lower_limit=-math.pi, upper_limit=math.pi, num_samples=100,
        )
        mean = np.mean(result.position_error_mm)
        std = np.std(result.position_error_mm)
        assert mean == pytest.approx(50.0, abs=1e-6)  # exactly 50mm, constant
        assert std < 1e-6

    def test_flat_signature_holds_when_sweeping_a_different_joint_too(self, robot_config):
        table = build_table()
        table.set_fixed_cell("base", "x", "0.05")
        chain = build_chain()
        result = sweep_joint(
            table, chain, robot_config, q_index=3,
            hold_at={1: 0.2, 2: -0.3}, lower_limit=-math.pi, upper_limit=math.pi, num_samples=100,
        )
        assert np.std(result.position_error_mm) < 1e-6


class TestSignErrorSignature:
    """"error zero at q_j=0 and growing monotonically with q_j suggests
    a sign error on joint j" (read as: growing with |q_j|, i.e. away
    from zero in either direction -- a sign flip is symmetric)."""

    def test_zero_at_q_equals_zero(self, reference_setup):
        table, _, config = reference_setup
        faulty = build_table(theta2_sign=-1.0)
        chain = build_chain()
        result = sweep_joint(
            faulty, chain, config, q_index=2,
            hold_at={1: 0.0, 3: 0.0}, lower_limit=-0.02, upper_limit=0.02, num_samples=41,
        )
        mid = len(result.q_values_rad) // 2
        assert result.q_values_rad[mid] == pytest.approx(0.0, abs=1e-9)
        assert result.orientation_error_deg[mid] == pytest.approx(0.0, abs=1e-6)

    def test_grows_away_from_zero_on_both_sides(self, robot_config):
        faulty = build_table(theta2_sign=-1.0)
        chain = build_chain()
        result = sweep_joint(
            faulty, chain, robot_config, q_index=2,
            hold_at={1: 0.0, 3: 0.0}, lower_limit=-math.pi / 2, upper_limit=math.pi / 2, num_samples=101,
        )
        errors = result.orientation_error_deg
        mid = len(errors) // 2
        negative_half = errors[: mid + 1][::-1]  # from q=0 outward, decreasing index
        positive_half = errors[mid:]              # from q=0 outward, increasing index
        assert np.all(np.diff(negative_half) >= -1e-9)
        assert np.all(np.diff(positive_half) >= -1e-9)
        assert errors[0] > 1.0  # clearly nonzero far from q=0
        assert errors[-1] > 1.0


class TestWrongLengthSignature:
    """"error constant in q_j but non-zero suggests a wrong a or d in
    that row" -- perturb row3's `a` and sweep joint3 (that row's own
    joint)."""

    def test_constant_nonzero_error_when_sweeping_the_perturbed_rows_own_joint(self, robot_config):
        faulty = build_table(a_prev=[0.0, 0.2, 0.25 + 0.01])  # +1 cm on row3's a
        chain = build_chain()  # ground truth keeps the correct 0.25 m

        result = sweep_joint(
            faulty, chain, robot_config, q_index=3,
            hold_at={1: 0.3, 2: -0.5}, lower_limit=-math.pi, upper_limit=math.pi, num_samples=100,
        )
        mean = np.mean(result.position_error_mm)
        std = np.std(result.position_error_mm)
        assert mean == pytest.approx(10.0, abs=0.1)  # ~1cm
        assert std / mean < 0.05  # low relative variation = "constant"


class TestWrongAlphaSignature:
    """"a sinusoidal signature of one joint suggests a wrong alpha
    upstream of it" -- perturb row2's alpha (upstream of theta2 in that
    same row's own matrix product) and sweep joint2."""

    def test_sinusoidal_signature_when_sweeping_the_affected_joint(self, robot_config):
        faulty = build_table(alpha_prev=[0.0, math.pi / 2 + math.radians(20), 0.0])
        chain = build_chain()  # ground truth keeps alpha1 = 90 deg exactly

        result = sweep_joint(
            faulty, chain, robot_config, q_index=2,
            hold_at={1: 0.2, 3: -0.4}, lower_limit=-math.pi, upper_limit=math.pi, num_samples=200,
        )
        r2 = _sinusoidal_r_squared(result.q_values_rad, result.position_error_mm)
        assert r2 > 0.9
        # Distinguish from "constant": this signature has real spread...
        assert np.std(result.position_error_mm) > 1.0
        # ...and from "monotonic": at least one interior local extremum.
        d = np.diff(result.position_error_mm)
        sign_changes = np.sum(np.diff(np.sign(d)) != 0)
        assert sign_changes >= 1
