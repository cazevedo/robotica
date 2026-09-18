import numpy as np
import pytest

from dh_lab.diagnostics.batch import random_batch, zero_configuration_check

from conftest import build_chain, build_table


def test_zero_configuration_check_matches_reference(reference_setup):
    table, chain, config = reference_setup
    sample = zero_configuration_check(table, chain, config)
    assert sample.position_error_mm == pytest.approx(0.0, abs=1e-6)
    assert sample.orientation_error_deg == pytest.approx(0.0, abs=1e-6)
    assert sample.q_rad == {1: 0.0, 2: 0.0, 3: 0.0}


def test_zero_configuration_check_can_hide_a_fault(robot_config):
    # A sign-error table is exactly correct at q=0 by construction (see
    # test_sweep_diagnostics.TestSignErrorSignature) -- the zero test
    # must report ~zero here even though the table is wrong elsewhere.
    # That blind spot *is* the lesson (spec section 6's "zero test").
    faulty = build_table(theta2_sign=-1.0)
    chain = build_chain()
    sample = zero_configuration_check(faulty, chain, robot_config)
    assert sample.orientation_error_deg == pytest.approx(0.0, abs=1e-6)


def test_random_batch_statistics_on_reference_table(reference_setup, joint_limits):
    table, chain, config = reference_setup
    stats = random_batch(table, chain, config, joint_limits, num_samples=200, rng=np.random.default_rng(0))
    assert stats.mean_position_mm == pytest.approx(0.0, abs=1e-6)
    assert stats.worst_position_mm == pytest.approx(0.0, abs=1e-6)
    assert len(stats.samples) == 200


def test_random_batch_statistics_on_faulty_table(robot_config, joint_limits):
    faulty = build_table(a_prev=[0.0, 0.2, 0.26])
    chain = build_chain()
    stats = random_batch(faulty, chain, robot_config, joint_limits, num_samples=500, rng=np.random.default_rng(1))
    assert stats.mean_position_mm > 5.0
    assert stats.worst_position_mm >= stats.p95_position_mm >= stats.median_position_mm
    # "offer to jump to the worst one" -- worst_sample must be an actual
    # sampled configuration, and its own error must equal worst_position_mm.
    assert stats.worst_sample.position_error_mm == pytest.approx(stats.worst_position_mm)


def test_random_batch_is_reproducible_with_a_seeded_rng(reference_setup, joint_limits):
    table, chain, config = reference_setup
    stats_a = random_batch(table, chain, config, joint_limits, num_samples=50, rng=np.random.default_rng(42))
    stats_b = random_batch(table, chain, config, joint_limits, num_samples=50, rng=np.random.default_rng(42))
    assert [s.q_rad for s in stats_a.samples] == [s.q_rad for s in stats_b.samples]
