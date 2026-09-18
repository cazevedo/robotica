import math

import pytest

from dh_lab.grading.headless_grade import format_report, grade

from conftest import build_chain, build_table


def _write_fixtures(tmp_path, table, chain, config):
    table_path = tmp_path / "table.json"
    chain_path = tmp_path / "chain.json"
    config_path = tmp_path / "config.json"
    table.save(table_path)
    chain.save(chain_path)
    config.save(config_path)
    return table_path, config_path, chain_path


def test_grade_reference_table_is_green(tmp_path, robot_config):
    table, chain = build_table(), build_chain()
    table_path, config_path, chain_path = _write_fixtures(tmp_path, table, chain, robot_config)

    report = grade(table_path, config_path, chain_path, num_samples=100, sweep_samples=20)

    assert report["valid"] is True
    assert report["batch"]["worst_position_mm"] == pytest.approx(0.0, abs=1e-6)
    assert report["worst_case_color"] == "green"
    assert "GREEN" in format_report(report)


def test_grade_faulty_table_is_not_green(tmp_path, robot_config):
    faulty = build_table(a_prev=[0.0, 0.2, 0.3])  # +5cm on row3's a -- a large fault
    chain = build_chain()
    table_path, config_path, chain_path = _write_fixtures(tmp_path, faulty, chain, robot_config)

    report = grade(table_path, config_path, chain_path, num_samples=100, sweep_samples=20)

    assert report["valid"] is True
    assert report["worst_case_color"] in ("amber", "red")
    assert report["batch"]["worst_position_mm"] > 10.0


def test_grade_invalid_table_is_reported_not_raised(tmp_path, robot_config):
    table = build_table()
    table.set_cell(0, "theta", "q1 + import os")  # a bad expression
    chain = build_chain()
    table_path, config_path, chain_path = _write_fixtures(tmp_path, table, chain, robot_config)

    report = grade(table_path, config_path, chain_path, num_samples=10, sweep_samples=5)

    assert report["valid"] is False
    assert report["invalid_cells"][0]["row"] == 0
    assert report["invalid_cells"][0]["column"] == "theta"
    assert "INVALID TABLE" in format_report(report)


def test_grade_is_reproducible_with_same_seed(tmp_path, robot_config):
    table, chain = build_table(), build_chain()
    table_path, config_path, chain_path = _write_fixtures(tmp_path, table, chain, robot_config)

    report_a = grade(table_path, config_path, chain_path, num_samples=50, seed=7)
    report_b = grade(table_path, config_path, chain_path, num_samples=50, seed=7)
    assert report_a["batch"]["worst_q_rad"] == report_b["batch"]["worst_q_rad"]


def test_sweep_max_errors_are_reported_per_joint(tmp_path, robot_config):
    table, chain = build_table(), build_chain()
    table_path, config_path, chain_path = _write_fixtures(tmp_path, table, chain, robot_config)

    report = grade(table_path, config_path, chain_path, num_samples=10, sweep_samples=20)
    assert set(report["sweep_max_position_error_mm"]) == {"joint1", "joint2", "joint3"}
    assert all(v == pytest.approx(0.0, abs=1e-6) for v in report["sweep_max_position_error_mm"].values())
