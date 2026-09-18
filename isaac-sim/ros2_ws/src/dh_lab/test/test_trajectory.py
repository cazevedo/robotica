import pytest

from dh_lab.ros2_node.trajectory import clamp_configuration, interpolate_trajectory


def test_zero_distance_returns_single_step_at_target():
    steps = interpolate_trajectory({"j1": 0.5}, {"j1": 0.5}, max_speed_rad_s=1.0, speed_scale=0.2)
    assert len(steps) == 1
    assert steps[0].positions == {"j1": pytest.approx(0.5)}
    assert steps[0].time_from_start == 0.0


def test_single_joint_duration_matches_allowed_speed():
    steps = interpolate_trajectory(
        {"j1": 0.0}, {"j1": 1.0}, max_speed_rad_s=2.0, speed_scale=0.5, control_rate_hz=100.0
    )
    # allowed speed = 1.0 rad/s, distance = 1.0 rad -> duration = 1.0 s
    assert steps[-1].time_from_start == pytest.approx(1.0)
    assert steps[-1].positions == {"j1": pytest.approx(1.0)}


def test_multi_joint_duration_set_by_slowest_joint():
    steps = interpolate_trajectory(
        {"j1": 0.0, "j2": 0.0},
        {"j1": 1.0, "j2": 4.0},
        max_speed_rad_s=2.0, speed_scale=1.0, control_rate_hz=100.0,
    )
    # j2 needs 4.0/2.0 = 2.0s, j1 needs 1.0/2.0 = 0.5s -> duration = 2.0s
    assert steps[-1].time_from_start == pytest.approx(2.0)
    assert steps[-1].positions == {"j1": pytest.approx(1.0), "j2": pytest.approx(4.0)}


def test_steps_are_monotonic_in_time_and_never_overshoot():
    steps = interpolate_trajectory(
        {"j1": 0.0}, {"j1": -1.0}, max_speed_rad_s=1.0, speed_scale=0.3, control_rate_hz=50.0
    )
    times = [s.time_from_start for s in steps]
    assert times == sorted(times)
    assert all(-1.0 <= s.positions["j1"] <= 0.0 for s in steps)


def test_last_step_is_exact_target_even_if_duration_not_a_multiple_of_dt():
    steps = interpolate_trajectory(
        {"j1": 0.0}, {"j1": 0.37}, max_speed_rad_s=1.0, speed_scale=1.0, control_rate_hz=30.0
    )
    assert steps[-1].positions["j1"] == 0.37


def test_mismatched_joint_keys_raises():
    with pytest.raises(ValueError):
        interpolate_trajectory({"j1": 0.0}, {"j2": 1.0}, max_speed_rad_s=1.0, speed_scale=0.2)


@pytest.mark.parametrize("speed_scale", [0.0, -0.1, 1.5])
def test_invalid_speed_scale_raises(speed_scale):
    with pytest.raises(ValueError):
        interpolate_trajectory({"j1": 0.0}, {"j1": 1.0}, max_speed_rad_s=1.0, speed_scale=speed_scale)


class TestClampConfiguration:
    def test_within_limits_is_unchanged(self):
        result = clamp_configuration({"j1": 0.5}, {"j1": (-1.0, 1.0)})
        assert result.clamped == {"j1": pytest.approx(0.5)}
        assert result.out_of_range == {}

    def test_above_upper_is_clamped_and_reported(self):
        result = clamp_configuration({"j1": 5.0}, {"j1": (-1.0, 1.0)})
        assert result.clamped == {"j1": pytest.approx(1.0)}
        assert result.out_of_range == {"j1": (5.0, -1.0, 1.0)}

    def test_below_lower_is_clamped_and_reported(self):
        result = clamp_configuration({"j1": -5.0}, {"j1": (-1.0, 1.0)})
        assert result.clamped == {"j1": pytest.approx(-1.0)}
        assert result.out_of_range == {"j1": (-5.0, -1.0, 1.0)}

    def test_exactly_at_boundary_is_not_reported_out_of_range(self):
        result = clamp_configuration({"j1": 1.0}, {"j1": (-1.0, 1.0)})
        assert result.out_of_range == {}

    def test_multiple_joints_only_offenders_reported(self):
        result = clamp_configuration(
            {"j1": 0.5, "j2": 99.0}, {"j1": (-1.0, 1.0), "j2": (-1.0, 1.0)},
        )
        assert result.clamped == {"j1": pytest.approx(0.5), "j2": pytest.approx(1.0)}
        assert list(result.out_of_range) == ["j2"]
