import pytest

from dh_lab.config.robot_config import RobotConfig, Tolerances


def _config(**overrides):
    defaults = dict(joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])
    defaults.update(overrides)
    return RobotConfig(**defaults)


def test_joint_name_and_q_index_are_inverse():
    config = _config()
    assert config.joint_name_for(1) == "joint1"
    assert config.joint_name_for(6) == "joint6"
    assert config.q_index_for("joint3") == 3


def test_reorder_joint_state_handles_out_of_order_message():
    config = _config(joint_names=["joint1", "joint2", "joint3"])
    # Message lists them in a different order than joint_names, plus an
    # extra sensor this config doesn't know about.
    names = ["joint3", "gripper", "joint1", "joint2"]
    positions = [0.3, 99.0, 0.1, 0.2]
    q = config.reorder_joint_state(names, positions)
    assert q == {1: pytest.approx(0.1), 2: pytest.approx(0.2), 3: pytest.approx(0.3)}


def test_reorder_joint_state_raises_on_missing_joint():
    config = _config(joint_names=["joint1", "joint2"])
    with pytest.raises(KeyError):
        config.reorder_joint_state(["joint1"], [0.0])


class TestColorForError:
    def test_green(self):
        config = _config()
        assert config.color_for_error(0.5, 0.1) == "green"

    def test_amber(self):
        config = _config()
        assert config.color_for_error(5.0, 2.0) == "amber"

    def test_red(self):
        config = _config()
        assert config.color_for_error(50.0, 20.0) == "red"

    def test_worse_of_the_two_dominates(self):
        config = _config()
        # Position is green-level but orientation is red-level -> red.
        assert config.color_for_error(0.1, 20.0) == "red"

    def test_boundary_is_exclusive(self):
        config = _config(tolerances=Tolerances(position_green_mm=1.0, orientation_green_deg=0.5))
        assert config.color_for_error(1.0, 0.1) != "green"  # exactly at the boundary


def test_json_roundtrip(tmp_path):
    config = _config(
        base_link="base_link", tool_link="link6",
        presets={"home": {"joint1": 0.0, "joint2": -0.3}},
        default_speed_scale=0.35,
    )
    path = tmp_path / "config.json"
    config.save(path)
    loaded = RobotConfig.load(path)
    assert loaded.joint_names == config.joint_names
    assert loaded.base_link == "base_link"
    assert loaded.tool_link == "link6"
    assert loaded.presets == {"home": {"joint1": 0.0, "joint2": -0.3}}
    assert loaded.default_speed_scale == pytest.approx(0.35)


def test_base_and_tool_link_default_to_none():
    config = _config()
    assert config.base_link is None
    assert config.tool_link is None
