"""Tests for DHLabNode. Needs rclpy and the stock ROS 2 message packages
-- run these from inside the container, not on the bare host:

    docker exec isaac-sim-ros bash -c '\
        source /opt/ros/jazzy/setup.bash && cd /isaac-sim/ros2_ws/src/dh_lab && \
        PYTHONPATH="$(pwd)" python3 -m pytest test/test_dh_lab_node.py -v'

Exercises the node's decision logic (:meth:`DHLabNode._compute_tick`)
and state transitions (joint-state handling, commanding, abort)
directly, rather than relying on a live pub/sub round trip within the
test itself -- see dh_lab_node.py's own docstring for why _compute_tick
is kept separate from message construction in the first place.
"""
import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from sensor_msgs.msg import JointState

from dh_lab.ros2_node.dh_lab_node import DHLabNode

from conftest import build_chain, build_table


@pytest.fixture(scope="module", autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(robot_config):
    n = DHLabNode(robot_config, build_chain(), build_table(), node_name="test_dh_lab_node")
    yield n
    n.destroy_node()


def _joint_state_msg(names, positions):
    msg = JointState()
    msg.name = list(names)
    msg.position = list(positions)
    return msg


class TestJointStateHandling:
    def test_starts_with_no_data(self, node):
        assert node._latest_q_rad is None
        assert node._is_stale() is True

    def test_receiving_a_joint_state_updates_latest_q(self, node):
        node._on_joint_state(_joint_state_msg(["joint1", "joint2", "joint3"], [0.1, 0.2, 0.3]))
        assert node._latest_q_rad == {1: pytest.approx(0.1), 2: pytest.approx(0.2), 3: pytest.approx(0.3)}
        assert node._is_stale() is False

    def test_message_missing_a_configured_joint_is_ignored_not_partially_applied(self, node):
        node._on_joint_state(_joint_state_msg(["joint1"], [0.1]))  # missing joint2, joint3
        assert node._latest_q_rad is None


class TestCommandConfiguration:
    def test_raises_before_any_joint_state(self, node):
        with pytest.raises(RuntimeError):
            node.command_configuration({"joint1": 0.5})

    def test_builds_a_trajectory_to_target_holding_other_joints(self, node):
        node._on_joint_state(_joint_state_msg(["joint1", "joint2", "joint3"], [0.0, 0.0, 0.0]))
        node.command_configuration({"joint1": 0.5})
        assert len(node._active_trajectory) > 0
        assert node._active_trajectory[-1].positions["joint1"] == pytest.approx(0.5)
        assert all(step.positions["joint2"] == pytest.approx(0.0) for step in node._active_trajectory)

    def test_clamps_out_of_range_target_and_reports_it(self, node):
        node._on_joint_state(_joint_state_msg(["joint1", "joint2", "joint3"], [0.0, 0.0, 0.0]))
        result = node.command_configuration({"joint1": 999.0})
        assert "joint1" in result.out_of_range
        _, upper = node._joint_limits["joint1"]
        assert node._active_trajectory[-1].positions["joint1"] == pytest.approx(upper)

    def test_zero_distance_target_is_a_single_immediate_step(self, node):
        node._on_joint_state(_joint_state_msg(["joint1", "joint2", "joint3"], [0.2, 0.0, 0.0]))
        node.command_configuration({"joint1": 0.2})
        assert len(node._active_trajectory) == 1

    def test_abort_clears_trajectory(self, node):
        node._on_joint_state(_joint_state_msg(["joint1", "joint2", "joint3"], [0.0, 0.0, 0.0]))
        node.command_configuration({"joint1": 0.5})
        assert node._active_trajectory
        node.abort()
        assert node._active_trajectory == []


class TestComputeTick:
    def test_no_joint_state_is_stale(self, node):
        tick = node._compute_tick()
        assert tick["level"] == DiagnosticStatus.STALE
        assert "predicted" not in tick

    def test_no_table_warns(self, node):
        node.set_table(None)
        node._on_joint_state(_joint_state_msg(["joint1", "joint2", "joint3"], [0.0, 0.0, 0.0]))
        tick = node._compute_tick()
        assert tick["level"] == DiagnosticStatus.WARN
        assert "predicted" not in tick

    def test_correct_table_at_zero_is_green_with_zero_error(self, node):
        node._on_joint_state(_joint_state_msg(["joint1", "joint2", "joint3"], [0.0, 0.0, 0.0]))
        tick = node._compute_tick()
        assert tick["color"] == "green"
        assert tick["values"]["position_error_mm"] == pytest.approx(0.0, abs=1e-6)
        assert tick["level"] == DiagnosticStatus.OK

    def test_faulty_table_at_a_bad_configuration_is_not_green(self, node):
        node.set_table(build_table(a_prev=[0.0, 0.2, 0.3]))  # +5cm fault on row3
        node._on_joint_state(_joint_state_msg(["joint1", "joint2", "joint3"], [0.1, 0.2, 1.0]))
        tick = node._compute_tick()
        assert tick["color"] != "green"
        assert tick["values"]["position_error_mm"] > 10.0

    def test_stamp_staleness_recovers_once_new_data_arrives(self, node):
        node._latest_stamp_sec = node._now_sec() - 10.0  # force stale
        assert node._is_stale() is True
        node._on_joint_state(_joint_state_msg(["joint1", "joint2", "joint3"], [0.0, 0.0, 0.0]))
        assert node._is_stale() is False


class TestPublishStateSmoke:
    """`_publish_state` wraps _compute_tick's output in real ROS messages
    and publishes -- these just confirm that wrapping doesn't itself
    raise, in each of the three tick states; delivery isn't asserted."""

    def test_stale_state_does_not_raise(self, node):
        node._publish_state()

    def test_valid_state_does_not_raise(self, node):
        node._on_joint_state(_joint_state_msg(["joint1", "joint2", "joint3"], [0.1, -0.2, 0.3]))
        node._publish_state()
