"""The DH Lab ROS 2 node (spec section 9): subscribes to ``joint_states``,
holds the student's :class:`~dh_lab.kinematics.dh_table.DHTable` and the
extracted ground-truth :class:`~dh_lab.ground_truth.chain.ChainDescription`,
and publishes predicted/ground-truth tool poses and error diagnostics at
~30 Hz. Commands the arm by publishing *position* setpoints on
``joint_command`` -- the only command field this asset's articulation
actually responds to (see :mod:`.trajectory`) -- interpolated into a
velocity-limited trajectory rather than sent as an instantaneous jump.

Needs ``rclpy`` and the stock ROS 2 message packages. Two ways to run it:

* Standalone (headless/grading/CLI): ``ros2 run dh_lab dh_lab_node``,
  using the apt-installed ROS 2 Jazzy (``/opt/ros/jazzy``) -- a normal,
  separate ROS 2 process, talking to Isaac Sim's bridge over DDS like
  any other node (see CLAUDE.md's own architecture notes).
* Embedded in the omni.ui GUI extension, which runs *inside* Isaac Sim's
  Kit process and so has Kit's own *bundled* rclpy already loaded (the
  same one ``isaacsim.ros2.bridge`` uses) -- the GUI instantiates this
  exact same :class:`DHLabNode` there instead of spinning up a second OS
  process, pumping it with ``rclpy.spin_once(node, timeout_sec=0)`` from
  Kit's own per-frame update callback rather than the blocking
  ``rclpy.spin()`` this module's own :func:`main` uses. Nothing in
  :class:`DHLabNode` calls ``spin()`` itself, so it doesn't care which
  rclpy or which pump loop is in charge.

Predict-then-reveal is a *display* concept only: this node always
computes and publishes both poses on every tick (a grading script or
RViz always needs ground truth) -- :meth:`DHLabNode.set_reveal_ground_truth`
just records whether the GUI is currently showing it, it never gates
what gets published.
"""
from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray

from ..config.robot_config import RobotConfig
from ..ground_truth.analytic_fk import forward_kinematics as ground_truth_forward_kinematics
from ..ground_truth.chain import ChainDescription
from ..kinematics.dh_table import DHTable
from ..kinematics.errors import orientation_error, position_error
from ..kinematics.rotations import rotation_matrix_to_quaternion
from ..kinematics.transforms import forward_kinematics as dh_forward_kinematics
from ..visualization.markers import build_dh_frame_markers, build_tool_comparison_markers
from .trajectory import ClampResult, clamp_configuration, interpolate_trajectory

CONTROL_RATE_HZ = 30.0  # spec: "Live mode at ~30 Hz"
STALE_AFTER_SEC = 0.5  # spec: "never show stale numbers as current"

_DIAGNOSTIC_LEVEL_BY_COLOR = {
    "green": DiagnosticStatus.OK,
    "amber": DiagnosticStatus.WARN,
    "red": DiagnosticStatus.ERROR,
}


def _matrix_to_pose(matrix, frame_id: str, stamp) -> PoseStamped:
    """geometry_msgs/PoseStamped from a 4x4 homogeneous transform."""
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(c) for c in matrix[:3, 3])
    w, x, y, z = rotation_matrix_to_quaternion(matrix[:3, :3])
    msg.pose.orientation.w = float(w)
    msg.pose.orientation.x = float(x)
    msg.pose.orientation.y = float(y)
    msg.pose.orientation.z = float(z)
    return msg


class DHLabNode(Node):
    def __init__(
        self,
        config: RobotConfig,
        chain: ChainDescription,
        table: Optional[DHTable] = None,
        node_name: str = "dh_lab_node",
    ):
        super().__init__(node_name)
        self.config = config
        self.chain = chain
        self.table = table

        self._base_link = config.base_link or chain.base_link
        self._tool_link = config.tool_link or chain.tool_link
        self._joint_limits = {
            j.name: (j.lower_limit, j.upper_limit) for j in chain.joints if j.joint_type == "revolute"
        }

        self._latest_q_rad: Optional[dict] = None  # q_index -> radians, from joint_states
        self._latest_stamp_sec: Optional[float] = None  # node-clock seconds, for staleness
        self._reveal_ground_truth = True  # GUI-only display flag -- see module docstring

        self._speed_scale = config.default_speed_scale
        self._active_trajectory: list = []  # remaining TrajectoryStep, one consumed per tick

        self.joint_state_sub = self.create_subscription(JointState, "joint_states", self._on_joint_state, 10)
        self.command_pub = self.create_publisher(JointState, "joint_command", 10)
        self.predicted_pose_pub = self.create_publisher(PoseStamped, "~/predicted_pose", 10)
        self.ground_truth_pose_pub = self.create_publisher(PoseStamped, "~/ground_truth_pose", 10)
        self.error_pub = self.create_publisher(DiagnosticArray, "~/error", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "~/markers", 10)

        self.create_timer(1.0 / CONTROL_RATE_HZ, self._on_timer)
        self.get_logger().info(
            f"DH Lab node up: base_link={self._base_link!r} tool_link={self._tool_link!r}, "
            f"{len(self._joint_limits)} revolute joints."
        )

    # --- inputs ----------------------------------------------------------

    def _on_joint_state(self, msg: JointState) -> None:
        try:
            self._latest_q_rad = self.config.reorder_joint_state(list(msg.name), list(msg.position))
        except KeyError as exc:
            self.get_logger().warning(
                f"joint_states message missing an expected joint, ignoring: {exc}", throttle_duration_sec=5.0
            )
            return
        self._latest_stamp_sec = self._now_sec()

    def set_table(self, table: Optional[DHTable]) -> None:
        self.table = table

    def set_reveal_ground_truth(self, reveal: bool) -> None:
        self._reveal_ground_truth = reveal

    def set_speed_scale(self, scale: float) -> None:
        if not (0.0 < scale <= 1.0):
            raise ValueError(f"speed scale must be in (0, 1], got {scale}")
        self._speed_scale = scale

    # --- commanding --------------------------------------------------------

    def command_configuration(self, q_by_name: dict) -> ClampResult:
        """Clamp ``q_by_name`` (radians) to the real joint limits, then
        replace whatever trajectory is currently running with a new
        velocity-limited one from the last known joint state to the
        clamped target (joints not mentioned in ``q_by_name`` hold at
        their current reading). Returns the
        :class:`~dh_lab.ros2_node.trajectory.ClampResult` so a caller
        (the GUI) can surface which joints were out of range -- *this*
        clamps defensively regardless of what called it; the GUI
        additionally rejects out-of-range entry up front with a visible
        message (spec section 9), which is a separate, earlier check
        this method does not perform or assume.
        """
        if self._latest_q_rad is None:
            raise RuntimeError("cannot command a motion before any joint_states has been received")

        clamp_result = clamp_configuration(q_by_name, self._joint_limits)
        current = {self.config.joint_name_for(i): v for i, v in self._latest_q_rad.items()}
        target = dict(current)
        target.update(clamp_result.clamped)

        self._active_trajectory = interpolate_trajectory(
            current, target,
            max_speed_rad_s=self.config.max_joint_speed_rad_s,
            speed_scale=self._speed_scale,
            control_rate_hz=CONTROL_RATE_HZ,
        )
        return clamp_result

    def abort(self) -> None:
        """Always-available abort (spec section 9): drop the remaining
        trajectory immediately. Nothing more needs to happen -- every
        joint's drive already holds its last commanded position (see
        CLAUDE.md's drive-gain notes), so simply not sending any further
        setpoints halts motion exactly where the arm currently is."""
        self._active_trajectory = []
        self.get_logger().info("Abort: trajectory cleared.")

    # --- main loop -----------------------------------------------------

    def _on_timer(self) -> None:
        self._pump_trajectory()
        self._publish_state()

    def _pump_trajectory(self) -> None:
        if not self._active_trajectory:
            return
        step = self._active_trajectory.pop(0)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(step.positions)
        msg.position = [float(step.positions[name]) for name in msg.name]
        self.command_pub.publish(msg)

    def _is_stale(self) -> bool:
        if self._latest_stamp_sec is None:
            return True
        return (self._now_sec() - self._latest_stamp_sec) > STALE_AFTER_SEC

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _compute_tick(self) -> dict:
        """Pure decision logic for one control-loop tick -- no ROS
        message construction here, so it's directly testable without a
        live pub/sub round trip (see :meth:`_publish_state`, the thin
        wrapper that turns this into messages and publishes them).

        Always returns ``level``/``message``/``values``; also returns
        ``predicted``/``truth`` (4x4 tool matrices), ``predicted_fk``
        (the full FKResult, for the per-frame markers), and ``color``
        when (and only when) the data is fresh and the table is valid --
        their absence is exactly the "never show stale numbers as
        current" signal :meth:`_publish_state` uses to skip the pose and
        marker topics.
        """
        if self._is_stale():
            return {"level": DiagnosticStatus.STALE, "message": "no joint state data", "values": {}}

        if self.table is None or not self.table.is_valid:
            return {"level": DiagnosticStatus.WARN, "message": "no valid DH table loaded", "values": {}}

        q_by_name = {self.config.joint_name_for(i): v for i, v in self._latest_q_rad.items()}
        rows, base, tool_t = self.table.evaluate(self._latest_q_rad)
        predicted_fk = dh_forward_kinematics(rows, base, tool_t, base_frame_name=self._base_link)
        predicted = predicted_fk.tool.matrix
        truth = ground_truth_forward_kinematics(self.chain, q_by_name).tool.matrix

        pos_err = position_error(predicted, truth)
        ori_err = orientation_error(predicted, truth)
        color = self.config.color_for_error(pos_err.total_mm, ori_err.angle_deg)

        return {
            "level": _DIAGNOSTIC_LEVEL_BY_COLOR[color],
            "message": f"position {pos_err.total_mm:.3f} mm, orientation {ori_err.angle_deg:.3f} deg ({color})",
            "values": {
                "position_error_mm": pos_err.total_mm,
                "position_error_x_mm": pos_err.x_mm,
                "position_error_y_mm": pos_err.y_mm,
                "position_error_z_mm": pos_err.z_mm,
                "orientation_error_deg": ori_err.angle_deg,
            },
            "predicted": predicted,
            "predicted_fk": predicted_fk,
            "truth": truth,
            "color": color,
        }

    def _publish_state(self) -> None:
        stamp = self.get_clock().now().to_msg()
        tick = self._compute_tick()

        diag = DiagnosticArray()
        diag.header.stamp = stamp
        status = DiagnosticStatus(name="dh_lab", hardware_id=self._tool_link)
        status.level = tick["level"]
        status.message = tick["message"]
        status.values = [KeyValue(key=k, value=f"{v:.6f}") for k, v in tick["values"].items()]
        diag.status = [status]
        self.error_pub.publish(diag)

        if "predicted" not in tick:
            return  # stale or no table -- never publish poses computed from bad data

        self.predicted_pose_pub.publish(_matrix_to_pose(tick["predicted"], self._base_link, stamp))
        self.ground_truth_pose_pub.publish(_matrix_to_pose(tick["truth"], self._base_link, stamp))

        markers = MarkerArray()
        markers.markers.extend(
            build_dh_frame_markers(tick["predicted_fk"], self._base_link, stamp=stamp).markers
        )
        markers.markers.extend(
            build_tool_comparison_markers(tick["predicted"], tick["truth"], self._base_link, stamp=stamp).markers
        )
        self.marker_pub.publish(markers)


_DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "default_config.json"
_DEFAULT_CHAIN_PATH = "/isaac-sim/content/lite6_chain.json"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="DH Lab ROS 2 node.")
    parser.add_argument("--config", default=str(_DEFAULT_CONFIG_PATH), help="RobotConfig JSON path")
    parser.add_argument("--chain", default=_DEFAULT_CHAIN_PATH, help="extracted ChainDescription JSON path")
    parser.add_argument("--table", default=None, help="optional initial DH table JSON path")
    args = parser.parse_args(argv)

    rclpy.init()
    config = RobotConfig.load(args.config)
    chain = ChainDescription.load(args.chain)
    table = DHTable.load(args.table) if args.table else None
    node = DHLabNode(config, chain, table)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
