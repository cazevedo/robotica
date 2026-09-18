# Called from autoload_stage.py once the stage is open (so /World/lite6
# already exists). Builds an OmniGraph that wires ROS 2 JointState commands
# (position/velocity/effort, any subset per message) to the lite6
# articulation, and publishes its joint state back out.
#
# Node types and I/O confirmed against this image's installed extensions:
#   isaacsim.ros2.nodes/.../OgnROS2SubscribeJointState.ogn
#   isaacsim.core.nodes/.../OgnIsaacArticulationController.ogn
#   isaacsim.ros2.nodes/.../OgnROS2PublishJointState.ogn
import omni.graph.core as og
import omni.usd
from pxr import Sdf

GRAPH_PATH = "/World/ROS2ControlGraph"
ROBOT_PATH = "/World/lite6"
# PhysX's tensor API (used by IsaacArticulationController) matches the exact
# prim carrying ArticulationRootAPI, not just any ancestor of it -- and on
# this asset that's the world-to-base fixed joint, not the /World/lite6
# payload root itself (confirmed by reading the downloaded lite6.usd/
# configuration/lite6_base.usd: ArticulationRootAPI is applied to
# root_joint). Passing ROBOT_PATH itself here reproducibly fails with
# "Pattern '/World/lite6' did not match any articulations".
ARTICULATION_ROOT_PATH = f"{ROBOT_PATH}/root_joint"
COMMAND_TOPIC = "joint_command"
STATE_TOPIC = "joint_states"


def setup_ros2_control_graph():
    stage = omni.usd.get_context().get_stage()

    if not stage.GetPrimAtPath(ROBOT_PATH).IsValid():
        print(f"[ros2_control_graph] No prim at {ROBOT_PATH}; skipping ROS 2 control graph setup.")
        return

    if stage.GetPrimAtPath(GRAPH_PATH).IsValid():
        print(f"[ros2_control_graph] {GRAPH_PATH} already exists; leaving it as-is.")
        return

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("SubscribeJointCommand", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "SubscribeJointCommand.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("SubscribeJointCommand.outputs:execOut", "ArticulationController.inputs:execIn"),
                ("SubscribeJointCommand.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                ("SubscribeJointCommand.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                ("SubscribeJointCommand.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                ("SubscribeJointCommand.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
            ],
            keys.SET_VALUES: [
                ("SubscribeJointCommand.inputs:topicName", COMMAND_TOPIC),
                ("ArticulationController.inputs:robotPath", ARTICULATION_ROOT_PATH),
                ("PublishJointState.inputs:topicName", STATE_TOPIC),
                ("PublishJointState.inputs:targetPrim", [Sdf.Path(ARTICULATION_ROOT_PATH)]),
            ],
        },
    )
    print(
        f"[ros2_control_graph] Built {GRAPH_PATH}: '{COMMAND_TOPIC}' (sensor_msgs/JointState) -> "
        f"{ROBOT_PATH}, state published on '{STATE_TOPIC}'."
    )
