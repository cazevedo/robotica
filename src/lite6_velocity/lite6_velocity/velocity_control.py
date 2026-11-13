"""Labwork 3 - velocity control of the UFACTORY Lite 6 along a circle, as a ROS 2 node.

THIS IS THE ONLY FILE YOU EDIT IN LABWORK 3.

Implement the following, replacing every ``raise NotImplementedError``:

    Part A - the task
        circle_reference(t, c, r, alpha_dot)      desired pose and velocity at time t

    Part B - the control
        jacobian(q)                               the 6x6 Jacobian of {6} in {0}
        open_loop_step(q, V_d)                    joint velocities, open loop
        closed_loop_step(q, T_d, V_d, kp)         joint velocities, closed loop
        VelocityNode                              the node that moves the arm

    The constants KP, DT, QDOT_MAX and SIGMA_MIN below are chosen by the group
    (Part B and analysis E.2).

Run the node with ``ros2 run lite6_velocity vel_node`` (in Gazebo add
``--ros-args -p use_sim_time:=true``).

``pose_error``, ``rotation_vector``, ``load_circle``, ``JOINT_NAMES``,
``TRAJECTORY_TOPIC`` and ``main`` below are PROVIDED: already implemented, do
not edit them.

Keep the function and class names, arguments and return types exactly as they
are: the instructor's grader uses them by name.

Conventions - those of Labworks 1 and 2:

    * Frame {0} coincides with the ROS frame ``link_base``; frame {6} with
      ``link_eef``.
    * Joint angles in RADIANS, in the controller's convention, as published on
      /joint_states; joint velocities in rad/s.
    * Lengths in METRES, linear velocities in m/s, angular velocities in rad/s,
      time in seconds.

Allowed imports: ``numpy`` and ``math`` for the maths; ``forward_kinematics``
from your own Labwork 1 and ``inverse_kinematics`` and ``select_solution``
from your own Labwork 2 (already imported below); and, for the node,
``rclpy``, ``sensor_msgs``, ``geometry_msgs``, ``std_msgs``,
``trajectory_msgs`` and ``builtin_interfaces`` (already imported below).
Nothing else - no MoveIt, no kinematics library, no URDF parser, no tf.
"""

import math
import os

import numpy as np

from lite6_kinematics.forward_kinematics import forward_kinematics
from lite6_ik.inverse_kinematics import inverse_kinematics, select_solution

try:
    import rclpy
    from builtin_interfaces.msg import Duration
    from geometry_msgs.msg import PoseStamped
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
except ImportError:         # the maths also runs without ROS
    rclpy = None
    Node = object

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
BASE_FRAME = 'link_base'
TRAJECTORY_TOPIC = '/lite6_traj_controller/joint_trajectory'

# --- chosen by the group (Part B, analysis E.2): replace each None ------------
KP = None               # 1/s   gain of the closed loop
DT = None               # s     control period
QDOT_MAX = None         # rad/s largest joint velocity the node may command
SIGMA_MIN = None        # smallest singular value of J below which the node stops


# =============================================================================
# PROVIDED - do not edit
# =============================================================================
def rotation_vector(R) -> np.ndarray:
    """Rotation vector phi(R) = theta * u of a 3x3 rotation matrix.

    u is the unit axis of the rotation and theta in [0, pi] its angle, so the
    result has norm theta; the zero vector for the identity. Expressed in the
    frame in which R is expressed.
    """
    R = np.asarray(R, dtype=float)
    v = 0.5 * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    s = np.linalg.norm(v)                   # sin(theta)
    c = 0.5 * (np.trace(R) - 1.0)           # cos(theta)
    theta = math.atan2(s, c)
    if s > 1e-6 or (s > 0.0 and c > 0.0):
        return (theta / s) * v
    if c > 0.0:                             # R is the identity
        return v
    M = 0.5 * (R + np.eye(3))               # theta = pi: R = 2 u u^T - I
    i = int(np.argmax(np.diag(M)))
    u = M[:, i] / math.sqrt(M[i, i])
    return math.pi * u


def pose_error(T_c, T_d) -> np.ndarray:
    """Error between a current pose T_c and a desired pose T_d, both (4, 4) in {0}.

    Returns the (6,) array [p_d - p_c ; phi(R_d R_c^T)]: the vector from the
    current to the desired position (m), then the rotation vector (rad) of the
    rotation that takes the current orientation to the desired one, expressed
    in {0}.
    """
    T_c = np.asarray(T_c, dtype=float)
    T_d = np.asarray(T_d, dtype=float)
    e = np.zeros(6)
    e[:3] = T_d[:3, 3] - T_c[:3, 3]
    e[3:] = rotation_vector(T_d[:3, :3] @ T_c[:3, :3].T)
    return e


def load_circle(name):
    """(c, r) of circle ``name`` in config/lw3_circles.yaml: centre (3,) in m, radius in m."""
    import yaml
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('lite6_velocity'), 'config', 'lw3_circles.yaml')
    except Exception:       # outside ROS: the source tree
        path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            'config', 'lw3_circles.yaml')
    with open(path) as f:
        circles = yaml.safe_load(f)['circles']
    if name not in circles:
        raise KeyError(f'circle {name!r} is not in {path}: {sorted(circles)}')
    return np.asarray(circles[name]['center'], dtype=float), float(circles[name]['radius'])


# =============================================================================
# Part A - the task
# =============================================================================
def circle_reference(t, c, r, alpha_dot):
    """Desired pose and velocity of {6} at time t on the circle of Part A.

    Args:
        t:         time since the start of the circle (s).
        c:         (3,) centre of the circle in {0} (m).
        r:         radius (m).
        alpha_dot: angular rate of the point on the circle (rad/s).

    Returns:
        (T_d, V_d): T_d the (4, 4) desired pose of {6} in {0}; V_d the (6,)
        desired velocity [v; w] in {0}: v (m/s) the linear velocity of the
        origin of {6}, w (rad/s) its angular velocity.
    """
    # TODO (LW3, Part A)
    raise NotImplementedError('circle_reference() is not implemented yet')


# =============================================================================
# Part B - the control
# =============================================================================
def jacobian(q) -> np.ndarray:
    """The Jacobian J(q) of {6}, expressed in {0}.

    Args:
        q: six joint angles (rad), order joint1..joint6.

    Returns:
        (6, 6) array J such that, for the joint velocities dq (rad/s),
        [v; w] = J(q) @ dq, where v (rows 0-2) is the linear velocity (m/s) of
        the origin of {6} and w (rows 3-5) its angular velocity (rad/s), both
        expressed in {0}. How J is obtained is the group's choice.
    """
    # TODO (LW3, Part B)
    raise NotImplementedError('jacobian() is not implemented yet')


def open_loop_step(q, V_d) -> np.ndarray:
    """Joint velocities of the open-loop law of Part B.

    Args:
        q:   six joint angles (rad), the current configuration.
        V_d: (6,) desired velocity [v; w] in {0} (m/s, rad/s).

    Returns:
        (6,) joint velocities (rad/s).
    """
    # TODO (LW3, Part B)
    raise NotImplementedError('open_loop_step() is not implemented yet')


def closed_loop_step(q, T_d, V_d, kp) -> np.ndarray:
    """Joint velocities of the closed-loop law of Part B.

    Args:
        q:   six joint angles (rad), the current configuration.
        T_d: (4, 4) desired pose of {6} in {0}.
        V_d: (6,) desired velocity [v; w] in {0} (m/s, rad/s).
        kp:  gain (1/s).

    Returns:
        (6,) joint velocities (rad/s). The error is pose_error(), which is
        provided.
    """
    # TODO (LW3, Part B)
    raise NotImplementedError('closed_loop_step() is not implemented yet')


# =============================================================================
# Part B - the node
# =============================================================================
class VelocityNode(Node):
    """Moves the end-effector along a circle by velocity control.

    ROS parameters (name, default):
        circle      'C1'        a circle of config/lw3_circles.yaml (load_circle())
        center      [0, 0, 0]   centre (m) of a circle of the group's own; used,
        radius      0.0         with radius (m), when radius > 0
        alpha_dot   pi/2        rad/s
        law         'closed'    'open' or 'closed'
        kp          KP          1/s
        dt          DT          s, the control period
        laps        3
        qdot_max    QDOT_MAX    rad/s
        sigma_min   SIGMA_MIN
        start_time  5.0         s, duration of the motion to the start of the circle

    Subscribes   /joint_states         sensor_msgs/JointState (joints picked by name)
    Publishes    TRAJECTORY_TOPIC      trajectory_msgs/JointTrajectory
                 /lite6_vel/reference  geometry_msgs/PoseStamped, frame BASE_FRAME
                 /lite6_vel/status     std_msgs/String, durability TRANSIENT_LOCAL

    Behaviour, with every time taken from the node's clock:
        1. When the first joint angles are received, it selects, with the
           group's Labwork 2 functions, the solution for the start of the
           circle (t = 0) nearest to the current angles, sends it as a
           trajectory of one point reached in start_time, and publishes the
           status 'moving_to_start'. With no executable solution it publishes
           'unreachable' and does nothing more.
        2. Once start_time has elapsed and every joint is within 1e-3 rad of
           that solution, it publishes 'running'; t = 0 is that instant.
        3. Every dt seconds, with q the latest angles received on
           /joint_states: it publishes T_d(t) on /lite6_vel/reference;
           if the smallest singular value of jacobian(q) is below sigma_min it
           publishes 'singular' and stops; it computes the joint velocities
           dq with the law chosen; if any |dq_i| exceeds qdot_max it publishes
           'too_fast' and stops; otherwise it sends q + dt * dq as a trajectory
           of one point reached in dt.
        4. After laps full turns of the circle it publishes 'done' and stops.
    Stopping means sending nothing more: the controller holds the last target.
    """

    def __init__(self):
        super().__init__('vel_node')
        # TODO (LW3, Part B)
        raise NotImplementedError('VelocityNode is not implemented yet')


def main(args=None):
    """PROVIDED. Entry point of ``ros2 run lite6_velocity vel_node``."""
    if rclpy is None:
        raise RuntimeError('ROS 2 (rclpy) is not available')
    rclpy.init(args=args)
    node = VelocityNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
