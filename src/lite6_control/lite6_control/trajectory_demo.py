"""The portable node: identical code in Gazebo and on the real arm.

This is the one to copy for lab work.

Why this and not the /ufactory services
---------------------------------------
There are two different ways to command the Lite 6, and they are not
interchangeable:

    ros2_control  ->  /lite6_traj_controller/follow_joint_trajectory
                      present in Gazebo AND on the real arm (lab sim / lab real)

    driver API    ->  /ufactory/set_servo_angle
                      present ONLY with the bare driver (lab driver)

Both stacks load the same `xarm_controller/config/lite6_controllers.yaml`, so
`lite6_traj_controller` means exactly the same thing whether the joints are
simulated or real. Write against it and your node moves from simulation to
hardware without a single edit - which is the whole point of the course rule
that nothing runs on the real robot until it has run in Gazebo.

`move_joints_demo.py` uses the driver API instead: lower level, direct, and
real-robot only.

Run it
------
    # terminal 1 - simulation
    lab sim
    # terminal 1 - or the real arm
    lab real 192.168.1.xxx

    # terminal 2 - the same command either way
    ros2 run lite6_control trajectory_demo --dry-run
    ros2 run lite6_control trajectory_demo

SAFETY: on hardware this moves a real arm. Emergency stop within reach.
Waypoints are small offsets from wherever the arm starts, so it cannot make a
large unplanned move - but check the space around it.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
DEFAULT_ACTION = '/lite6_traj_controller/follow_joint_trajectory'


class TrajectorySender(Node):

    def __init__(self, action_name: str = DEFAULT_ACTION):
        super().__init__('trajectory_demo')
        self._joint_state: Optional[JointState] = None
        self.create_subscription(JointState, '/joint_states',
                                 self._on_joint_state, 10)
        self._client = ActionClient(self, FollowJointTrajectory, action_name)
        self.action_name = action_name

    def _on_joint_state(self, msg: JointState) -> None:
        self._joint_state = msg

    # ------------------------------------------------------------------ #

    def wait_for_joint_states(self, timeout: float = 10.0) -> List[float]:
        """Current joint positions, ordered to match JOINT_NAMES.

        /joint_states does not promise any particular ordering, and it may
        carry joints you did not ask about (a gripper, for instance), so map
        by name rather than trusting the index.
        """
        deadline = time.monotonic() + timeout
        while self._joint_state is None:
            if time.monotonic() > deadline:
                raise RuntimeError(
                    'nothing published on /joint_states. Is the stack running? '
                    '  lab sim   (Gazebo)   or   lab real <ip>   (hardware)')
            rclpy.spin_once(self, timeout_sec=0.1)

        msg = self._joint_state
        index = {name: i for i, name in enumerate(msg.name)}
        missing = [n for n in JOINT_NAMES if n not in index]
        if missing:
            raise RuntimeError(f'/joint_states is missing {missing}; it has {list(msg.name)}')
        return [float(msg.position[index[n]]) for n in JOINT_NAMES]

    def wait_for_server(self, timeout: float = 10.0) -> None:
        if not self._client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError(
                f'no action server on {self.action_name}.\n'
                f'  - with `lab driver` there is no controller_manager at all; '
                f'use `lab real <ip>` or `lab sim` instead\n'
                f'  - otherwise check:  ros2 action list')

    def send_trajectory(self, points: List[List[float]], seconds_each: float = 3.0):
        """Send one trajectory through the controller and block until it ends.

        The controller interpolates between the points; `time_from_start` is
        measured from the beginning of the whole trajectory, not from the
        previous point, which is the usual thing to get wrong.
        """
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINT_NAMES)

        for i, positions in enumerate(points, start=1):
            point = JointTrajectoryPoint()
            point.positions = [float(p) for p in positions]
            point.velocities = [0.0] * len(JOINT_NAMES)   # come to rest at each one
            elapsed = seconds_each * i
            point.time_from_start = Duration(sec=int(elapsed),
                                             nanosec=int((elapsed % 1.0) * 1e9))
            goal.trajectory.points.append(point)

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if not handle.accepted:
            raise RuntimeError('the controller rejected the trajectory')

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        # error_code 0 == SUCCESSFUL. Anything else and the arm stopped early.
        if result.error_code != 0:
            raise RuntimeError(
                f'trajectory failed: error_code={result.error_code} '
                f'{result.error_string!r}')
        return result


def show(label: str, angles: List[float]) -> str:
    rad = ' '.join(f'{a:8.4f}' for a in angles)
    deg = ' '.join(f'{math.degrees(a):7.2f}' for a in angles)
    return f'  {label:<18} rad [{rad} ]\n  {"":<18} deg [{deg} ]'


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog='trajectory_demo',
        description='Send a joint trajectory - works in Gazebo and on hardware.')
    parser.add_argument('--amplitude', type=float, default=0.15,
                        help='waypoint offset in radians (default 0.15 = 8.6 deg)')
    parser.add_argument('--seconds', type=float, default=3.0,
                        help='seconds per waypoint')
    parser.add_argument('--action', default=DEFAULT_ACTION)
    parser.add_argument('--dry-run', action='store_true',
                        help='print the trajectory, send nothing')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(
        sys.argv if argv is None else argv)[1:])

    rclpy.init()
    node = TrajectorySender(action_name=args.action)
    try:
        start = node.wait_for_joint_states()
        print(show('start', start))

        a = args.amplitude
        waypoints = [
            [start[0] + a, start[1], start[2], start[3], start[4], start[5]],
            [start[0] + a, start[1], start[2], start[3], start[4], start[5] + a],
            list(start),                       # and back to where we began
        ]
        for i, wp in enumerate(waypoints, 1):
            print(show(f'waypoint {i}', wp))

        if args.dry_run:
            print(f'\n--dry-run: nothing sent. Target action: {args.action}')
            return 0

        node.wait_for_server()
        print(f'\nsending {len(waypoints)} waypoints to {args.action} ...')
        node.send_trajectory(waypoints, seconds_each=args.seconds)
        print('controller reports the trajectory finished')

        reached = node.wait_for_joint_states()
        error = max(abs(r - s) for r, s in zip(reached, start))
        print(show('ended at', reached))
        print(f'  worst deviation from the start pose: {error:.5f} rad '
              f'({math.degrees(error):.4f} deg)')
        print('\nIn Gazebo this residual is physics: the arm sags, overshoots and '
              'settles.\nRViz alone would show none of it.')
        return 0

    except RuntimeError as exc:
        print(f'\n{exc}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
