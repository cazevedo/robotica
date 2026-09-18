# Lite 6 in Gazebo with ros2_control, plus an RViz that shows the robot and its
# frames — and no MoveIt.
#
# xarm_gazebo's own launch can start RViz (show_rviz:=true), but it points it at
# xarm_moveit_config's moveit.rviz and hands it an empty robot_description_semantic,
# so the MotionPlanning display has no SRDF and no move_group to talk to: the
# robot never appears and RViz reports missing transforms. We leave that RViz off
# (show_rviz defaults to false) and start our own with a plain RobotModel + TF
# config instead.
#
# Run it by path — it is not in a package:
#   ros2 launch /opt/lab/lite6_bare.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('xarm_gazebo'), 'launch', 'lite6_beside_table_gazebo.launch.py',
        ])),
        # load_controller defaults to FALSE down in _robot_beside_table_gazebo.launch.py,
        # and lite6_beside_table_gazebo.launch.py does not override it. Without it you get
        # Gazebo and the gz_ros2_control hardware interface but no spawned controllers,
        # so nothing publishes /joint_states, robot_state_publisher emits no moving
        # transforms, and RViz draws a robot it cannot place. Hence: true.
        launch_arguments={'load_controller': 'true'}.items(),
    )

    # use_sim_time, because every transform Gazebo publishes is stamped with
    # simulation time; an RViz on wall clock treats them all as stale.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/opt/lab/lite6_bare.rviz'],
        parameters=[{'use_sim_time': True}],
    )

    # Started late: robot_description and the first transforms need to exist,
    # or RViz comes up with an empty model and you have to reload it by hand.
    return LaunchDescription([gazebo, TimerAction(period=10.0, actions=[rviz])])
