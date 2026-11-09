# xarm_gazebo: the lab's copy

This is a copy of `xarm_gazebo` from `xarm_ros2` (branch `jazzy`, as built into the image), kept
here so that it shadows the image's package once it is built (`lab build`). Everything is upstream
except one line:

| File | Change |
|---|---|
| `launch/_robot_beside_table_gazebo.launch.py` | the Gazebo spawn yaw of the robot, `-Y 1.571` → `-Y -1.571` |

Upstream spawns the Lite 6 on the edge of the table, facing off it. With the yaw changed it faces
-y, onto the table, where `lite6_checkers` puts the checkerboard (`-0.2, -0.74`: 0.24 m in front of
the base axis). Both `lab sim` and `lab sim-lite` use this launch file.

Only the `gz` branch of the launch file is changed; the `ignition` and classic Gazebo branches are
not used here.

To go back to upstream, delete this folder and run `lab build`; to see what differs from the image:

```bash
diff -r /opt/xarm_ws/src/xarm_ros2/xarm_gazebo ~/dev_ws/src/xarm_gazebo
```
