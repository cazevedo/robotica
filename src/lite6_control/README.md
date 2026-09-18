# `lite6_control`

Python ROS 2 nodes for the UFACTORY Lite 6 — Robotics (02000537), 2026/2027.

```
lite6_control/
├── lite6_client.py   the reusable wrapper over the /ufactory services
├── joint_echo.py     minimal subscriber — read this first, moves nothing
└── jog_demo.py       a real algorithm: exercise each joint, measure the result
```

## Running it

The driver must be up and owning the arm:

```bash
lab driver 192.168.1.181
```

Then, in another terminal:

```bash
ros2 run lite6_control joint_echo --ros-args -p degrees:=true
```

```bash
ros2 run lite6_control jog_demo --dry-run
```

```bash
ros2 run lite6_control jog_demo --joints 0 --delta 0.1
```

## Building

```bash
lab build
```

**You usually don't need to.** `lab build` uses `colcon build --symlink-install`,
which symlinks the Python package rather than copying it, so editing a `.py`
file takes effect on the next `ros2 run` with no rebuild at all.

Rebuild only when you change `setup.py` (a new entry point), `package.xml`, or
add a new node.

## Writing your own node

Copy `jog_demo.py` and change the middle. The shape is:

```python
rclpy.init()
arm = Lite6Client()
try:
    arm.enable()                    # motion_enable -> mode 0 -> state 0
    arm.require_position_mode()     # fail loudly if joint commands can't work
    # ... your algorithm here ...
finally:
    arm.destroy_node()
    rclpy.shutdown()
```

Add it to `entry_points` in `setup.py`, then `lab build` once.

## Four things that will cost you an afternoon

**Angles are radians.** The UFACTORY docs are mostly in degrees; these services
are not. Verified on the robot: `get_servo_angle` returns exactly what
`/ufactory/joint_states` publishes.

**`ret` is the real result.** A service call can return perfectly happily with
`ret=9` and do nothing. `Lite6Client` raises on any non-zero `ret`, so the arm
refusing you is loud rather than silent. If you call the services directly,
check `ret` yourself.

**Mode and state gate everything.** `set_servo_angle` needs mode 0 and state 0.
If MoveIt / `ros2_control` is running it holds the arm in mode 1 and your
commands vanish without a trace. One owner of the arm at a time: either the
realmove stack *or* your node. `arm.describe_state()` prints mode/state/err in
one line.

**Don't make blocking calls inside callbacks.** Every `Lite6Client` method spins
the node waiting for a reply. Call one from inside a subscription or timer
callback and you deadlock — you are already inside the spin. Write your
algorithm as a straight-line procedure in `main()`. If you genuinely need both,
that's a `MultiThreadedExecutor` with a `ReentrantCallbackGroup`.

## Safety

This moves a real arm. Emergency stop within reach, nobody inside the workspace,
`--dry-run` first, one joint before six. The course rule stands: nothing runs on
the real robot that has not run in Gazebo first.
