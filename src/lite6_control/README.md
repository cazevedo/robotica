# `lite6_control`

Worked example nodes for the UFACTORY Lite 6 — Robotics (02000537), 2026/2027.

## Two interfaces, and why it matters

There are two ways to command this arm, and they are **not** interchangeable:

| | `ros2_control` | driver API |
|---|---|---|
| interface | `/lite6_traj_controller/follow_joint_trajectory` | `/ufactory/set_servo_angle` |
| in Gazebo | **yes** (`lab sim`) | no |
| on hardware | **yes** (`lab real <ip>`) | yes (`lab driver <ip>`) |
| level | trajectories, through a controller | direct, one command at a time |

Only the first runs in simulation, so it is the only one where "test it in Gazebo
before the real arm" is possible. **Write your lab code against it.**

## The files

| | Works with | |
|---|---|---|
| `joint_echo.py` | `lab sim`, `lab real`, `lab driver` | the minimal node — subscribe, spin, print. The only one here that runs unchanged against both simulation and hardware, because it only reads |
| `move_joints_demo.py` | `lab driver` | self-contained driver-API example, nothing hidden in a helper |
| `lite6_client.py` | `lab driver` | the reusable `Lite6Client` class to import |
| `jog_demo.py` | `lab driver` | one joint at a time, built on `Lite6Client` |

## Running

Terminal 1, either one:

```bash
lab sim
```

```bash
lab real 192.168.1.xxx
```

Terminal 2, the same command against either:

```bash
ros2 run lite6_control joint_echo
```

For the driver-API nodes instead, terminal 1 is `lab driver 192.168.1.xxx`, and:

```bash
ros2 run lite6_control move_joints_demo --dry-run
```

## Building

```bash
lab build
```

**Usually unnecessary.** `--symlink-install` links the package rather than copying
it, so editing a `.py` takes effect on the next `ros2 run`. Rebuild only for a new
entry point, a changed `package.xml`/`setup.py`, or a new package — and re-source
(or open a new shell) only for a brand-new package.

## Writing your own

Start from `joint_echo.py` for the plumbing — node, subscription, spin — and from
`move_joints_demo.py` for commanding. The shape is: read `/joint_states` to find
where the arm is, build waypoints as offsets from there, send them, check the
result. Then add it to `entry_points` in `setup.py` and `lab build` once.

Nothing here currently commands both simulation and hardware from one command:
the demos that move the arm go through the driver's `/ufactory` services, which
Gazebo does not provide. A node that sends a `FollowJointTrajectory` goal to
`ros2_control` would work against both — that is the portable shape, and it is
the one worth writing.

Make waypoints relative to the current pose rather than absolute — the node is then
safe from any starting configuration and cannot swing the arm across the workspace
because someone left it somewhere unexpected.

## Four things that will cost you an afternoon

**Angles are radians.** The UFACTORY docs are mostly degrees; these interfaces are
not. Verified on the robot: `get_servo_angle` returns exactly what
`/ufactory/joint_states` publishes.

**`ret` is the real result** (driver API). A service call can return perfectly
happily with `ret=9` having done nothing. `Lite6Client` raises on any non-zero
`ret`; if you call the services directly, check it yourself. The action interface
has `error_code` on its result, for the same reason.

**`set_state()` and the reported state are different value spaces.** You *pass* 0
to make the arm ready; it then *reports* 2 (sleeping = idle and ready) or 1 (in
motion). A reported 0 never happens. Constants for both are in `lite6_client.py`.

**Don't make blocking calls inside callbacks.** Every `Lite6Client` method spins
the node waiting for a reply — call one from a subscription or timer callback and
you deadlock, because you are already inside the spin. Write your algorithm as a
straight-line procedure in `main()`. If you truly need both, that is a
`MultiThreadedExecutor` with a `ReentrantCallbackGroup`.

## Safety

On hardware this moves a real arm. Emergency stop within reach, nobody in the
workspace, `--dry-run` first. Nothing runs on the real robot that has not run in
Gazebo first.
