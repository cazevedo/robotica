# `lite6_kinematics` — Labwork 1

Forward kinematics of the UFACTORY Lite 6, as a ROS 2 node — Robotics (02000537), 2026/2027.
The handout is the specification; this file is the map of the package.

## The files

Every file of the package. You implement one, and you do not touch the rest.

| File | | Purpose |
|---|---|---|
| `lite6_kinematics/forward_kinematics.py` | **to implement** | `dh_table`, `mdh_transform`, `forward_kinematics` and the node `FkNode`. The only file you edit. It also holds `pose_from_transform`, `quaternion_from_matrix` and `main`, which are **provided** — do not edit them |
| `lite6_kinematics/fk_check.py` | provided | the checker node: moves the arm through the ten poses and compares the pose your node publishes with tf and, on the real arm, with the controller |
| `lite6_kinematics/checking.py` | provided | the checker's helpers: reference poses as 4×4 matrices, the error metrics e_p and e_o |
| `lite6_kinematics/__init__.py` | provided | empty; makes the folder a Python package |
| `config/lw1_poses.yaml` | provided | the ten joint configurations the checker visits |
| `test/test_forward_kinematics.py` | provided | the unit tests; the node test needs ROS, so it runs inside the container |
| `package.xml` | provided | the ROS 2 package description and its dependencies |
| `setup.py` | provided | installs the package and registers the `fk_node` and `fk_check` commands |
| `setup.cfg` | provided | where `ros2 run` finds the installed command |
| `resource/lite6_kinematics` | provided | empty marker that registers the package with ROS 2 |
| `README.md` | provided | this file |

The provided files are compared against the originals at grading. Edit them and
your results do not count.

`forward_kinematics.py` may import `numpy` and `math` for the maths, and `rclpy`,
`sensor_msgs` and `geometry_msgs` for the node (already imported in the template),
nothing else.

## Conventions

Craig's modified DH. Frame {0} = `link_base`, frame {6} = `link_eef` (the
end-effector frame). Joint angles in **radians**, exactly as on `/joint_states`.
Lengths in **metres**.

Your node `fk_node` reads `/joint_states` and publishes the pose of `link_eef`
in `link_base` on `/lite6_fk/pose` (`geometry_msgs/PoseStamped`, position in
metres, orientation as a quaternion). Its `header.stamp` is the stamp of the
joint state it was computed from.

All commands below run inside the container, in a shell opened as described in
the top-level README.

## 1. Tests

```bash
python3 -m pytest ~/dev_ws/src/lite6_kinematics/test -v
```

All green before you go further. They check structure and units, and that your
node publishes the pose of the joint states it receives — not whether your table
is right; that is step 2. The `pose_from_transform` and `quaternion_from_matrix`
tests pass from the start: they guard the provided code, not your work. The
maths tests need no ROS; the node test does, so it runs inside the container.

## 2. Gazebo

Once, to register the package and its `fk_node` and `fk_check` commands:

```bash
lab build
```

Then open a new shell (a brand-new package needs re-sourcing).

**Terminal 1:**

```bash
lab sim
```

**Terminal 2** — your node:

```bash
ros2 run lite6_kinematics fk_node
```

Watch what it publishes while you move the arm in RViz:

```bash
ros2 topic echo /lite6_fk/pose
```

**Terminal 3** — the checker:

```bash
ros2 run lite6_kinematics fk_check --ros-args -p robot:=sim
```

The arm visits ten poses. At each one the checker reads the pose your node
published, prints it and its distance from `link_base → link_eef` in tf.
Result: `~/shared/lw1_fk_sim.csv`.

The robot model that tf and Gazebo use differs slightly from the manufacturer's
sheet, so a correct implementation shows about 1 mm of position error here.
That is within P1.3.

## 3. The real arm

Only Part D moves the physical robot. The rules of Labwork 0 part 1 §3 apply
without exception: reduced mode on, one group member at the emergency stop, no
hands within the workspace while the robot is enabled.

**Terminal 1:**

```bash
lab real 192.168.1.xxx
```

**Terminal 2** — your node, the same one:

```bash
ros2 run lite6_kinematics fk_node
```

**Terminal 3:**

```bash
ros2 run lite6_kinematics fk_check --ros-args -p robot:=real
```

Same ten poses, slowly, one at a time: it waits for Enter before every motion.
Ctrl+C quits between motions; **it does not stop a motion in progress — the
emergency stop does.** Besides tf, it compares with the pose the arm's own
controller reports (`/ufactory/robot_states`). Result: `~/shared/lw1_fk_real.csv`.
