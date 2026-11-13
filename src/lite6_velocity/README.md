# `lite6_velocity` — Labwork 3

Velocity control of the UFACTORY Lite 6 along a circle, open loop and closed loop, as a ROS 2
node — Robotics (02000537), 2026/2027. The handout is the specification; this file is the map of
the package.

This package is **new and separate**: it sits beside `lite6_kinematics` and `lite6_ik` and uses
them. Your Labwork 1 `forward_kinematics.py` and your Labwork 2 `inverse_kinematics.py` stay where
they are and are not touched; this package imports them.

## The files

| File | | Purpose |
|---|---|---|
| `lite6_velocity/velocity_control.py` | **LW3: to implement** | `circle_reference`, `jacobian`, `open_loop_step`, `closed_loop_step`, the node `VelocityNode` and the constants `KP`, `DT`, `QDOT_MAX`, `SIGMA_MIN`. The only file of this package you edit. It also holds `pose_error`, `rotation_vector`, `load_circle` and `main`, which are **provided** — do not edit them |
| `config/lw3_circles.yaml` | provided | the circles C1, C2 and C3 |
| `lite6_velocity/__init__.py` | provided | empty; makes the folder a Python package |
| `package.xml`, `setup.py`, `setup.cfg`, `resource/lite6_velocity` | provided | the ROS 2 package description; `setup.py` registers the `vel_node` command |
| `README.md` | provided | this file |

There is no checker in this package: the measurement and verification tools are yours to
design and write (handout, Part C), in files of your own, outside this package.

## Getting the Labwork 3 files

Labwork 3 only **adds** a new folder, `src/lite6_velocity`. In the repository root:

```powershell
git pull
```

Then, inside the container, once, to register the new package and its command:

```bash
lab build
```

and open a new shell.

## Running the node

**Gazebo.** Terminal 1:

```bash
lab sim-lite
```

Terminal 2 — your node, with the simulation's clock (`use_sim_time`), because Gazebo does not run
at real time and the trajectory controller uses the simulation's clock:

```bash
ros2 run lite6_velocity vel_node --ros-args -p use_sim_time:=true -p circle:=C1 -p law:=closed -p laps:=3
```

Your own circle: `-p center:="[0.25, 0.0, 0.2]" -p radius:=0.04` instead of `-p circle:=`.

**The real arm**, only after Gazebo is done (the rules of Labwork 0 part 1 §3 apply without
exception). Terminal 1: `lab real 192.168.1.xxx`. Terminal 2: the same command **without**
`use_sim_time`.

The node publishes its state on `/lite6_vel/status` (`moving_to_start`, `running`, `done`,
`singular`, `too_fast`, `unreachable`) and the desired pose on `/lite6_vel/reference`:

```bash
ros2 topic echo /lite6_vel/status
```

## Conventions

Those of Labworks 1 and 2: frame {0} = `link_base`, frame {6} = `link_eef`, joint angles in
**radians**, in the controller's convention, lengths in **metres**, time in **seconds**.
