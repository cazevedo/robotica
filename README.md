# Lab 0 environment — Windows / Docker

**Robotics (02000537) — 2026/2027** · UFACTORY Lite 6 · ROS 2 Jazzy Jalisco · MoveIt 2 · Gazebo Harmonic

Path **C** of the Lab 0 handout, for Windows. Everything in Part 1 (Steps 1–8) is
baked into the image — including the `xarm_ros2` clone and its `colcon build` — so
that time is spent once, by `docker build`, and never again.

> The `ubuntu` branch has the Linux version of the same environment. This branch
> is Windows only. The [Dockerfile](Dockerfile) is commented step by step against
> the handout.

---

## First time

Docker Desktop, on the WSL 2 based engine. It installs its own WSL distribution —
you do **not** need `wsl --install`. No GPU required.

```powershell
winget install -e --id Docker.DockerDesktop
```

Then start Docker Desktop once and let it finish. In this folder:

```powershell
copy .env.example .env
```

Put your assigned `ROS_DOMAIN_ID` in `.env`, then build — once, mostly downloads:

```powershell
.\lab.ps1 build
```

```powershell
.\lab.ps1 up
```

```powershell
.\lab.ps1 doctor
```

`doctor` checks ROS, the xarm packages, the display, the GL renderer and both
workspaces. Run it before asking anyone anything.

Every terminal after that is `.\lab.ps1 shell`, which drops you into the same
running container. Inside, `lab` is the menu. `.\lab.ps1 down` stops everything;
your work is kept. On Linux or in Git Bash, `./lab.sh` takes the same commands.

---

# The development pipeline

**This is how you work.** Two terminals: one runs the robot, the other runs your
node. Write your node once and it runs against both.

## 1. In simulation

```powershell
.\lab.ps1 gazebo on
```

**Terminal 1** — Gazebo, MoveIt and `ros2_control`, all from one launch:

```powershell
.\lab.ps1 shell
```

```bash
lab sim
```

Wait for the Lite 6 to appear in the Gazebo window standing under gravity — up to
a minute, and several minutes the very first time while Gazebo downloads models.

**Terminal 2** — your node:

```powershell
.\lab.ps1 shell
```

```bash
ros2 run lite6_control trajectory_demo
```

## 2. On the real robot

**Prerequisite: it ran in Gazebo first.** That is the course rule, and Test 7 is
the gate.

**Terminal 1** — the same stack, with real hardware underneath:

```powershell
.\lab.ps1 shell
```

```bash
lab real 192.168.1.xxx
```

**Terminal 2** — the *identical* command as in simulation, not edited:

```powershell
.\lab.ps1 shell
```

```bash
ros2 run lite6_control trajectory_demo
```

Set `ROBOT_IP` in `.env` and you can drop the address: `lab real`.

## Why the same node works in both

`lab sim` and `lab real` load the same `xarm_controller/config/lite6_controllers.yaml`,
so both expose the same interface:

```
/lite6_traj_controller/follow_joint_trajectory     control_msgs/action/FollowJointTrajectory
/joint_states                                      sensor_msgs/msg/JointState
```

Only the hardware behind it changes — `gz_ros2_control` in simulation,
`uf_robot_hardware` on the arm. A node written against that action does not know
or care which, and that is what makes "Gazebo first, then hardware" a real
workflow rather than a slogan.

Watch the tracking error your node prints. Against fake controllers it is ~1e-5 rad;
in Gazebo it is far larger, because Gazebo simulates mass, gravity and friction
and the arm sags and overshoots. That difference *is* Test 7.

## The other path: talking to the driver directly

```bash
lab driver 192.168.1.xxx
```

This starts the bare xArm driver instead — `/ufactory/set_servo_angle`,
`/ufactory/motion_enable` and friends, with **no `controller_manager` at all**.
Lower level and more direct, and what `move_joints_demo` and `jog_demo` use.

**It has no Gazebo equivalent**, so a node written this way cannot be tested in
simulation. Use it when you specifically want the driver API; use `lab sim` /
`lab real` for anything you want to develop in simulation first.

**One owner of the arm at a time.** `lab real` hands the arm to `ros2_control`,
which holds it in **mode 1** (servo streaming). `set_servo_angle` needs **mode 0**.
Run one or the other — with both, your service calls are silently ignored. Check
with:

```bash
ros2 topic echo /ufactory/robot_states --once
```

---

## Your code

`src/lite6_control` is a worked package — see [its README](src/lite6_control/README.md).

| Node | Works with | |
|---|---|---|
| `trajectory_demo` | `lab sim`, `lab real` | **the one to copy.** Sends a joint trajectory through `ros2_control` |
| `joint_echo` | anything | minimal subscriber: subscribe, spin, print |
| `move_joints_demo` | `lab driver` | self-contained; the driver API, read and command |
| `jog_demo` | `lab driver` | one joint at a time, via the reusable `Lite6Client` |

Start your own package in `src/`:

```bash
ros2 pkg create --build-type ament_python my_pkg --dependencies rclpy control_msgs trajectory_msgs
```

```bash
lab build
```

`lab build` uses `--symlink-install`, so **editing a `.py` needs no rebuild**.
Rebuild only when you add an entry point, change `package.xml`/`setup.py`, or
create a package — and re-source (or just open a new shell) only for a brand-new
package.

---

## Part 2 — the seven tests

| Handout | Inside the container | Gazebo |
|---|---|---|
| 1 — ROS 2 core is alive | `lab test 1`, and `lab test 1b` in a second terminal | no |
| 2 — UFACTORY packages visible | `lab test 2` | no |
| 3 — robot model loads | `lab test 3` *(leave running)* | no |
| 4 — kinematic chain | `lab test 4` → PDF in `.\shared\` | no |
| 5 — read a transform | `lab test 5` | no |
| 6 — Gazebo on its own | `lab test 6` | **yes** |
| 7 — Lite 6 in Gazebo under MoveIt | `lab test 7` | **yes** |

Each prints the real `ros2` command before running it — copy that into your
submission if something fails.

**To submit:** path C; pass/fail for 1–7; the two 4×4 matrices from Test 5; one
sentence from Test 7 on something Gazebo showed that RViz did not; and for any
failure, the **first** error message as text.

---

## The Gazebo flag

Gazebo is a physics engine; under software rendering it uses every core you have.
When you are driving the real arm you do not want it near your CPU.

```powershell
.\lab.ps1 gazebo off
```

`lab sim`, `lab gz` and tests 6–7 then refuse to start and say why — nothing can
quietly spin up a simulation behind your back. `lab moveit` (RViz + fake
controllers), `lab real` and `lab driver` are unaffected. Effective immediately in
every open terminal; `lab kill` stops a simulation already running.

It is a guard rail, not a throttle: Gazebo only costs CPU while it runs. Stored in
`.env` as `LAB_GAZEBO=0|1`, overridden for the running container by `lab gazebo
on|off`, and `lab gazebo reset` hands control back to `.env`.

---

## Where the windows appear

`DISPLAY_MODE` in `.env`, default `auto`: WSLg if Docker Desktop exposes it —
RViz and Gazebo open as ordinary Windows windows — otherwise a noVNC desktop at
<http://localhost:6080/> (`.\lab.ps1 vnc`). Force either with `wslg` or `vnc`;
`x11` is for your own VcXsrv/X410, `none` is headless.

If `lab test 3` opens nothing, `lab doctor` says whether the container can reach
an X server at all. Usual fix: `DISPLAY_MODE=vnc`, then `.\lab.ps1 restart`.

Rendering is `LIBGL_ALWAYS_SOFTWARE=1` — Docker Desktop gives the container no
GPU. RViz is comfortable, Gazebo is slow but usable. That is the expected Path C
experience; say so when you submit if it is unusable.

---

## Where your work lives

| On Windows | In the container | |
|---|---|---|
| `.\src\` | `~/dev_ws/src` | your packages — survives everything, including `nuke` |
| `.\shared\` | `~/shared` | files in and out (the `view_frames` PDF, notes) |
| — | `~/dev_ws/{build,install,log}` | Docker volume; `nuke` deletes it, `lab build` recreates it |
| — | `/opt/xarm_ws` | xarm_ros2, pre-built into the image |

`.\src\` is a bind mount — the same bytes under two names, no copy and no sync.
Edit in VS Code on Windows, build and run in the container shell. Build artefacts
stay in a Docker volume deliberately: compiling across the Windows/VM boundary is
slow, and `--symlink-install` needs symlinks that are unreliable there.

**Anything else in the container is disposable.** A `pip install` or `apt install`
disappears when the container is recreated — put it in the `Dockerfile`.

`.\lab.ps1 code` opens VS Code; choose **Reopen in Container** for an editor and
debugger running inside, with ROS sourced.

Workspace layering is `/opt/ros/jazzy` → `/opt/xarm_ws` → `~/dev_ws`, all sourced
by `~/.bashrc` as in Step 7 of the handout. Your overlay shadows the underlay,
which is what `lab overlay-xarm` is for if you ever need to modify `xarm_ros2`.

---

## When it goes wrong

| Symptom | Do this |
|---|---|
| `Docker is not installed (or not on PATH)` | Install it. If you just did, `lab.ps1` re-reads PATH itself — this means it really is absent. |
| `the engine is not responding` | Docker Desktop installed but not started. Launch it, wait for the whale to settle. |
| Build fails | The **first** error matters, not the last. Scroll up. |
| Build killed / machine freezes | Lower `COLCON_JOBS` in `.env`, then `.\lab.ps1 rebuild`. |
| `no action server on /lite6_traj_controller/...` | You are on `lab driver`, which has no `controller_manager`. Use `lab sim` or `lab real`. |
| Service calls ignored, arm does not move | Two owners. `ros2 topic echo /ufactory/robot_states --once` — mode 1 means `ros2_control` has it. |
| `ros2: command not found` inside | A shell that skipped `~/.bashrc`. Run `bash -l`. |
| RViz/Gazebo open nothing | `lab doctor`, then `DISPLAY_MODE=vnc` + `.\lab.ps1 restart`. |
| Gazebo hangs for minutes on first launch | Normal — Fuel models and shaders, cached in a volume, once only. |
| A deleted node still shows in `ros2 pkg executables` | `colcon` never prunes `install/`. `lab clean`, then `lab build`. |
| Everything is confused | `.\lab.ps1 nuke` then `.\lab.ps1 up`. `.\src` and `.\shared` survive. |
| `xarm_ros2` upstream moved | `.\lab.ps1 rebuild-xarm` — fresh recursive clone and build. |

Container logs are UTC; your Windows clock is local. Timestamps will look offset.

---

## Files

```
Dockerfile              Steps 1-8 of the handout, commented per step
docker-compose.yml      volumes, display, ports, the LAB_GAZEBO switch
.env.example            settings template (ROS_DOMAIN_ID, Gazebo, display)
lab.ps1 / lab.sh        host driver: build / up / shell / gazebo / doctor
docker/lab              in-container helper: the seven tests and the launches
docker/entrypoint.sh    sources ROS, sets up the display, prints the banner
docker/ros_setup.sh     Step 7 of the handout, sourced by every shell
docker/start-display    WSLg / VNC / external X
.devcontainer/          VS Code "Reopen in Container"
src/lite6_control/      worked example ROS 2 Python package
src/                    YOUR packages     (= ~/dev_ws/src)
shared/                 files in and out  (= ~/shared)
```
