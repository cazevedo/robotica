# Lab 0 environment — Windows / Docker

**Robotics (02000537) — 2026/2027** · UFACTORY Lite 6 · ROS 2 Jazzy Jalisco · MoveIt 2 · Gazebo Harmonic

Path **C** of the Lab 0 handout, for Windows. Everything in Part 1 (Steps 1–8) is
baked into the image — including the `xarm_ros2` clone and its `colcon build` — so
that time is spent once, by `docker build`, and never again.

> The `ubuntu` branch of this repo has the Linux version of the same environment,
> plus an unrelated Isaac Sim setup. This branch is Windows only.

The [Dockerfile](Dockerfile) is commented step by step against the handout, so you
can read it as a record of exactly what was installed and why.

---

## Requirements

- Windows 10/11 with WSL2 available
- Docker Desktop, on the WSL 2 based engine
- ~20 GB free disk

```powershell
winget install -e --id Docker.DockerDesktop
```

Start Docker Desktop once and let it finish setting up. **Docker Desktop installs
its own WSL distribution — you do not need `wsl --install`.** A GPU is not
required; rendering falls back to Mesa's software rasteriser.

---

## Quick start

```powershell
git clone -b windows https://github.com/cazevedo/robotica.git
```

```powershell
cd robotica
```

Copy the settings file and put your assigned number in it:

```powershell
copy .env.example .env
```

```powershell
notepad .env
```

Then build — once, and it is mostly downloads:

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
workspaces in one go. Run it before asking anyone anything.

Then open a terminal inside the container — once per "new terminal" the handout
asks for:

```powershell
.\lab.ps1 shell
```

Inside, `lab` is the menu. `.\lab.ps1 down` stops everything; your work is kept.

> On Linux or in Git Bash, `./lab.sh` takes the same commands.

---

## Part 2 — the seven tests

| Handout | Inside the container | Needs Gazebo |
|---|---|---|
| 1 — ROS 2 core is alive | `lab test 1`, and `lab test 1b` in a second terminal | no |
| 2 — UFACTORY packages visible | `lab test 2` | no |
| 3 — robot model loads | `lab test 3` *(leave running)* | no |
| 4 — kinematic chain | `lab test 4` → PDF appears in `.\shared\` | no |
| 5 — read a transform | `lab test 5` | no |
| 6 — Gazebo on its own | `lab test 6` | **yes** |
| 7 — Lite 6 in Gazebo under MoveIt | `lab test 7` | **yes** |

Each prints the real `ros2` command before running it — copy that into your
submission if something fails.

Test 2 should list `uf_ros_lib`, `xarm_api`, `xarm_controller`, `xarm_description`,
`xarm_gazebo`, `xarm_moveit_config`, `xarm_msgs`, `xarm_planner` (and a couple
more). They come from `/opt/xarm_ws`, built into the image.

---

## The Gazebo flag

Gazebo is a physics engine; under software rendering it will use every core you
have. When you are driving the real arm you do not want it near your CPU.

```powershell
.\lab.ps1 gazebo off
```

```powershell
.\lab.ps1 gazebo on
```

or `lab gazebo off` / `on` / `status` from inside the container. It takes effect
immediately in every open terminal — no restart.

**What "off" does:** `lab sim`, `lab gz`, `lab test 6` and `lab test 7` refuse to
start and say why. Nothing can quietly spin up a simulation behind your back.
RViz, MoveIt, tf2 and the real-robot driver are unaffected:

```bash
lab moveit
```

It is a guard rail, not a throttle — Gazebo only costs CPU while it is running,
so the honest fix for "Gazebo is loading my PC" is not to launch it. `lab kill`
stops one that is already up.

Persisted in `.env` as `LAB_GAZEBO=0|1`; `lab gazebo` overrides it for the running
container, `lab gazebo reset` hands control back to `.env`.

---

## Where the windows appear

`DISPLAY_MODE` in `.env`. Default is `auto`.

| Mode | |
|---|---|
| `auto` | WSLg if available, otherwise VNC. Start here. |
| `wslg` | RViz and Gazebo open as ordinary Windows windows. Best. |
| `vnc` | The Linux desktop in your browser at <http://localhost:6080/>. Always works. `.\lab.ps1 vnc` opens it. |
| `x11` | An X server you run yourself (VcXsrv, X410); set `DISPLAY` too. |
| `none` | Headless. |

If `lab test 3` opens nothing, run `lab doctor` — it says whether the container
can reach an X server at all. Usual fix: `DISPLAY_MODE=vnc`, then
`.\lab.ps1 restart`.

Rendering is `LIBGL_ALWAYS_SOFTWARE=1`, because Docker Desktop gives the container
no GPU. RViz is comfortable; Gazebo is slow but usable. That is the expected
experience for Path C — say so when you submit if it is unusable, per the handout.

---

## Where your work lives

| On Windows | In the container | Survives |
|---|---|---|
| `.\src\` | `~/dev_ws/src` | everything, including `nuke` |
| `.\shared\` | `~/shared` | everything |
| — | `~/dev_ws/{build,install,log}` | restarts; deleted by `nuke` |
| — | `/opt/xarm_ws` (xarm_ros2, pre-built) | baked into the image |

`.\src\` is a bind mount — the same bytes under two names, no copy and no sync
step. Edit in VS Code on Windows, build and run in the container shell.

Build artefacts deliberately stay in a Docker volume: compiling across the
Windows/VM boundary is slow, and `colcon --symlink-install` needs symlinks that
are unreliable there.

**Anything else inside the container is disposable.** If you `pip install` or
`apt install` something, it disappears when the container is recreated — put it
in the `Dockerfile` instead.

### Editing inside the container

```powershell
.\lab.ps1 code
```

VS Code opens; choose **Reopen in Container** (needs the *Dev Containers*
extension). `vim` and `nano` are in there too.

### Workspace layering

`/opt/ros/jazzy` → `/opt/xarm_ws` (underlay, xarm_ros2) → `~/dev_ws` (your
overlay). All three are sourced by `~/.bashrc`, exactly as Step 7 of the handout
describes. A package in your overlay shadows one of the same name in the underlay
— which is what `lab overlay-xarm` exists for, if you ever need to modify
`xarm_ros2` itself.

---

## Your own ROS 2 code

`src/lite6_control` is a worked starter package — see
[its README](src/lite6_control/README.md).

| | |
|---|---|
| `joint_echo` | the minimal node: subscribe, spin, print. Commands nothing. |
| `move_joints_demo` | self-contained: read joint states, command joint positions, measure the error |
| `jog_demo` | one joint at a time, built on the reusable client |
| `lite6_client.py` | the `Lite6Client` class to import into your own nodes |

```bash
lab build
```

**You usually don't need to.** `lab build` uses `colcon build --symlink-install`,
so editing a `.py` takes effect on the next `ros2 run` with no rebuild. Rebuild
only when you add an entry point, change `package.xml`/`setup.py`, or create a new
package — and re-source (or open a new shell) only for a brand-new package.

---

## The real robot

Nothing extra to install. Read the handout's safety appendix first, and the course
rule stands: **nothing runs on the real robot that has not run in Gazebo first.**

Your PC needs a static IP on the arm's subnet (the control box IP is on a label on
the base). Set `ROBOT_IP` in `.env`, then:

```bash
lab real 192.168.1.xxx
```

```bash
lab enable
```

Bridge networking is enough — the container reaches the control box through NAT on
all the ports the driver uses (502, 30001, 30003, and 18333 for UFACTORY Studio).
Host networking is only needed for ROS 2 discovery with *other machines* on the lab
LAN; see the commented block in [docker-compose.yml](docker-compose.yml).

### Three things that catch everybody

**The namespace is `/ufactory`, not `/xarm`.** Most tutorials online are written
for the xArm series. `lab enable` already uses the right one.

**One owner of the arm at a time.** `lite6_moveit_realmove.launch.py` hands the arm
to `ros2_control`, which holds it in **mode 1** (servo streaming). `set_servo_angle`
needs **mode 0**. Run one or the other, not both — otherwise your service calls are
silently ignored. Check with:

```bash
ros2 topic echo /ufactory/robot_states --once
```

**`set_state()` and the reported state are different value spaces.** You *pass* 0
to make the arm ready; it then *reports* 2 (sleeping = idle and ready) or 1 (in
motion). A reported 0 never happens. Likewise `ret` in every service response is
the real result — a call can return perfectly happily having done nothing.

---

## When it goes wrong

| Symptom | Do this |
|---|---|
| `Docker is not installed (or not on PATH)` | Install it. If you just did, `lab.ps1` re-reads PATH itself — this message means it really is absent. |
| `the engine is not responding` | Docker Desktop is installed but not started. Launch it and wait for the whale to stop animating. |
| Build fails | The **first** error matters, not the last. Scroll up. |
| Build killed / machine freezes | Lower `COLCON_JOBS` in `.env`, then `.\lab.ps1 rebuild`. |
| `ros2: command not found` inside | A shell that skipped `~/.bashrc`. Run `bash -l`. |
| `Package 'xarm_moveit_config' not found` | `lab doctor`. If the underlay is missing, `.\lab.ps1 rebuild`. |
| RViz/Gazebo open nothing | `lab doctor`, then `DISPLAY_MODE=vnc` + `.\lab.ps1 restart`. |
| Gazebo hangs for minutes on first launch | Normal — Fuel models and shaders. Cached in a volume, so only once. |
| A node you deleted still shows in `ros2 pkg executables` | `colcon` never prunes `install/`. `lab clean`, then `lab build`. |
| Arm moves in RViz but not in Gazebo | The `ros2_control` bridge. Find `controller_manager` / `gz_ros2_control` in the launch output, copy the first error. |
| Everything is confused | `.\lab.ps1 nuke` then `.\lab.ps1 up`. `.\src` and `.\shared` survive. |
| `xarm_ros2` upstream moved | `.\lab.ps1 rebuild-xarm` — fresh recursive clone, fresh build. |

Container logs are UTC; your Windows clock is local. Timestamps will look offset.

---

## What to submit

1. Path **C**.
2. Pass/fail for Tests 1–7.
3. Two 4×4 matrices from Test 5 (poses A and B).
4. One sentence from Test 7 on something Gazebo showed that RViz did not.
5. For failures: the **first** error message, as text.

`.\shared\` is a good place to keep all of it — it is a normal Windows folder.

---

## Files

```
Dockerfile              Steps 1-8 of the handout, commented per step
docker-compose.yml      volumes, display, ports, the LAB_GAZEBO switch
.env.example            settings template (ROS_DOMAIN_ID, Gazebo, display)
lab.ps1                 Windows driver: build / up / shell / gazebo / doctor
lab.sh                  the same, for Git Bash / WSL / Linux / macOS
docker/lab              in-container helper: the seven tests and the launches
docker/entrypoint.sh    sources ROS, sets up the display, prints the banner
docker/ros_setup.sh     Step 7 of the handout, sourced by every shell
docker/start-display    WSLg / VNC / external X
.devcontainer/          VS Code "Reopen in Container"
src/lite6_control/      worked example ROS 2 Python package
src/                    YOUR packages        (= ~/dev_ws/src)
shared/                 files in and out     (= ~/shared)
```
