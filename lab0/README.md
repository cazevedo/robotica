# Lab 0 environment — ROS 2 Jazzy + MoveIt 2 + Gazebo Harmonic + Lite 6

Path **C** of the Lab 0 handout, as a container. Everything in Part 1 (Steps 1–8)
is baked into the image, including the `xarm_ros2` clone and `colcon build`, so
the 45–90 minutes are spent once, by `docker build`, and never again.

> Not to be confused with the Isaac Sim setup in `../isaac-sim`. Different base
> image, different purpose, nothing shared.

## Requirements

Docker on the host, and an X server if you want to see RViz and Gazebo (any
normal Linux desktop; WSLg on Windows). A GPU is not required — `run.sh` uses
one if the NVIDIA container toolkit is installed and does without otherwise.

## Getting started

```bash
cd lab0
cp .env.example .env          # put your assigned ROS_DOMAIN_ID in it
./run.sh build                # 45–90 min, once
./run.sh check                # environment self-test
./run.sh                      # a shell in the container
```

Run `./run.sh` again in another terminal and you land in the *same* container,
which is what the handout's "open a new terminal for each test" needs.

| | |
|---|---|
| `./run.sh` | shell in the container (starts it if needed) |
| `./run.sh check` | self-test: Tests 1, 2, 6, plus display and graphviz |
| `./run.sh start` | launch the Lite 6 stack (Test 3 or Test 7, per the flag below) |
| `./run.sh -- <cmd>` | run one command inside and exit |
| `./run.sh stop` | stop the container, keep its filesystem |
| `./run.sh restart` | throw it away and make a fresh one |
| `./run.sh status` | what is built, what is running, with which settings |
| `./run.sh build` / `rebuild` | build the image (`rebuild` = `--no-cache`) |

## The Gazebo flag

Gazebo is installed in the image but only runs when something launches it.
`ENABLE_GAZEBO` decides whether anything will:

```bash
./run.sh --no-gazebo      # this shell: lab-start uses fake controllers, and
                          # `lab-start gazebo` refuses with an explanation
./run.sh --gazebo         # this shell: back on
```

Settings resolve **flag > `lab0/.env` > default**, so put the ones you always
want in `.env` (`ENABLE_GAZEBO=0`) and type only the exceptions. The flag is
per-shell on purpose: you can keep a Gazebo terminal open while another terminal
works against the real arm, without physics eating the machine.

`lab-start` inside the container takes the back end directly:

```bash
lab-start gazebo    # Gazebo Harmonic + RViz + ros2_control     (Test 7)
lab-start fake      # RViz + MoveIt, fake controllers, no physics (Test 3)
lab-start real      # the physical arm; needs --robot-ip / ROBOT_IP
```

## Where your work lives

`lab0/dev_ws` on the host is `~/dev_ws` in the container, and it is the shell's
working directory. Anything you write there is on your disk, not in the
container — it survives `./run.sh restart`, `clean`, and a rebuild of the image.
You run as your own UID, so the files come out owned by you, not by root.

```bash
cd ~/dev_ws/src && ros2 pkg create --build-type ament_python my_pkg
lab-build                     # colcon build ~/dev_ws (--symlink-install)
```

New shells source `~/dev_ws/install/setup.bash` automatically when it exists.

Also persisted, in `lab0/.home`: the Gazebo model cache (so the "hangs for
minutes on first launch" happens once, not once per container), your RViz
layout, and shell history. Nothing else inside the container is kept — if you
`sudo apt install` something you want permanently, add it to the `Dockerfile`.

To edit in VS Code: open the `lab` folder and use **Dev Containers: Reopen in
Container**, or — better, because it is the same container your terminals are
in — start it with `./run.sh` and use **Dev Containers: Attach to Running
Container** → `robotica-lab`.

### One difference from the handout

The handout builds `xarm_ros2` into `~/dev_ws`. Here it is pre-built at
`/opt/xarm_ws` and `~/dev_ws` is left empty for your own packages, which overlay
it. Test 2 passes exactly the same way. If you ever want to modify the UFACTORY
packages themselves, clone them into your own workspace, where they will take
precedence and persist:

```bash
cd ~/dev_ws/src && git clone https://github.com/xArm-Developer/xarm_ros2.git --recursive -b jazzy
lab-build
```

## The seven tests

| Test | How to run it here |
|---|---|
| 1 — ROS 2 core | `./run.sh check`, or by hand with `ros2 run demo_nodes_cpp talker` in one `./run.sh` shell and `ros2 run demo_nodes_py listener` in another |
| 2 — packages visible | `ros2 pkg list \| grep -E "xarm\|uf_"` |
| 3 — robot model loads | `lab-start fake` — leave it running |
| 4 — kinematic chain | `ros2 run tf2_tools view_frames` from `~/dev_ws`, **not** from `~` — that way the PDF lands in `lab0/dev_ws` on the host where you can open it |
| 5 — read a transform | `ros2 run tf2_ros tf2_echo link_base link_eef` (use the names from your own PDF) |
| 6 — Gazebo alone | `gz sim shapes.sdf` |
| 7 — Lite 6 in Gazebo | `lab-start gazebo` |

## The real robot

```bash
./run.sh --robot-ip 192.168.1.xxx
ping 192.168.1.xxx          # before touching ROS
lab-start real
```

The container uses host networking, so it is on your machine's network
directly — if the host can reach the control box, so can the container, and
UFACTORY Studio at `http://<robot_ip>:18333` opens in your ordinary browser.

Two things that catch everybody: the Lite 6 namespace is `ufactory`, not `xarm`,
and the arm does not move until it is enabled —

```bash
ros2 service call /ufactory/motion_enable xarm_msgs/srv/SetInt16ById "{id: 8, data: 1}"
ros2 service call /ufactory/set_mode  xarm_msgs/srv/SetInt16 "{data: 0}"
ros2 service call /ufactory/set_state xarm_msgs/srv/SetInt16 "{data: 0}"
```

Nothing runs on the real robot that has not run in Gazebo first.

## When something breaks

**`ros2: command not found` inside the container** — you got a shell that did
not source `/etc/lab-env.sh`. `source /etc/lab-env.sh` fixes it; `./run.sh`
shells do it for you.

**RViz or Gazebo opens nothing, or `DISPLAY` is unset** — check
`./run.sh -- xeyes`. If that fails too it is the display plumbing, not ROS.
`./run.sh restart` re-creates the X cookie. On a host that refuses it entirely,
`xhost +local:` on the host is the blunt instrument.

**Black window or a crash in Gazebo** — no hardware GL. `./run.sh --software-gl`
(sets `LIBGL_ALWAYS_SOFTWARE=1`). Slow but usable.

**No display at all** (a machine you only reach over ssh) — everything still
runs against a fake screen, you just cannot see it:

```bash
xvfb-run -a lab-start gazebo
```

**Gazebo hangs for minutes on first launch** — normal; it is downloading models.
It is cached in `lab0/.home` afterwards, so it happens once.

**The listener hears nothing** — both terminals need the same `ROS_DOMAIN_ID`.
`./run.sh status` shows what the container was started with.

**The arm moves in RViz but not in Gazebo** — that is a genuine Test 7 failure.
Ask the controllers what they think:

```bash
ros2 control list_controllers
```

Then look in the launch output for `controller_manager` or `gz_ros2_control`,
and report the first error, not the last.

**`docker build` runs out of memory** — fewer colcon workers:
`COLCON_JOBS=1 ./run.sh build`.
