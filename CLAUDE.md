# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A Docker setup combining NVIDIA Isaac Sim with ROS 2 Jazzy in a single image.

- Base image: `nvcr.io/nvidia/isaac-sim:6.0.1` (Ubuntu 24.04 / Noble), which is what makes installing ROS 2 Jazzy (Noble-targeted) directly on top possible.
- ROS 2 package set is `ros-jazzy-ros-base` (headless, no rviz/GUI tools) plus `ros-jazzy-vision-msgs` and `ros-jazzy-ackermann-msgs`, which Isaac Sim's ROS 2 bridge extension references.
- Display mode is WebRTC streaming (`./runheadless.sh`), not X11 passthrough — chosen so the container also works headlessly/remotely later.

**The apt-installed ROS 2 Jazzy and Isaac Sim's ROS 2 bridge are deliberately kept separate, not wired together.** Isaac Sim's `isaacsim.ros2.bridge` extension dynamically loads ROS 2 client/RMW libraries *into the Isaac Sim process itself* (it's not a separate ROS node), and it ships its own complete, self-contained copy of the Jazzy libraries at `/isaac-sim/exts/isaacsim.ros2.core/jazzy/lib` (344 `.so` files, message packages included) for exactly this purpose. Isaac Sim's own launch chain (`runheadless.sh` -> `isaac-sim.streaming.sh` -> `setup_ros_env.sh`) auto-detects Ubuntu 24.04 and wires the bridge up to that bundled copy automatically — but **only if `$ROS_DISTRO` is unset at that point**; if it's already set (e.g. by an image-wide `ENV ROS_DISTRO=jazzy`, or by sourcing `/opt/ros/jazzy/setup.bash` before launch), `setup_ros_env.sh` assumes the caller already configured `LD_LIBRARY_PATH` correctly and skips its own setup — and the caller's env did not survive into the actual `kit` process in testing, so the bridge failed with `librcutils.so: cannot open shared object file`. See "Known-bad configuration" below.

## Repository layout — two unrelated Docker setups

```
isaac-sim/            Isaac Sim 6.0.1 + ROS 2 Jazzy, WebRTC-streamed. Everything
                      else in this file is about this one; bare filenames below
                      (Dockerfile, run.sh, autoload_stage.py, content/,
                      ros2_ws/) are relative to it.
Dockerfile, docker/,  The course's Lab 0 environment (handout path C), at the
docker-compose.yml,   repository root. See README.md; driven with ./lab.sh on
lab.sh, lab.ps1,      Linux/macOS or .\lab.ps1 on Windows.
src/, shared/
```

The root-level Lab 0 environment is **separate and self-contained**: ROS 2 Jazzy desktop + MoveIt 2 + Gazebo Harmonic + a pre-built `xarm_ros2` workspace, driven over X11/VNC rather than WebRTC. It shares no layers, no image and no volumes with the Isaac Sim image, and none of the ROS-bridge constraints below apply to it — in particular it *does* set `ENV ROS_DISTRO=jazzy` and source `/opt/ros/jazzy/setup.bash` on every shell, which is only a problem in an image that contains Isaac Sim. Changes to one are not changes to the other.

Its `.dockerignore` is `*` plus `!docker/`, so `isaac-sim/` never enters that image's build context, and `docker-compose.yml` mounts only `./src` and `./shared`. The two can coexist at the root without interfering.

The apt-installed `/opt/ros/jazzy` is for *your own* ROS 2 usage — CLI tools, `colcon` workspaces, custom nodes — used interactively (`docker exec` / `--entrypoint bash`, where `/etc/bash.bashrc` sources it automatically). It talks to the bridge over DDS like any other ROS 2 node would, not by sharing libraries with it.

## Commands

Build and run:
```bash
./isaac-sim/run.sh
```
This builds the image, creates the host cache/config directories Isaac Sim expects, opens up permissions on the ones your user still owns (see below), and runs the container with GPU access, host networking (required for WebRTC), and the standard Isaac Sim volume mounts (shader/compute cache, logs, config, data).

Debug shell:
```bash
docker run --rm -it --gpus all --network=host -e "ACCEPT_EULA=Y" \
  --entrypoint bash isaac-sim-ros2-jazzy:latest
```
From there, `ros2 topic list` (ROS 2 is auto-sourced via `/etc/bash.bashrc` in interactive shells) is a quick sanity check of the apt-installed ROS 2 install. To launch Isaac Sim manually from this shell, just run `./runheadless.sh` — don't `source /opt/ros/jazzy/setup.bash` first, or you'll reproduce the bridge bug described below.

Connect to the running simulation once the container is up. This setup (the plain `nvcr.io/nvidia/isaac-sim` image, no extra services) exposes the native Kit streaming server on port 49100 — confirmed listening via `ss -tlnp` and responding (501 to a plain GET, i.e. it's the WebRTC signaling protocol, not an HTTP page). That only works with NVIDIA's dedicated **Isaac Sim WebRTC Streaming Client** desktop app, not a plain browser tab:
```bash
curl -LO https://downloads.isaacsim.nvidia.com/isaacsim-webrtc-streaming-client-2.0.0-linux-x86_64.deb
sudo dpkg -i ./isaacsim-webrtc-streaming-client-2.0.0-linux-x86_64.deb
sudo apt -f install
isaacsim-webrtc-streaming-client
```
Enter `127.0.0.1` as the server and connect. No container-side changes needed — `--network=host` already exposes everything the client needs (signaling on 49100, media on the 47995–48012 / 49000–49007 UDP/TCP ranges).

**Connecting from another machine on the LAN needs `ISAACSIM_HOST` set to this host's real LAN IP, or you get signaling-succeeds-but-black-screen.** The signaling connection (port 49100) is a plain TCP connection, so it succeeds to any reachable address, including from a remote machine — but the WebRTC extension separately advertises an address of its own choosing to negotiate the actual media session, controlled by the `omni.kit.livestream.app` extension's `primaryStream.publicIp` setting (default `""`, i.e. auto-detect; see `apps/isaacsim.exp.full.streaming.kit` in the base image). `runheadless.sh` maps an `ISAACSIM_HOST` env var straight to that setting (`--/exts/omni.kit.livestream.app/primaryStream/publicIp=${ISAACSIM_HOST}`) but `run.sh` didn't set it, so with `--network=host` exposing every interface on the host (Docker bridges, libvirt, CNI/flannel, the real LAN NIC), auto-detect confirmed empirically to pick one a remote client can't reach — signaling connects (so the client shows the app, not a connection error), but no media arrives, hence black screen. Fixed by having `run.sh` auto-detect the host's actual LAN IP via `ip route get 1.1.1.1` (the `src` field) and pass it as `ISAACSIM_HOST`; override by exporting `ISAACSIM_HOST` yourself before running `run.sh` if that auto-detection ever picks the wrong interface.

A browser-based client (e.g. a URL like `http://127.0.0.1:8211/streaming/webrtc-client/`) does exist, but it's served by a *separate* NVIDIA Docker Compose "web viewer" bundle with its own proxy/web-server container and `ISAACSIM_SIGNAL_PORT` / `ISAACSIM_STREAM_PORT` / `WEB_VIEWER_PORT` env vars — not something the plain image serves on its own. Not set up here; the desktop client above is simpler and already works with this setup.

Persist an environment/robot so it auto-loads next time: in the Isaac Sim GUI (via the WebRTC client), **File > Save As** to `/isaac-sim/content/scene.usd` — that path is bind-mounted from `isaac-sim/content` in the repo checkout on the host (see `run.sh`), so it survives the container being removed (it runs with `--rm`; nothing saved outside a mounted path persists). The next `./isaac-sim/run.sh` opens that exact file automatically instead of the default blank stage — see "Stage autoload" below. The filename must be exactly `scene.usd`; that's what `autoload_stage.py` looks for. To go back to a blank stage, remove/rename `isaac-sim/content/scene.usd`.

## ROS 2 robot control

`autoload_stage.py` calls `setup_ros2_control_graph()` (in `ros2_control_graph.py`) right after opening the saved stage, then starts timeline playback (`omni.timeline`'s `.play()`) — the control graph's nodes only execute on playback ticks, and physics (so drive commands actually move anything) only steps while playing, so this happens automatically on every container start rather than requiring someone to open the WebRTC client and press Play first.

`setup_ros2_control_graph()` builds an OmniGraph (`/World/ROS2ControlGraph`) that wires:
- **`isaacsim.ros2.bridge.ROS2SubscribeJointState`** (topic `joint_command`, `sensor_msgs/msg/JointState`) →
- **`isaacsim.core.nodes.IsaacArticulationController`** (targets the Lite6's articulation root) →
  the robot's joints, plus a **`isaacsim.ros2.bridge.ROS2PublishJointState`** publishing feedback on `joint_states`.

It's a no-op (prints and returns) if `/World/lite6` isn't present in the opened stage, and it's idempotent (skips if `/World/ROS2ControlGraph` already exists) — it does not check for or reconcile a graph that was saved into `scene.usd` under a *different* path or name.

Test from a debug shell (`source /opt/ros/jazzy/setup.bash` first):
```bash
ros2 topic pub --once /joint_command sensor_msgs/msg/JointState "{name: [joint1], position: [0.5]}"
ros2 topic echo /joint_states --once
```

**Articulation root is not `/World/lite6`.** The Lite6 asset (`.../Robots/Ufactory/lite6/lite6.usd`, payload-referenced at `/World/lite6`) applies `UsdPhysics.ArticulationRootAPI` to `/World/lite6/root_joint` (the fixed joint welding the base to world), not to the `/World/lite6` Xform itself — confirmed by downloading and inspecting the asset's USD layers directly (`lite6.usd` + its `configuration/lite6_base.usd` sublayer) via `pxr` bindings borrowed from `/isaac-sim/extscache/omni.usd.libs-*`, since `/isaac-sim/python.sh` alone doesn't have `pxr` on its path. `isaacsim.core.nodes.IsaacArticulationController`'s `robotPath` goes through PhysX's tensor API, which matches the *exact* prim carrying the API, not an ancestor — pointing it at `/World/lite6` reproducibly fails with `Pattern '/World/lite6' did not match any articulations`. Both `ArticulationController.inputs:robotPath` and `PublishJointState.inputs:targetPrim` in `ros2_control_graph.py` therefore use `/World/lite6/root_joint`. Joint names are `joint1`..`joint6` (revolute, all currently driven around the Z axis of their own frame).

**Velocity and effort commands don't produce real motion with the asset's default drive gains — this is expected, not a bug in the graph wiring.** Each joint's `UsdPhysics.DriveAPI` (angular) has both stiffness and damping baked in as imported (e.g. joint1: stiffness ≈44, damping ≈0.018), i.e. a PD *position* drive. `IsaacArticulationController` writes whatever command type it's given into that same drive without changing gains, so a velocity or effort target gets fought and largely cancelled by the existing position-holding stiffness — confirmed empirically: a `position` command on joint1 (0.5 rad) reliably reached target, but a sustained `velocity` command on joint2 (0.3 rad/s) and an `effort` command on joint3 (5.0) each produced no measurable motion over 2+ seconds. Position control is thus the only mode that actually works today; this is by explicit user choice, not an oversight (the alternative would require dynamically zeroing stiffness/damping per joint based on which command field is populated, which was considered and deliberately deferred — see git history if you need to revisit this).

## Host prerequisites (not handled by the Dockerfile)

The NVIDIA Container Toolkit must be installed on the host for `--gpus all` to work. It requires `sudo`, so it's not scripted here:
```bash
sudo apt-get install -y ca-certificates curl gnupg2
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

## Architecture notes

- **Single combined container, not split.** A split (Isaac Sim in one container, ROS 2 app graph in another) was considered but rejected as the default: the bridge's in-process design means ROS 2 client libraries are needed inside the Isaac Sim container either way, and cross-container DDS discovery on Linux needs `--network=host` on both sides regardless. A split remains reasonable later for keeping app-specific ROS packages out of the (large, slow-to-build) Isaac Sim image, but isn't set up here.
- **Runtime user is UID 1234, not the host user.** `/isaac-sim` in the base image is owned by `isaac-sim:isaac-sim` (UID 1234) mode `750` — only that UID can access it at all, so the container must run as `-u 1234:1234` (`run.sh` does this). The host-side cache directories `run.sh` bind-mounts are created as your host user instead, so they need opening up for UID 1234 to write into: `find ... -user "$(id -un)" -exec chmod o+rwX {} +`, scoped to files you actually own (files the container itself already wrote, owned by 1234, are skipped — you can't `chmod` files you don't own, and don't need to).
- **Stage autoload (`autoload_stage.py`) uses `kit`'s `--exec` flag, not a CLI stage-path argument.** Kit-based apps (confirmed via `kit --help` and by tracing the actual process argv in a running container) have no supported way to open a specific USD stage by passing its path as a bare positional CLI argument — an extra positional after `[APP_CONFIG]` is silently ignored (verified empirically: no log output, no error, stage unchanged). `--exec SCRIPT` *is* a real, documented flag ("execute a console command on startup"); it runs *after* the app finishes loading (extensions started, default blank stage already created) and after `isaacsim.app.setup`'s own `create_new_stage` step, confirmed via log timestamps in testing, so `autoload_stage.py`'s `open_stage()` call cleanly replaces the blank stage with no race. `runheadless.sh` forwards all trailing args through to `kit` unmodified, so `CMD ["./runheadless.sh", "--exec", "/isaac-sim/autoload_stage.py"]` reaches it intact. The script itself checks whether `/isaac-sim/content/scene.usd` (bind-mounted from `isaac-sim/content` in the repo — see `run.sh`) exists before opening it, so nothing changes when no scene has been saved yet.
- **Known-bad configuration: don't pre-set `ROS_DISTRO` or source `/opt/ros/jazzy/setup.bash` before launching Isaac Sim.** Doing so (originally via an image-wide `ENV ROS_DISTRO=jazzy` plus a wrapper script sourcing ROS before `exec`ing `runheadless.sh`) caused `isaacsim.ros2.core` to fail at startup with `librcutils.so: cannot open shared object file`, because it suppressed Isaac Sim's own bundled-library auto-setup without successfully substituting for it. Fixed by not setting `ROS_DISTRO` globally and launching `./runheadless.sh` directly as `CMD`, letting `setup_ros_env.sh` do its own thing.
