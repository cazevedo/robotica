# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A Docker setup combining NVIDIA Isaac Sim with ROS 2 Jazzy in a single image.

- Base image: `nvcr.io/nvidia/isaac-sim:6.0.1` (Ubuntu 24.04 / Noble), which is what makes installing ROS 2 Jazzy (Noble-targeted) directly on top possible.
- ROS 2 package set is `ros-jazzy-ros-base` (headless, no rviz/GUI tools) plus `ros-jazzy-vision-msgs` and `ros-jazzy-ackermann-msgs`, which Isaac Sim's ROS 2 bridge extension references.
- Display mode is WebRTC streaming (`./runheadless.sh`), not X11 passthrough — chosen so the container also works headlessly/remotely later.

**The apt-installed ROS 2 Jazzy and Isaac Sim's ROS 2 bridge are deliberately kept separate, not wired together.** Isaac Sim's `isaacsim.ros2.bridge` extension dynamically loads ROS 2 client/RMW libraries *into the Isaac Sim process itself* (it's not a separate ROS node), and it ships its own complete, self-contained copy of the Jazzy libraries at `/isaac-sim/exts/isaacsim.ros2.core/jazzy/lib` (344 `.so` files, message packages included) for exactly this purpose. Isaac Sim's own launch chain (`runheadless.sh` -> `isaac-sim.streaming.sh` -> `setup_ros_env.sh`) auto-detects Ubuntu 24.04 and wires the bridge up to that bundled copy automatically — but **only if `$ROS_DISTRO` is unset at that point**; if it's already set (e.g. by an image-wide `ENV ROS_DISTRO=jazzy`, or by sourcing `/opt/ros/jazzy/setup.bash` before launch), `setup_ros_env.sh` assumes the caller already configured `LD_LIBRARY_PATH` correctly and skips its own setup — and the caller's env did not survive into the actual `kit` process in testing, so the bridge failed with `librcutils.so: cannot open shared object file`. See "Known-bad configuration" below.

The apt-installed `/opt/ros/jazzy` is for *your own* ROS 2 usage — CLI tools, `colcon` workspaces, custom nodes — used interactively (`docker exec` / `--entrypoint bash`, where `/etc/bash.bashrc` sources it automatically). It talks to the bridge over DDS like any other ROS 2 node would, not by sharing libraries with it.

## Commands

Build and run:
```bash
./run.sh
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

A browser-based client (e.g. a URL like `http://127.0.0.1:8211/streaming/webrtc-client/`) does exist, but it's served by a *separate* NVIDIA Docker Compose "web viewer" bundle with its own proxy/web-server container and `ISAACSIM_SIGNAL_PORT` / `ISAACSIM_STREAM_PORT` / `WEB_VIEWER_PORT` env vars — not something the plain image serves on its own. Not set up here; the desktop client above is simpler and already works with this setup.

Persist an environment/robot so it auto-loads next time: in the Isaac Sim GUI (via the WebRTC client), **File > Save As** to `/isaac-sim/content/scene.usd` — that path is bind-mounted from `./content` (next to this Dockerfile, in the repo checkout) on the host (see `run.sh`), so it survives the container being removed (it runs with `--rm`; nothing saved outside a mounted path persists). The next `./run.sh` opens that exact file automatically instead of the default blank stage — see "Stage autoload" below. The filename must be exactly `scene.usd`; that's what `autoload_stage.py` looks for. To go back to a blank stage, remove/rename `./content/scene.usd`.

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
- **Stage autoload (`autoload_stage.py`) uses `kit`'s `--exec` flag, not a CLI stage-path argument.** Kit-based apps (confirmed via `kit --help` and by tracing the actual process argv in a running container) have no supported way to open a specific USD stage by passing its path as a bare positional CLI argument — an extra positional after `[APP_CONFIG]` is silently ignored (verified empirically: no log output, no error, stage unchanged). `--exec SCRIPT` *is* a real, documented flag ("execute a console command on startup"); it runs *after* the app finishes loading (extensions started, default blank stage already created) and after `isaacsim.app.setup`'s own `create_new_stage` step, confirmed via log timestamps in testing, so `autoload_stage.py`'s `open_stage()` call cleanly replaces the blank stage with no race. `runheadless.sh` forwards all trailing args through to `kit` unmodified, so `CMD ["./runheadless.sh", "--exec", "/isaac-sim/autoload_stage.py"]` reaches it intact. The script itself checks whether `/isaac-sim/content/scene.usd` (bind-mounted from `./content` in the repo — see `run.sh`) exists before opening it, so nothing changes when no scene has been saved yet.
- **Known-bad configuration: don't pre-set `ROS_DISTRO` or source `/opt/ros/jazzy/setup.bash` before launching Isaac Sim.** Doing so (originally via an image-wide `ENV ROS_DISTRO=jazzy` plus a wrapper script sourcing ROS before `exec`ing `runheadless.sh`) caused `isaacsim.ros2.core` to fail at startup with `librcutils.so: cannot open shared object file`, because it suppressed Isaac Sim's own bundled-library auto-setup without successfully substituting for it. Fixed by not setting `ROS_DISTRO` globally and launching `./runheadless.sh` directly as `CMD`, letting `setup_ros_env.sh` do its own thing.
