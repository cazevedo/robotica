#!/bin/bash
# Build and run the Isaac Sim + ROS 2 Jazzy container.
#
# Prerequisite (one-time, on the host, not handled by this script):
#   NVIDIA Container Toolkit must be installed so Docker can see the GPU.
#   See README notes / chat for the install commands.
set -e

# Anchor every relative path to this script's directory, so it can be run from
# anywhere (./isaac-sim/run.sh from the repository root, say).
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

IMAGE_NAME="isaac-sim-ros2-jazzy:latest"

docker build -t "$IMAGE_NAME" "$DIR"

mkdir -p ~/docker/isaac-sim/{cache/main,cache/computecache,logs,config,data,pkg} ~/.cache/ov/hub "$DIR/content" "$DIR/ros2_ws"

# The container's app files (/isaac-sim) are only accessible to UID 1234
# (owner isaac-sim:isaac-sim, mode 750), so the container must run as that
# UID. These host cache dirs are owned by the current host user instead, so
# open them up for "other" (UID 1234 falls into that class here). Only
# touch files we actually own — files/dirs the container already wrote
# (owned by UID 1234 from a previous run) don't need it, and find can't
# even traverse into some of them (permission denied), which is expected
# and harmless but would otherwise trip `set -e` via find's exit status.
find ~/docker/isaac-sim ~/.cache/ov/hub "$DIR/content" "$DIR/ros2_ws" -user "$(id -un)" -exec chmod o+rwX {} + 2>/dev/null || true

# Isaac Sim's WebRTC extension advertises this address to clients for the
# actual media session (separate from the initial signaling connection,
# which works over any reachable address, which is why a black screen on
# remote clients can coexist with a working local connection). Left unset,
# it auto-detects one of this host's interfaces to advertise, which with
# --network=host can be any of several (docker0, virbr0, ...) instead of
# the real LAN one. Auto-detect the LAN IP via the routing table instead;
# override by exporting ISAACSIM_HOST yourself if this picks the wrong one.
ISAACSIM_HOST="${ISAACSIM_HOST:-$(ip route get 1.1.1.1 2>/dev/null | grep -oP 'src \K\S+' || true)}"

docker run --name isaac-sim-ros \
  --rm -it \
  --gpus all \
  --network=host \
  -e "ACCEPT_EULA=Y" \
  -e "PRIVACY_CONSENT=Y" \
  -e "ISAACSIM_HOST=${ISAACSIM_HOST}" \
  -v ~/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw \
  -v ~/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw \
  -v ~/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw \
  -v ~/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw \
  -v "$DIR/content":/isaac-sim/content:rw \
  -v "$DIR/ros2_ws":/isaac-sim/ros2_ws:rw \
  -v ~/.cache/ov/hub:/var/cache/hub:rw \
  -u 1234:1234 \
  "$IMAGE_NAME"
