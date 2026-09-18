#!/usr/bin/env bash
# Robotics (02000537) — Lab 0 container: build it, start it, get a shell in it.
#
#   ./run.sh                  a shell in the lab container (starts it if needed)
#   ./run.sh build            build the image (the 45–90 minute part); rebuild
#                             does the same with --no-cache
#   ./run.sh check            run the environment self-test
#   ./run.sh start            launch the Lite 6 stack straight away
#   ./run.sh stop             stop the container, keep its filesystem
#   ./run.sh restart          throw it away and make a fresh one
#   ./run.sh clean            remove the container (image and dev_ws untouched)
#   ./run.sh status           what is built, what is running, with what settings
#   ./run.sh -- <command>     run one command in the container and exit
#
# Options, all of which apply to the shell you type them in:
#   --gazebo / --no-gazebo    whether anything may launch Gazebo
#   --domain-id N             your assigned ROS_DOMAIN_ID
#   --robot-ip ADDR           the physical arm, for `lab-start real`
#   --gpu / --no-gpu          override GPU autodetection
#   --software-gl             LIBGL_ALWAYS_SOFTWARE=1, for a black Gazebo window
#
# Settings resolve flag > lab0/.env > default, so put the ones you always want
# in .env and only type the exceptions.
#
# Every invocation joins the *same* container, so "open a new terminal for each
# test" just means running ./run.sh again in a new terminal.
set -euo pipefail

LAB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="${LAB_IMAGE:-robotica-lab:jazzy}"
CONTAINER="${LAB_CONTAINER:-robotica-lab}"
XAUTH="/tmp/.${CONTAINER}.xauth"

# Personal settings — domain id, robot ip — live here so they are not typed
# every time and not committed. See lab0/.env.example.
# shellcheck disable=SC1091
[ -f "$LAB_DIR/.env" ] && source "$LAB_DIR/.env"

# Defaults; empty means "not given on the command line", which matters when the
# container is already running and we only want to override what was asked for.
cmd=""
arg_gazebo=""
arg_domain=""
arg_robot_ip=""
arg_gpu=""
arg_softgl=""
passthrough=()
npass=0

die() { echo "run.sh: $*" >&2; exit 1; }

usage() { sed -n '2,26p' "$0" | sed 's/^# \{0,1\}//'; }

while [ $# -gt 0 ]; do
    case "$1" in
        build|rebuild|shell|start|check|stop|restart|clean|status) cmd="$1";;
        --gazebo)      arg_gazebo=1;;
        --no-gazebo)   arg_gazebo=0;;
        --domain-id)   [ $# -ge 2 ] || die "--domain-id needs a number"; arg_domain="$2"; shift;;
        --robot-ip)    [ $# -ge 2 ] || die "--robot-ip needs an address"; arg_robot_ip="$2"; shift;;
        --gpu)         arg_gpu=1;;
        --no-gpu)      arg_gpu=0;;
        --software-gl) arg_softgl=1;;
        -h|--help)     usage; exit 0;;
        --)            shift; passthrough=("$@"); npass=$#; break;;
        *)             die "unknown option '$1' (try --help)";;
    esac
    shift
done

[ -z "$cmd" ] && [ "$npass" -gt 0 ] && cmd="exec"
[ -z "$cmd" ] && cmd="shell"

ENABLE_GAZEBO="${arg_gazebo:-${ENABLE_GAZEBO:-1}}"
ROS_DOMAIN_ID="${arg_domain:-${ROS_DOMAIN_ID:-0}}"
ROBOT_IP="${arg_robot_ip:-${ROBOT_IP:-}}"
LIBGL_ALWAYS_SOFTWARE="${arg_softgl:-${LIBGL_ALWAYS_SOFTWARE:-0}}"
COLCON_JOBS="${COLCON_JOBS:-2}"

# --------------------------------------------------------------------------
# image
# --------------------------------------------------------------------------
image_exists() { docker image inspect "$IMAGE" >/dev/null 2>&1; }

build_image() {
    local extra=()
    [ "${1:-}" = "nocache" ] && extra+=(--no-cache)
    echo "Building $IMAGE — this takes 45–90 minutes the first time."
    echo "(ROS 2 desktop + MoveIt + Gazebo download, then xarm_ros2 compiles.)"
    docker build "${extra[@]}" \
        --build-arg "USER_UID=$(id -u)" \
        --build-arg "USER_GID=$(id -g)" \
        --build-arg "COLCON_JOBS=${COLCON_JOBS}" \
        -t "$IMAGE" "$LAB_DIR"
}

# --------------------------------------------------------------------------
# display
#
# RViz and Gazebo need an X server. Rather than opening the host's X server to
# everything (xhost +), copy the current display cookie into a wildcard-host
# entry the container can use, and fall back to xhost only if that is not
# possible.
# --------------------------------------------------------------------------
setup_display() {
    [ -z "${DISPLAY:-}" ] && { echo "run.sh: DISPLAY is unset — RViz/Gazebo will not open a window."; return; }
    if command -v xauth >/dev/null 2>&1; then
        # Truncated in place rather than recreated: a running container has this
        # file bind-mounted, and a new inode would not reach it.
        : > "$XAUTH"
        # On a GNOME/GDM desktop — which is what Ubuntu 24.04 gives you — the
        # cookie for the session display is not in ~/.Xauthority at all, it is
        # in GDM's own file, so look in all the usual places. The sed rewrites
        # each entry's address family to FamilyWild (ffff) so that it matches
        # whatever hostname the container presents.
        local src
        for src in "${XAUTHORITY:-}" "$HOME/.Xauthority" "/run/user/$(id -u)/gdm/Xauthority"; do
            { [ -n "$src" ] && [ -r "$src" ]; } || continue
            XAUTHORITY="$src" xauth nlist "$DISPLAY" 2>/dev/null \
                | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge - 2>/dev/null || true
            [ -s "$XAUTH" ] && break
        done
        chmod 600 "$XAUTH"
    fi
    if [ ! -s "$XAUTH" ] && command -v xhost >/dev/null 2>&1; then
        echo "run.sh: no X cookie found for $DISPLAY, falling back to 'xhost +local:',"
        echo "        which lets any local user talk to your X server. Undo with: xhost -local:"
        xhost +local: >/dev/null 2>&1 || true
    fi
}

# --------------------------------------------------------------------------
# container
# --------------------------------------------------------------------------
container_state() {
    local st
    st="$(docker inspect -f '{{.State.Status}}' "$CONTAINER" 2>/dev/null || true)"
    echo "${st:-missing}"
}

gpu_args() {
    local want="${arg_gpu:-auto}"
    [ "$want" = "0" ] && return 0
    if [ "$want" = "1" ] || { command -v nvidia-smi >/dev/null 2>&1 && docker info 2>/dev/null | grep -qi nvidia; }; then
        echo "--gpus all -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all,graphics"
    fi
}

create_container() {
    setup_display
    mkdir -p "$LAB_DIR/dev_ws/src" "$LAB_DIR/.home"

    local args=(
        --name "$CONTAINER" --detach
        # host networking: DDS discovery with the lab machines and the real arm
        # on 192.168.1.x both go through it. This is also why ROS_DOMAIN_ID is
        # not optional on the lab LAN.
        --network=host
        # shared memory for Qt/RViz; avoids the classic MIT-SHM crash
        --ipc=host
        -e "DISPLAY=${DISPLAY:-}"
        -e "ENABLE_GAZEBO=${ENABLE_GAZEBO}"
        -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
        -e "ROBOT_IP=${ROBOT_IP}"
        -e "LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE}"
        -e "COLCON_JOBS=${COLCON_JOBS}"
        -v "$LAB_DIR/dev_ws:/home/ros/dev_ws"
        -v "$LAB_DIR/.home:/home/ros/.persist"
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw
    )
    [ -s "$XAUTH" ] && args+=(-v "$XAUTH:$XAUTH:rw" -e "XAUTHORITY=$XAUTH")
    [ -d /dev/dri ] && args+=(--device /dev/dri)
    # WSL2 (path B): WSLg provides the display and GL through this mount.
    [ -d /run/WSLg ] && args+=(-v /mnt/wslg:/mnt/wslg -e "WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-}" -e "PULSE_SERVER=${PULSE_SERVER:-}")
    # shellcheck disable=SC2046
    args+=($(gpu_args))

    docker run "${args[@]}" "$IMAGE" sleep infinity >/dev/null
    echo "Started container '$CONTAINER'."
}

# Each shell gets the currently resolved ROS_DOMAIN_ID, but a launch already
# running in another terminal kept whatever that terminal was given. Editing
# .env mid-session therefore splits the ROS graph in two, and the symptom is
# silence: an empty view_frames PDF, a listener that hears nothing. Say so.
warn_domain_drift() {
    local created
    created="$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$CONTAINER" 2>/dev/null \
                 | sed -n 's/^ROS_DOMAIN_ID=//p' | head -1)"
    if [ -n "$created" ] && [ "$created" != "$ROS_DOMAIN_ID" ]; then
        echo "run.sh: this shell is on ROS_DOMAIN_ID=$ROS_DOMAIN_ID, but the container was" >&2
        echo "        started with $created — anything already running in another terminal" >&2
        echo "        is still on $created and will not see this shell's nodes." >&2
        echo "        Restart those launches, or './run.sh restart' for a clean slate." >&2
    fi
    return 0
}

# A container keeps the filesystem of the image it was created from, so after
# a rebuild the old one is still running the old helpers and the old packages —
# which looks like the rebuild silently did nothing.
warn_stale_image() {
    local cimg iimg
    cimg="$(docker inspect -f '{{.Image}}' "$CONTAINER" 2>/dev/null || true)"
    iimg="$(docker image inspect -f '{{.Id}}' "$IMAGE" 2>/dev/null || true)"
    if [ -n "$cimg" ] && [ -n "$iimg" ] && [ "$cimg" != "$iimg" ]; then
        echo "run.sh: this container was created from an older build of $IMAGE, so" >&2
        echo "        anything added since (helpers, apt packages) is missing from it." >&2
        echo "        Pick the new image up with:  $0 restart" >&2
    fi
    return 0
}

ensure_running() {
    image_exists || { echo "Image $IMAGE not found."; build_image; }
    case "$(container_state)" in
        running) setup_display; warn_stale_image; warn_domain_drift;;
        exited|created) setup_display; docker start "$CONTAINER" >/dev/null; echo "Resumed container '$CONTAINER'.";;
        *) create_container;;
    esac
}

# Every shell is given the settings resolved above — command-line flag beats
# .env beats the built-in default — rather than whatever the container happened
# to be created with. Otherwise editing .env would do nothing until the next
# `restart`, and a wrong ROS_DOMAIN_ID is exactly the kind of thing that shows
# up later as "Test 1 mysteriously fails". Flags stay per-shell: the other
# terminals keep what they were given.
EXEC_ENV=()
build_exec_env() {
    EXEC_ENV=(
        -e "DISPLAY=${DISPLAY:-}"
        -e "ENABLE_GAZEBO=${ENABLE_GAZEBO}"
        -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
        -e "ROBOT_IP=${ROBOT_IP}"
        -e "LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE}"
    )
    [ -s "$XAUTH" ] && EXEC_ENV+=(-e "XAUTHORITY=$XAUTH")
    return 0
}

in_container() {
    build_exec_env
    # -t only when there is a terminal on both ends: `docker exec -it` fails
    # outright under a pipe or a script ("the input device is not a TTY").
    local tty=(-i)
    [ -t 0 ] && [ -t 1 ] && tty=(-i -t)
    docker exec "${tty[@]}" "${EXEC_ENV[@]}" -w /home/ros/dev_ws "$CONTAINER" "$@"
}

case "$cmd" in
    build)   build_image;;
    rebuild) build_image nocache;;
    stop)    docker stop "$CONTAINER" >/dev/null 2>&1 && echo "Stopped (filesystem kept — ./run.sh to resume)." || echo "Not running.";;
    clean)   docker rm -f "$CONTAINER" >/dev/null 2>&1 && echo "Container removed. The image and ./dev_ws are untouched." || echo "Nothing to remove.";;
    restart) docker rm -f "$CONTAINER" >/dev/null 2>&1 || true; ensure_running;;
    status)
        echo "image     $IMAGE   $(image_exists && echo built || echo "NOT BUILT — run '$0 build'")"
        echo "container $CONTAINER   $(container_state)"
        echo "workspace $LAB_DIR/dev_ws"
        echo "gazebo    $([ "$ENABLE_GAZEBO" = "0" ] && echo disabled || echo enabled)   domain id $ROS_DOMAIN_ID   display ${DISPLAY:-<unset>}"
        [ -n "$ROBOT_IP" ] && echo "robot     $ROBOT_IP"
        true
        ;;
    shell)   ensure_running; in_container bash;;
    check)   ensure_running; in_container lab-check;;
    start)   ensure_running; in_container lab-start;;
    exec)    ensure_running; in_container bash -lc 'exec "$@"' run.sh "${passthrough[@]}";;
esac
