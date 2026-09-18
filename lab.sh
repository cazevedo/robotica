#!/usr/bin/env bash
###############################################################################
# lab.sh - the same commands as lab.ps1, for Git Bash / WSL / Linux / macOS.
#   ./lab.sh build | up | shell | down | gazebo on|off | doctor | help
###############################################################################
set -o pipefail
cd "$(dirname "$0")" || exit 1

SERVICE=lab

if [ -t 1 ]; then B=$'\033[1m'; R=$'\033[0m'; E=$'\033[31m'; else B=''; R=''; E=''; fi
say() { echo "${B}==>${R} $*"; }
die() { echo "${E}!!${R} $*" >&2; exit 1; }

command -v docker >/dev/null 2>&1 || die "docker is not installed or not on PATH"
docker compose version >/dev/null 2>&1 || die "'docker compose' is not available"
[ -f .env ] || { cp .env.example .env; say ".env created from .env.example - set ROS_DOMAIN_ID in it"; }

###############################################################################
# Display and GPU, the Linux way.
#
# RViz and Gazebo open as ordinary windows on your desktop: the container talks
# to the X server you are already running, through the socket in /tmp/.X11-unix
# and a copy of your auth cookie. No VNC, no WSLg, and the GPU you actually
# have does the rendering.
###############################################################################

# Where the cookie lives on the host, and where the container will look for it.
LAB_XAUTH_HOST="${LAB_XAUTH_HOST:-/tmp/.lab-robotica.xauth}"
LAB_XAUTH_CONTAINER=/tmp/.lab.xauth
export LAB_XAUTH_HOST LAB_XAUTH_CONTAINER

setup_xauth() {
    [ -z "${DISPLAY:-}" ] && return 0
    command -v xauth >/dev/null 2>&1 || {
        say "xauth is not installed (apt install xauth) - falling back to VNC"
        return 1
    }
    # Truncated, not recreated: a running container has this bind-mounted, and
    # a new inode would never reach it.
    : > "${LAB_XAUTH_HOST}" 2>/dev/null || return 1

    # On a GNOME/GDM desktop - i.e. stock Ubuntu - the cookie for your session
    # is not in ~/.Xauthority at all, it is in GDM's own file. Look in all the
    # usual places. The sed rewrites each entry's address family to FamilyWild
    # (ffff) so it matches whatever hostname the container presents.
    local src
    for src in "${XAUTHORITY:-}" "${HOME}/.Xauthority" "/run/user/$(id -u)/gdm/Xauthority"; do
        { [ -n "${src}" ] && [ -r "${src}" ]; } || continue
        XAUTHORITY="${src}" xauth nlist "${DISPLAY}" 2>/dev/null \
            | sed -e 's/^..../ffff/' | xauth -f "${LAB_XAUTH_HOST}" nmerge - 2>/dev/null || true
        [ -s "${LAB_XAUTH_HOST}" ] && break
    done

    if [ ! -s "${LAB_XAUTH_HOST}" ]; then
        say "no X cookie found for ${DISPLAY}; trying 'xhost +local:'"
        say "  (that lets any local user reach your X server; undo: xhost -local:)"
        command -v xhost >/dev/null 2>&1 && xhost +local: >/dev/null 2>&1 || true
        # Give the mount something to point at even so.
        : > "${LAB_XAUTH_HOST}"
    fi
    chmod 600 "${LAB_XAUTH_HOST}" 2>/dev/null || true
    return 0
}

# Extra compose files, chosen from what this machine actually has. Kept out of
# docker-compose.yml because compose fails outright on a device node that does
# not exist.
compose_files() {
    local files=(-f docker-compose.yml)
    if [ -d /dev/dri ]; then
        files+=(-f docker-compose.gpu.yml)
        # The nvidia stanza needs the container toolkit; without it compose
        # dies with "could not select device driver nvidia".
        if command -v nvidia-smi >/dev/null 2>&1 && docker info 2>/dev/null | grep -qi nvidia; then
            files+=(-f docker-compose.nvidia.yml)
        fi
    fi
    printf '%s\n' "${files[@]}"
}

COMPOSE_FILES=()
while IFS= read -r line; do COMPOSE_FILES+=("${line}"); done < <(compose_files)

dc() {
    echo "${B}\$${R} docker compose $*"
    docker compose "${COMPOSE_FILES[@]}" "$@"
}

running() {
    local id; id="$(docker compose "${COMPOSE_FILES[@]}" ps -q "${SERVICE}" 2>/dev/null)"
    [ -n "${id}" ] && [ "$(docker inspect -f '{{.State.Running}}' "${id}" 2>/dev/null)" = "true" ]
}
ensure_up() { setup_xauth; running || { say "starting container"; dc up -d >/dev/null; sleep 1; }; }

set_env() {  # set_env KEY VALUE
    if grep -qE "^\s*$1\s*=" .env; then
        sed -i.bak -E "s|^\s*$1\s*=.*|$1=$2|" .env && rm -f .env.bak
    else
        echo "$1=$2" >> .env
    fi
}

case "${1:-help}" in
    build)
        say "building robotics-lab0:jazzy - 45-90 minutes, mostly downloads"
        shift; dc --progress plain build "$@" ;;
    rebuild)        shift; dc --progress plain build "$@" ;;
    rebuild-xarm)   dc --progress plain build --build-arg "XARM_CACHEBUST=$(date +%Y-%m-%d-%H%M%S)" ;;
    up|start)       dc up -d && say "running. Open a terminal:  ./lab.sh shell" ;;
    down|stop)      dc down ;;
    restart)        dc down; dc up -d ;;
    shell|sh|bash)  ensure_up; docker compose exec "${SERVICE}" bash ;;
    run|exec)       shift; ensure_up; docker compose exec "${SERVICE}" "$@" ;;
    doctor|check)   ensure_up; docker compose exec "${SERVICE}" lab doctor ;;
    status|ps)      dc ps; echo; docker volume ls --filter name=robotics-lab0 ;;
    logs)           shift; dc logs --tail 200 "$@" ;;
    gazebo)
        case "${2:-status}" in
            on|1)  set_env LAB_GAZEBO 1; running && docker compose exec "${SERVICE}" lab gazebo on  || say "enabled in .env" ;;
            off|0) set_env LAB_GAZEBO 0; running && docker compose exec "${SERVICE}" lab gazebo off || say "disabled in .env" ;;
            *)     grep -E '^LAB_GAZEBO=' .env; running && docker compose exec "${SERVICE}" lab gazebo status || true ;;
        esac ;;
    nuke)
        echo "This deletes the container and its volumes (build artefacts, caches)."
        echo "./src and ./shared are NOT touched."
        read -r -p "Type 'yes' to continue: " a
        [ "${a}" = "yes" ] && dc down -v || echo "cancelled" ;;
    help|-h|--help)
        cat <<'HELP'
./lab.sh - Lab 0 container (UFACTORY Lite 6 / ROS 2 Jazzy / Gazebo Harmonic)

  build            build the image (once, 45-90 min)
  up               start the container
  shell            open a terminal inside it  (repeat per "new terminal")
  run <cmd...>     run one command inside, e.g.  ./lab.sh run lab test 2
  doctor           check the environment
  gazebo on|off    allow / block Gazebo - takes effect immediately
  down             stop it (your files are kept)
  restart          stop, start, re-read .env
  status | logs    what is running / its output
  nuke             delete container + volumes (./src and ./shared survive)

  ./src    -> ~/dev_ws/src   your packages: edit on the host, build inside
  ./shared -> ~/shared       files in and out (the view_frames PDF, notes)
HELP
        ;;
    *) die "unknown command: $1  (try: ./lab.sh help)" ;;
esac
