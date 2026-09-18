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

dc() { echo "${B}\$${R} docker compose $*"; docker compose "$@"; }

running() {
    local id; id="$(docker compose ps -q "${SERVICE}" 2>/dev/null)"
    [ -n "${id}" ] && [ "$(docker inspect -f '{{.State.Running}}' "${id}" 2>/dev/null)" = "true" ]
}
ensure_up() { running || { say "starting container"; dc up -d >/dev/null; sleep 1; }; }

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
