# Lab 0, Step 7 — shell configuration for the Robotics course container.
#
# Sourced from ~/.bashrc (interactive shells) and /etc/profile.d/ros-lab.sh
# (login shells). The guard below keeps the second one from prepending every
# ROS path a second time.
[ -n "${LAB_ENV_SOURCED:-}" ] && return 0
LAB_ENV_SOURCED=1

# --- Step 7, line 1: ROS 2 itself ------------------------------------------
source /opt/ros/jazzy/setup.bash

# --- Step 8: the xarm_ros2 workspace ---------------------------------------
# Built into the image (see the Dockerfile), not into ~/dev_ws, so that
# ~/dev_ws stays yours. Overlays are sourced underlay-first.
[ -f /opt/xarm_ws/install/setup.bash ] && source /opt/xarm_ws/install/setup.bash

# --- Your workspace, if you have built anything in it ----------------------
[ -f "$HOME/dev_ws/install/setup.bash" ] && source "$HOME/dev_ws/install/setup.bash"

# --- Step 7, line 2: your assigned domain id -------------------------------
# run.sh passes this in; the default only applies if it did not.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# Shell history survives container restarts by living on the bind mount.
if [ -d "$HOME/.persist" ]; then
    export HISTFILE="$HOME/.persist/bash_history"
    export HISTSIZE=10000
    export HISTFILESIZE=20000
    shopt -s histappend 2>/dev/null || true
fi

export QT_X11_NO_MITSHM=1
[ -d "$XDG_RUNTIME_DIR" ] || { mkdir -p "$XDG_RUNTIME_DIR" 2>/dev/null && chmod 700 "$XDG_RUNTIME_DIR" 2>/dev/null; } || true

# Interactive shells get a one-screen reminder of the state that actually
# changes between sessions: domain id, gazebo, display, robot.
if [[ $- == *i* ]]; then
    _lab_gz="on"
    [ "${ENABLE_GAZEBO:-1}" = "0" ] && _lab_gz="OFF (--no-gazebo)"
    printf '\n  Robotics lab — ROS 2 %s\n' "$ROS_DISTRO"
    printf '    ROS_DOMAIN_ID = %s        gazebo = %s\n' "$ROS_DOMAIN_ID" "$_lab_gz"
    printf '    DISPLAY       = %s\n' "${DISPLAY:-<unset — no GUI>}"
    [ -n "${ROBOT_IP:-}" ] && printf '    ROBOT_IP      = %s\n' "$ROBOT_IP"
    printf '    workspace     = ~/dev_ws  (saved on the host)\n'
    printf '    lab-check     run the environment self-test\n'
    printf '    lab-start     launch the Lite 6 (gazebo | fake | real)\n'
    printf '    lab-build     colcon build ~/dev_ws\n\n'
    unset _lab_gz
fi
