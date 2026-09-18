# Robotics (02000537) — Lab 0 development environment.
#
# Implements Lab 0 Part 1 (Steps 1–8) as an image: Ubuntu 24.04 + ROS 2 Jazzy
# desktop + MoveIt 2 + Gazebo Harmonic + a pre-built xarm_ros2 workspace for
# the UFACTORY Lite 6.
#
# This is "Path C" of the handout. Don't confuse it with the Isaac Sim image in
# ../isaac-sim — different base, different purpose, no shared layers.
#
# Build via lab0/run.sh build (it passes the arguments below for you).
FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive

# Your host UID/GID, so that everything the container writes into the
# bind-mounted workspace comes out owned by you on the host rather than by root.
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

# colcon parallelism for the xarm_ros2 build below. 2 is the handout's
# recommendation for laptops; raise it (--build-arg COLCON_JOBS=8) if you have
# the RAM — roughly 2 GB per worker.
ARG COLCON_JOBS=2

# Pinned fallback, only used if the GitHub API call in Step 3 fails (rate limit
# or no network). The API lookup is preferred because upstream rotates this.
ARG ROS_APT_SOURCE_FALLBACK=1.3.0

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# ---------------------------------------------------------------------------
# Step 1 — locale
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# ---------------------------------------------------------------------------
# Steps 2 & 3 — universe repository, then the ROS 2 apt repository
#
# The handout points at the official "Set up sources" page because upstream
# changes this from time to time. The current official method is the
# ros2-apt-source .deb, which carries the key and the source list; that is what
# is done here, so there is no apt-key/gpg handling to go stale.
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        software-properties-common curl ca-certificates gnupg \
    && add-apt-repository universe \
    && ROS_APT_SOURCE_VERSION="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
         | grep -F '"tag_name"' | awk -F\" '{print $4}')" \
    && ROS_APT_SOURCE_VERSION="${ROS_APT_SOURCE_VERSION:-${ROS_APT_SOURCE_FALLBACK}}" \
    && echo "Using ros-apt-source ${ROS_APT_SOURCE_VERSION}" \
    && curl -fsSL -o /tmp/ros2-apt-source.deb \
         "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb" \
    && apt-get install -y /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb \
    && apt-get update \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=jazzy

# ---------------------------------------------------------------------------
# Step 4 — ROS 2 Jazzy desktop + the build tools
#
#   ros-jazzy-desktop — ROS 2 core, RViz2, rqt, tf2 tools, demo nodes
#   ros-dev-tools     — colcon, rosdep, vcstool and friends
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-desktop \
        ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Step 5 — MoveIt 2
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-moveit \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Step 6 — Gazebo Harmonic (NOT Gazebo Classic — EOL since January 2025)
#
#   ros-jazzy-ros-gz          — Gazebo Harmonic + the ROS <-> Gazebo bridge
#   ros-jazzy-gz-ros2-control — lets ros2_control drive a robot inside Gazebo
#
# Gazebo is always installed; whether it is *launched* is a runtime decision —
# see ENABLE_GAZEBO / lab-start below. An idle, un-launched Gazebo costs disk,
# not CPU. (Skipping the install would also break the xarm_gazebo package in
# Step 8, which is why it is not a build-time option.)
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz \
        ros-jazzy-gz-ros2-control \
    && rm -rf /var/lib/apt/lists/*

# Extras the handout's tests need but that no ROS package pulls in:
#   graphviz      — tf2_tools view_frames renders the frames PDF with `dot`
#   mesa/x11-apps — GL for RViz/Gazebo, software fallback, and `xeyes` to tell
#                   a broken display apart from a broken RViz
#   xvfb          — a fake display, so Gazebo and RViz can run with no screen
#                   at all (over plain ssh, say): xvfb-run -a lab-start gazebo
#   ros2controlcli — `ros2 control list_controllers`, which is how you find out
#                   why the arm moves in RViz but not in Gazebo (Test 7)
#   iputils-ping  — `ping <robot_ip>` before touching ROS (real-robot appendix)
# plus the ordinary things you need to actually write code in here.
RUN apt-get update && apt-get install -y --no-install-recommends \
        graphviz \
        mesa-utils libgl1-mesa-dri x11-utils x11-apps xauth xvfb \
        ros-jazzy-ros2controlcli \
        iputils-ping iproute2 net-tools \
        git vim nano tmux less tree htop \
        python3-pip python3-venv \
        sudo bash-completion \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Non-root user, mapped onto your host account.
#
# Ubuntu 24.04 ships a stock "ubuntu" user already sitting on UID 1000, which
# is the UID most host accounts have, so it has to go before we can reuse it.
# ---------------------------------------------------------------------------
RUN set -eux; \
    if getent passwd "${USER_UID}" >/dev/null; then \
        userdel -r "$(getent passwd "${USER_UID}" | cut -d: -f1)" 2>/dev/null || true; \
    fi; \
    if ! getent group "${USER_GID}" >/dev/null; then groupadd -g "${USER_GID}" "${USERNAME}"; fi; \
    useradd -m -u "${USER_UID}" -g "${USER_GID}" -s /bin/bash "${USERNAME}"; \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}"; \
    chmod 0440 "/etc/sudoers.d/${USERNAME}"

# rosdep's system-wide init (Step 4). `rosdep update` is per-user and is run
# below as ${USERNAME}, so that your own `rosdep install` calls in ~/dev_ws work
# without a warning about a missing cache.
RUN rosdep init

# ---------------------------------------------------------------------------
# Step 8 — xarm_ros2, cloned and built once, here, at image build time.
#
# The handout builds this into ~/dev_ws on your machine. In a container that
# would land inside the image anyway and be shadowed by the bind mount, so it
# lives in /opt/xarm_ws instead and ~/dev_ws is left free for *your* packages
# (it is bind-mounted from the host and overlays this one — see /etc/lab-env.sh).
#
# --recursive is not optional: the C++ SDK that talks to the robot is a
# submodule and the build fails without it.
# ---------------------------------------------------------------------------
ENV XARM_WS=/opt/xarm_ws
RUN mkdir -p ${XARM_WS}/src \
    && git clone https://github.com/xArm-Developer/xarm_ros2.git --recursive -b jazzy ${XARM_WS}/src/xarm_ros2

RUN apt-get update \
    && rosdep update --rosdistro jazzy \
    && rosdep install --from-paths ${XARM_WS}/src --ignore-src --rosdistro jazzy -y \
    && rm -rf /var/lib/apt/lists/*

# The slow part: 15–30 minutes. Kept out of the layers above so that editing
# anything earlier does not force a rebuild of it.
RUN source /opt/ros/jazzy/setup.bash \
    && cd ${XARM_WS} \
    && colcon build \
        --parallel-workers ${COLCON_JOBS} \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf ${XARM_WS}/build ${XARM_WS}/log

RUN chown -R ${USER_UID}:${USER_GID} ${XARM_WS}

# ---------------------------------------------------------------------------
# Step 7 — shell configuration
#
# Everything lives in one guarded file so that login shells and interactive
# shells do not source the ROS setup twice and duplicate PATH entries.
# ROS_DOMAIN_ID keeps its runtime value if one was passed in (run.sh does),
# hence the :- default rather than a hard assignment.
# ---------------------------------------------------------------------------
COPY etc/lab-env.sh /etc/lab-env.sh
COPY bin/ /usr/local/bin/
# Launch file and RViz config for the MoveIt-free back ends (lab-start bare).
COPY share/ /opt/lab/
RUN chmod 0644 /etc/lab-env.sh /opt/lab/* \
    && chmod 0755 /usr/local/bin/lab-* \
    && echo 'source /etc/lab-env.sh' > /etc/profile.d/ros-lab.sh

USER ${USERNAME}
ENV HOME=/home/${USERNAME}

RUN printf '\n# --- Lab 0, Step 7 (ROS 2 + workspace overlays) ---\nsource /etc/lab-env.sh\n' >> ${HOME}/.bashrc \
    && rosdep update --rosdistro jazzy \
    && mkdir -p ${HOME}/dev_ws/src

# Silences the Qt/Gazebo "XDG_RUNTIME_DIR not set" noise on startup.
ENV XDG_RUNTIME_DIR=/tmp/runtime-ros

# Default: gazebo available. run.sh --no-gazebo overrides this per container or
# per shell; see lab-start.
ENV ENABLE_GAZEBO=1

WORKDIR ${HOME}/dev_ws

ENTRYPOINT ["/usr/local/bin/lab-entrypoint"]
CMD ["bash"]
