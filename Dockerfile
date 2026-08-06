# Isaac Sim 6.0.1 + ROS 2 Jazzy, single combined image.
# Base image is Ubuntu 24.04 (Noble), which is what makes installing ROS 2
# Jazzy directly on top possible (Jazzy targets Noble).
FROM nvcr.io/nvidia/isaac-sim:6.0.1

# The base image defaults to a non-root user (UID 1234); apt-get and writes
# to /etc need root during the build. Restored to the non-root default below.
USER root

ENV DEBIAN_FRONTEND=noninteractive

# ROS 2 requires a UTF-8 locale.
RUN apt-get update && apt-get install -y --no-install-recommends \
        locales \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

# Register the ROS 2 apt repository (current official method: the
# ros-apt-source package, replaces the old manual apt-key setup).
RUN apt-get update && apt-get install -y --no-install-recommends \
        software-properties-common curl ca-certificates gnupg \
    && add-apt-repository universe \
    && ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
    && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb" \
    && apt-get install -y /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 Jazzy, headless (ros-base — no rviz/GUI tools since we're streaming
# Isaac Sim itself over WebRTC, not forwarding X11) plus the message
# packages Isaac Sim's ROS 2 bridge depends on.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-base \
        ros-jazzy-vision-msgs \
        ros-jazzy-ackermann-msgs \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# For interactive shells only (debugging via `--entrypoint bash` / `docker
# exec`) — NOT set as an image-wide ENV. Isaac Sim's own launch chain
# (runheadless.sh -> isaac-sim.streaming.sh -> setup_ros_env.sh) auto-detects
# Ubuntu 24.04 and configures the ROS 2 bridge itself using its bundled
# internal Jazzy libraries, but only when it finds $ROS_DISTRO unset; a
# global ENV here would suppress that and break the bridge (it did, in
# testing — see CLAUDE.md).
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc

WORKDIR /isaac-sim

# Runs after Isaac Sim finishes loading (see autoload_stage.py): opens
# /isaac-sim/content/scene.usd if run.sh's content mount has one saved,
# otherwise leaves the default blank stage untouched.
COPY autoload_stage.py /isaac-sim/autoload_stage.py

# Restore the base image's documented rootless runtime default.
USER 1234:1234

# Default: launch Isaac Sim headless with WebRTC streaming. Its own launch
# chain handles ROS 2 bridge setup. Override with `--entrypoint bash` for an
# interactive debug shell (source /opt/ros/jazzy/setup.bash there for the
# ros2 CLI / colcon workspaces — that's what the apt-installed ROS 2 is for).
CMD ["./runheadless.sh", "--exec", "/isaac-sim/autoload_stage.py"]
