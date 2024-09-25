# Base Image: https://hub.docker.com/r/osrf/ros/tags?page=1&name=humble
FROM osrf/ros:noetic-desktop-full AS amd64
# Base Image: https://hub.docker.com/r/arm64v8/ros/tags?page=1&name=humble
FROM arm64v8/ros:noetic AS arm64

# Use docker automatic platform args to select the base image.
# It may be `arm64` or `amd64` depending on the platform.
# Ref: https://docs.docker.com/reference/dockerfile/#automatic-platform-args-in-the-global-scope
FROM $TARGETARCH
ARG TARGETARCH

LABEL org.opencontainers.image.authors="assume0701@gmail.com"

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
# add user with default bash
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Install common tools
RUN apt-get update && apt-get install -y \
    bash-completion \
    curl \
    git \
    git-extras \
    htop \
    tree \
    net-tools \
    tmux \
    vim \
    wget \
    nlohmann-json3-dev \
    libgoogle-glog-dev \
    libprotobuf-dev \
    libzmqpp-dev \
    protobuf-compiler \
    python3-pip \
    python3-rosdep \
    python3-catkin-tools \
    python3-vcstool \
    ros-noetic-gtsam \
    && rm -rf /var/lib/apt/lists/*

# Install python packages
RUN pip install timm==1.0.9
RUN pip install git+https://github.com/openai/CLIP.git

RUN apt-get update && apt-get dist-upgrade -y \
    && apt-get autoremove -y \
    && apt-get autoclean -y \
    && rm -rf /var/lib/apt/lists/*

ENV SHELL /bin/bash
ENV TERM=xterm-256color

RUN mkdir -p /home/user/catkin_ws/src/hydra

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]