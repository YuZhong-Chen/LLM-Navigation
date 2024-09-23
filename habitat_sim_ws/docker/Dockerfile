FROM nvidia/cudagl:11.3.0-devel-ubuntu20.04

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
    ca-certificates \
    curl \
    git \
    git-extras \
    git-lfs \
    build-essential \
    htop \
    tree \
    net-tools \
    tmux \
    vim \
    wget \
    zip \
    unzip \
    python3-pip \
    mesa-utils-extra \
    # Libraries
    libjpeg-dev \
    libpng-dev \
    # libglfw3-dev \
    # libglm-dev \
    libx11-dev \
    libomp-dev \
    libegl1-mesa-dev \
    && rm -rf /var/lib/apt/lists/*

# Install conda
RUN curl -L -o ~/miniconda.sh -O  https://repo.continuum.io/miniconda/Miniconda3-latest-Linux-x86_64.sh  &&\
    chmod +x ~/miniconda.sh &&\
    ~/miniconda.sh -b -p /opt/conda &&\
    rm ~/miniconda.sh &&\
    /opt/conda/bin/conda install numpy pyyaml scipy ipython mkl mkl-include &&\
    /opt/conda/bin/conda clean -ya
ENV PATH /opt/conda/bin:$PATH

# Install cmake
RUN wget https://github.com/Kitware/CMake/releases/download/v3.14.0/cmake-3.14.0-Linux-x86_64.sh
RUN mkdir /opt/cmake
RUN sh /cmake-3.14.0-Linux-x86_64.sh --prefix=/opt/cmake --skip-license
RUN ln -s /opt/cmake/bin/cmake /usr/local/bin/cmake
RUN cmake --version

# Conda environment
RUN conda create -n habitat python=3.9 cmake=3.14.0
RUN /bin/bash -c ". activate habitat; conda install --yes habitat-sim withbullet -c conda-forge -c aihabitat"

# Install habitat-lab
RUN git clone --branch stable https://github.com/facebookresearch/habitat-lab.git
RUN /bin/bash -c ". activate habitat; cd habitat-lab; pip install -e habitat-lab"
# RUN /bin/bash -c ". activate habitat; cd habitat-lab; python -m habitat_sim.utils.datasets_download --uids habitat_test_scenes --data-path data/"
# RUN /bin/bash -c ". activate habitat; cd habitat-lab; python -m habitat_sim.utils.datasets_download --uids habitat_test_pointnav_dataset --data-path data/"

ENV SHELL /bin/bash
ENV TERM=xterm-256color

# Install tzdata in noninteractive mode.
RUN DEBIAN_FRONTEND=noninteractive apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata

# Install ROS1 Noetic
RUN apt-get update && apt-get install -y lsb-release
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN DEBIAN_FRONTEND=noninteractive apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y ros-noetic-desktop

USER $USERNAME

# Change the owner of habitat-lab
RUN sudo chown user /habitat-lab && sudo chgrp user /habitat-lab

CMD ["/bin/bash"]