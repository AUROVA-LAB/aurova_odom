FROM osrf/ros:noetic-desktop-focal

ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
ENV SHELL /bin/bash


ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
ENV QT_X11_NO_MITSHM=1
ENV EDITOR=nano
ENV XDG_RUNTIME_DIR=/tmp

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y apt-utils curl wget git bash-completion build-essential sudo && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    git build-essential ros-noetic-perception-pcl ros-noetic-rviz ros-noetic-eigen-conversions iputils-ping screen vim nano && \
    rm -rf /var/lib/apt/lists/*

RUN cd /home/epvs && mkdir library

# Ceres dependeces and Install
RUN apt-get update && apt-get install cmake libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev -y

#Command line to download the library automatically from the ceres page.
RUN cd /home/epvs/library && git clone https://ceres-solver.googlesource.com/ceres-solver && cd ceres-solver && mkdir ceres-bin && cd ceres-bin

#Command line to copy the library inside the container for ceres installation (manual download of ceres is required).
#COPY ./ceres-solver-2.2.0.tar.gz /home/epvs/library

RUN cd /home/epvs/library && tar zxf ceres-solver-2.2.0.tar.gz && cd ceres-solver-2.2.0 && mkdir ceres-bin
RUN cd /home/epvs/library/ceres-solver-2.2.0/ceres-bin/  && cmake .. && make -j10 && make install

# Lidar Odometry dependeces
RUN apt-get update && \
    apt-get install -y --no-install-recommends \       
    ros-noetic-hector-trajectory-server     
   
# extras tools
RUN apt-get update && apt-get -y install vim 

# Install DQ library 
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository ppa:dqrobotics-dev/release && \
    apt-get update && \
    apt-get install -y libdqrobotics

# Install GTSAM
# Add PPA
RUN add-apt-repository ppa:borglab/gtsam-release-4.0
RUN apt update && apt install -y libgtsam-dev libgtsam-unstable-dev

USER $USERNAME
# bash update
RUN echo "TERM=xterm-256color" >> ~/.bashrc
RUN echo "# COLOR Text" >> ~/.bashrc
RUN echo "PS1='\[\033[01;33m\]\u\[\033[01;33m\]@\[\033[01;33m\]\h\[\033[01;34m\]:\[\033[00m\]\[\033[01;34m\]\w\[\033[00m\]\$ '" >> ~/.bashrc
RUN echo "CLICOLOR=1" >> ~/.bashrc
RUN echo "LSCOLORS=GxFxCxDxBxegedabagaced" >> ~/.bashrc
RUN echo "" >> ~/.bashrc
RUN echo "## ROS" >> ~/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

WORKDIR ${HOME}
