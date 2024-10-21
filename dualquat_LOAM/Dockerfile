FROM osrf/ros:noetic-desktop-focal

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y apt-utils curl wget git bash-completion build-essential sudo && rm -rf /var/lib/apt/lists/*

# Now create the user
ARG UID=1000
ARG GID=1000
RUN addgroup --gid ${GID} docker
RUN adduser --gecos "ROS User" --disabled-password --uid ${UID} --gid ${GID} docker
RUN usermod -a -G dialout docker
RUN mkdir config && echo "ros ALL=(ALL) NOPASSWD: ALL" > config/99_aptget
RUN cp config/99_aptget /etc/sudoers.d/99_aptget
RUN chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget

RUN apt-get update && apt-get install -y apt-utils curl wget git bash-completion build-essential sudo && rm -rf /var/lib/apt/lists/*

ENV HOME /home/docker
RUN mkdir -p ${HOME}/catkin_ws/library

# Install Ceres 
RUN apt-get update && apt-get install cmake libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev -y
RUN cd ${HOME}/catkin_ws/library && wget http://ceres-solver.org/ceres-solver-2.2.0.tar.gz
RUN cd ${HOME}/catkin_ws/library && tar zxf ceres-solver-2.2.0.tar.gz && cd ceres-solver-2.2.0 && mkdir ceres-bin && cd ceres-bin && cmake .. && cmake .. && make -j$(nproc) && make install


# ###########  

# Lidar Odometry dependeces
RUN apt-get update && \
    apt-get install -y --no-install-recommends \       
    ros-noetic-hector-trajectory-server  \   
    vim  # extras tools


RUN . /opt/ros/noetic/setup.sh && \
    mkdir -p ${HOME}/catkin_ws/src && \ 
    cd ${HOME}/catkin_ws/src   

  
# Install DQ library 
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository ppa:dqrobotics-dev/release && \
    apt-get update

# Instalar libdqrobotics
RUN apt-get install -y libdqrobotics

# STD dependences
# Add PPA
RUN add-apt-repository ppa:borglab/gtsam-release-4.0
RUN apt update && apt install -y libgtsam-dev libgtsam-unstable-dev

# install nanoflann
RUN cd ${HOME}/catkin_ws/library && git clone https://github.com/EPVelasco/nanoflann.git && cd nanoflann && mkdir build && cd build && cmake .. && sudo make install 

# catkin_make dualquat_loam
RUN mkdir ${HOME}/catkin_ws/src/dualquat_LOAM/
COPY . ${HOME}/catkin_ws/src/dualquat_LOAM/
RUN . /opt/ros/noetic/setup.sh && \ 
     cd ${HOME}/catkin_ws/ && catkin_make --only-pkg-with-deps dualquat_loam 
     
     
# Install Aurova preprocessing
RUN cd ${HOME}/catkin_ws/src/ && git clone https://github.com/AUROVA-LAB/aurova_preprocessed.git
RUN . /opt/ros/noetic/setup.sh && \ 
     cd ${HOME}/catkin_ws/ && catkin_make --only-pkg-with-deps pc_feature_extraction 

     
# bash update
RUN echo "TERM=xterm-256color" >> ~/.bashrc
RUN echo "# COLOR Text" >> ~/.bashrc
RUN echo "PS1='\[\033[01;33m\]\u\[\033[01;33m\]@\[\033[01;33m\]\h\[\033[01;34m\]:\[\033[00m\]\[\033[01;34m\]\w\[\033[00m\]\$ '" >> ~/.bashrc
RUN echo "CLICOLOR=1" >> ~/.bashrc
RUN echo "LSCOLORS=GxFxCxDxBxegedabagaced" >> ~/.bashrc
RUN echo "" >> ~/.bashrc
RUN echo "## ROS" >> ~/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

############
# Configura la variable de entorno DISPLAYWORKDIR
USER docker
ENV DISPLAY=:0    
ENV NVIDIA_DRIVER_CAPABILITIES graphics,compute,utility
WORKDIR ${HOME}/catkin_ws 
