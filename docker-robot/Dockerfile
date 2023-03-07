# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images/create_ros_image.Dockerfile.em
FROM ros:noetic-ros-core-focal

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*
    
# ----- Below this line added by Dustin for Moonrockers -----
RUN apt update && \
apt install -y git python3-pip xauth vim tmux iproute2 python3-tk \
    python3-opencv libopencv-dev libeigen3-dev

RUN pip3 install pysimplegui

# compile the ArUco library
# downloaded from https://sourceforge.net/projects/aruco/
ADD resources/aruco-3.1.12.tar.gz /build/aruco/
# Docker automatically extracts the archive, so we can go straight to the directory
WORKDIR /build/aruco/aruco-3.1.12
RUN mkdir build
WORKDIR build
RUN cmake ..
RUN make -j4
RUN make install

# install YouCompleteMe
ADD resources/.vimrc /root/
RUN git clone https://github.com/VundleVim/Vundle.vim.git /root/.vim/bundle/Vundle.vim
RUN vim +:PluginInstall +:qa
RUN apt install -y build-essential cmake vim-nox python3-dev
# this isn't a requirement of YouCompleteMe itself, but we need to use
#  "catkin build" instead of "catkin_make" for YouCompleteMe to work with ROS
RUN apt install -y python3-catkin-tools
WORKDIR /root/.vim/bundle/YouCompleteMe
RUN python3 install.py --clang-completer --force-sudo

# add an almost-default Ubuntu 18 .bashrc
ADD resources/.bashrc /root/.bashrc

WORKDIR /workspace/production/ros
