FROM ubuntu:20.04
MAINTAINER Eric franzs@andrew.cmu.edu

# Basic things that should be available
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=America/New_York
RUN apt-get update && \
    apt-get install -y \
        apt-file \
        apt-utils \
        curl \
        git \
        less \
        libzbar0 \
        ffmpeg libsm6 libxext6 \
        lsb-release \
        mlocate \
        nano \
        software-properties-common \
        unzip \
        wget \
        zip

# Necessary in order to "source" things later, like ROS setup scripts
SHELL ["/bin/bash", "-c"]

# Install ROS noetic (ubuntu 20.04)
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && apt-get install -y \
        ros-noetic-cv-bridge \
        ros-noetic-image-proc \
        ros-noetic-image-view \
        ros-noetic-pcl-ros \
        ros-noetic-ros-base \
        ros-noetic-rviz \
        && \
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Stuff required for RAFT and PSM
RUN pip3 install torch==1.7.0 \
         torchvision==0.8.1 \
         tensorboard \
         tqdm \
         opt_einsum \
         imageio \
         py7zr
RUN cd /home/ && git clone https://github.com/princeton-vl/RAFT-Stereo.git
RUN cd /home/RAFT-Stereo/ && git checkout f1fa15abd34187d101806f65b813f4d9d6f93ab0

# Set up the stereo processing code
RUN mkdir /home/extract/ && \
    cd /home/extract/ && \
    echo 'alias extract="python3 /home/extract/catkin_ws/src/rectified_from_bag/scripts/rectified_from_bag/extract.py"' >> ~/.bash_aliases
COPY . /home/extract/
RUN cd /home/extract/catkin_ws/ && \
    rm -rf build/ devel/ && \
    source /opt/ros/noetic/setup.bash && \
    catkin_make clean && \
    catkin_make && \
    echo "source /home/extract/catkin_ws/devel/setup.bash" >> ~/.bashrc
