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

# Get RAFT-stereo and install its pre-requisites
RUN cd /home/ && git clone https://github.com/princeton-vl/RAFT-Stereo.git
RUN cd /home/RAFT-Stereo/ && git checkout f1fa15abd34187d101806f65b813f4d9d6f93ab0
RUN cd /home/RAFT-Stereo/ && ./download_models.sh

# Install base utilities for Conda (used by RAFT for packaging)
RUN apt-get update && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*
# Install miniconda
ENV CONDA_DIR /opt/conda
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
     /bin/bash ~/miniconda.sh -b -p /opt/conda
# Put conda in path so we can use conda activate
ENV PATH=$CONDA_DIR/bin:$PATH

# Get the RAFT packages
RUN cd /home/RAFT-Stereo/ && conda env create -f environment.yaml

# Set up the stereo processing code
RUN mkdir /home/extract/
COPY . /home/extract/
RUN cd /home/extract/catkin_ws/ && \
    rm -rf build/ devel/ && \
    source /opt/ros/noetic/setup.bash && \
    catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 && \
    echo "source /home/extract/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Activate the conda environment (this is a huge pain)
RUN echo "source activate raftstereo" >> ~/.bashrc

# TODO: Get this working
# CMD roslaunch rectified_from_bag extraction.launch
RUN echo "alias process='roslaunch rectified_from_bag extraction.launch'" >> ~/.bashrc
