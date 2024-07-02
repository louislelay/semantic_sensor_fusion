FROM osrf/ros:noetic-desktop-full-focal

# install xauth for gui applications
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    xauth \
    && rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    net-tools \
    nano \
    terminator \
    wget \
    && rm -rf /var/lib/apt/lists/*

# install ROS useful tools
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    ros-noetic-gmapping \
    ros-noetic-map-server \
    ros-noetic-navigation \
    ros-noetic-robot-state-publisher \
    ros-noetic-move-base \
    ros-noetic-amcl \
    ros-noetic-depthimage-to-laserscan \
    ros-noetic-joy \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-openni-camera \
    ros-noetic-rqt-image-view \
    && rm -rf /var/lib/apt/lists/*


# create catkin_ws
RUN mkdir -p /home

# bash
COPY ./setup.bash /
RUN echo "source /setup.bash">>/root/.bashrc

CMD ["bash"]
