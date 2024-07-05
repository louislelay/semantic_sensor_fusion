ARG ARCH=osrf
ARG ROS_DISTRO=kinetic
ARG OS_DISTRO=xenial

FROM ${ARCH}/ros:${ROS_DISTRO}-desktop-full-${OS_DISTRO}

ARG USER_ID
ARG GROUP_ID
RUN addgroup --gid $GROUP_ID user
RUN useradd -ou 0 -g $GROUP_ID user

# install xauth for gui applications
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    xauth \
    && rm -rf /var/lib/apt/lists/*


#-----------------------------------------------------------------------------

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    build-essential \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    cmake \
    git \
    libpoco-dev \
    libeigen3-dev \
    net-tools \
    nano \
    terminator \
    wget \
    gedit \
    vim \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*



# install velodyne from the ROS repositories
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    ros-kinetic-velodyne \
    && rm -rf /var/lib/apt/lists/*

# install ROS useful tools
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    ros-kinetic-gmapping \
    ros-kinetic-map-server \
    ros-kinetic-navigation \
    ros-kinetic-robot-state-publisher \
    ros-kinetic-move-base \
    ros-kinetic-amcl \
    ros-kinetic-depthimage-to-laserscan \
    ros-kinetic-joy \
    ros-kinetic-teleop-twist-keyboard \
    ros-kinetic-openni-camera \
    ros-kinetic-rqt-image-view \
    ros-kinetic-rqt-controller-manager \
    ros-kinetic-rqt-joint-trajectory-controller \
    ros-kinetic-cv-camera \
    ros-kinetic-image-view \
    && rm -rf /var/lib/apt/lists/*

# install ROS realsense
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    ros-kinetic-realsense2-camera \
    ros-kinetic-realsense2-description \
    && rm -rf /var/lib/apt/lists/*

# create catkin_ws
RUN mkdir -p /home/catkin_ws/src

#-----------------------------------------------------------------------------

# config terminator
RUN mkdir -p /root/.config/terminator
RUN echo '[global_config]\n\
  title_hide_sizetext = True\n\
[keybindings]\n\
[profiles]\n\
  [[default]]\n\
    icon_bell = False\n\
    background_color = "#300a24"\n\
    cursor_shape = ibeam\n\
    cursor_color = "#aaaaaa"\n\
    foreground_color = "#ffffff"\n\
    show_titlebar = False\n\
    rewrap_on_resize = False\n\
[layouts]\n\
  [[default]]\n\
    [[[window0]]]\n\
      type = Window\n\
      parent = ""\n\
    [[[child1]]]\n\
      type = Terminal\n\
      parent = window0\n\
[plugins]'>>/root/.config/terminator/config

# bash
COPY ./setup.bash /
RUN echo "source /setup.bash">>/root/.bashrc
RUN echo "source /home/catkin_ws/devel/setup.bash">>/root/.bashrc
RUN echo 'export PS1="\[\e[38;5;40m\]\u\[\e[38;5;15m\]@\[\e[38;5;27m\]\h\[\e[38;5;15m\]:\[\e[38;5;128m\]\w\[\033[0m\]$ "'>>/root/.bashrc
RUN echo "export NO_AT_BRIDGE=1">>/root/.bashrc
RUN echo "umask g+w">>/root/.bashrc
USER user
CMD ["bash"]
