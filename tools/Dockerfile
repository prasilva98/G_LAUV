ARG BASE_IMAGE=garden

############################################################
###                        Base stage                    ###
############################################################

# Start off from ubuntu 20.04 base image with nvidia goodies (for 3D stuff of simulator)
FROM ros:humble-ros-core-jammy AS base

# This avoids keyboard interaction when asked for geographic area
ARG DEBIAN_FRONTEND=noninteractive

# Install basic dependencies
RUN apt-get -qq update && apt-get -q install -y \
    curl \
    git \
    gnupg2 \
    lsb-release \
    software-properties-common \
    sudo \
    tmux \
    tzdata \
    wget \
    gedit \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get -qq clean

## Install Gazebo build tools ##

# Adds ROS package to package manager and downloads authentication key
RUN echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list && \
    wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -O - | apt-key add -

# More dependencies, mostly colcon (ROS packages build tool)
RUN apt-get -qq update && apt-get -q -y install \
    build-essential \
    cmake \
    python3-vcstool \
    python3-colcon-common-extensions \
    && apt-get dist-upgrade -y \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get -qq clean

# Add a user with the same user_id as the user outside the container
# Usefull to easily exchange data between local computer and container 
ENV USERNAME developer
RUN addgroup --gid 1000 $USERNAME && \
    adduser --uid 1000 --ingroup $USERNAME \
      --home /home/$USERNAME --shell /bin/bash $USERNAME && \
    echo "$USERNAME:$USERNAME" | chpasswd && \
    adduser $USERNAME sudo && \
    echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USERNAME

RUN curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.5.1/fixuid-0.5.1-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \
    chown root:root /usr/local/bin/fixuid && \
    chmod 4755 /usr/local/bin/fixuid && \
    mkdir -p /etc/fixuid && \
    printf "user: $USERNAME\ngroup: $USERNAME\n" > /etc/fixuid/config.yml

# Create a setup bash script for convenience. 
RUN touch /setup.sh && chmod 0644 /setup.sh

# Entry point goes will be an bash environment
ENTRYPOINT [ "bash" ]

# Commands below run as the developer user
USER $USERNAME

# When running a container start in the developer's home folder
WORKDIR /home/$USERNAME

############################################################
###                  Gazebo Garden stage                 ###
############################################################
FROM base AS garden

# Folder for Gazebo Garden source code
ENV GZ_WS=/home/$USERNAME/gz_ws
RUN mkdir -p $GZ_WS/src
# Download collection of gazebo packages needed
RUN cd $GZ_WS && wget https://raw.githubusercontent.com/gazebo-tooling/gazebodistro/master/collection-garden.yaml
# Pull the source code
RUN cd $GZ_WS && vcs import src < collection-garden.yaml && vcs pull src
USER root

# Add gazebo packages to package handler and authentication keys
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list \
&& echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-nightly $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-nightly.list \
&& wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

# Install dependencies
RUN apt-get -qq update && apt-get -q install -y \
  $(sort -u $(find $GZ_WS/src -iname "packages-$(lsb_release -cs).apt" -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ') \
  libpcl-dev \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get -qq clean

# Build Gazebo
USER $USERNAME
RUN cd $GZ_WS && colcon build --merge-install --cmake-args -DBUILD_TESTING=OFF

# Add the sourcing of the installation to setup.sh
USER root
RUN echo ". $GZ_WS/install/setup.sh" >> /setup.sh && \
    echo "export PYTHONPATH=$GZ_WS/install/lib/python" >> /setup.sh
RUN chown -R root:root ${GZ_WS}

USER $USERNAME

############################################################
###             LRAUV simulation baseline stage          ###
############################################################
FROM ${BASE_IMAGE} AS lrauv-base

# More dependencies for LRAUV
USER root
RUN apt-get -qq update && apt-get -q install -y \
    libeigen3-dev \
    libpcl-dev \
    python3-empy \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get -qq clean

USER $USERNAME

ENV LRAUV_WS=/home/$USERNAME/lrauv_ws

# Create directory for LRAUV
RUN mkdir -p $LRAUV_WS/src

# Clone LRAUV repository
RUN git clone https://github.com/osrf/lrauv.git $LRAUV_WS/src
#RUN chown -R $USERNAME:$USERNAME $LRAUV_WS/src/lrauv
WORKDIR $LRAUV_WS

# Run the build process
RUN . /setup.sh && colcon build --merge-install --cmake-args -DBUILD_TESTING=OFF

############################################################
###                  LRAUV simulation stage              ###
############################################################
FROM lrauv-base AS lrauv

RUN cd $LRAUV_WS; . /setup.sh; \
    colcon build --merge-install --cmake-args -DBUILD_TESTING=OFF

USER root
RUN echo ". $LRAUV_WS/install/setup.sh" >> /setup.sh
RUN chown -R root:root ${LRAUV_WS}

############################################################
###                  ROS<->Gazebo                        ###
############################################################

# Add ROS to source library
RUN echo ". /opt/ros/humble/setup.sh" >> /setup.sh

USER root
RUN locale && \
    apt-get update && sudo apt install locales && \
    locale-gen en_US en_US.UTF-8 && \ 
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8
RUN locale  # verify settings

RUN . /setup.sh && \
    ros2

RUN cd $LRAUV_WS/src && \ 
    . /setup.sh && \
    # Specify a macro with the correct gazebo version for the bridge install
    export GZ_VERSION=garden && \
    # Clone the gazebo ros bridge repo
    git clone https://github.com/gazebosim/ros_gz.git -b humble && cd .. && \ 
    apt-get -q install -y ros-dev-tools && \  
    rosdep init && \
    rosdep update && \
    rosdep install -r --from-paths src -i -y --rosdistro humble && \
    colcon build --merge-install

RUN cd $LRAUV_WS/src && \
    . /setup.sh && \ 
    ls && \
    git clone https://github.com/prasilva98/G_LAUV.git && \
    cd .. && rosdep install -r --from-paths src -i -y --rosdistro humble && \
    colcon build --merge-install

# Install pip && x11 
RUN apt-get update && \
    apt-get install -y python3-pip && \
    apt-get install x11-apps && \
    apt-get install x11-xserver-utils && \
    pip install pandas 












