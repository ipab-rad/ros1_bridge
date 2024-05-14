FROM ros:noetic-ros-base-focal AS base

# Switch to much faster mirror for apt processes
ENV OLD_MIRROR archive.ubuntu.com
ENV SEC_MIRROR security.ubuntu.com
ENV NEW_MIRROR mirror.bytemark.co.uk

RUN sed -i "s/$OLD_MIRROR\|$SEC_MIRROR/$NEW_MIRROR/g" /etc/apt/sources.list

# Install basic dev tools (And clean apt cache afterwards)
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive \
        apt-get -y --quiet --no-install-recommends install \
        # Pip for Python3
        # python3-pip \
        # Follow instructions for ROS2 installation
        software-properties-common \
        curl \
    && rm -rf /var/lib/apt/lists/*

# Follow instructions for ROS2 installation
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 main debs
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive \
        apt-get -y --quiet --no-install-recommends upgrade \
    && DEBIAN_FRONTEND=noninteractive \
        apt-get -y --quiet --no-install-recommends install \
        ros-galactic-ros-base \
        ros-dev-tools \
        ros-galactic-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS1 environment variables [For AV server]
ENV ROS_MASTER_URI=http://172.31.0.1:11311/
ENV ROS_IP=172.31.0.1

# Setup ROS1 msgs workspace folder
ENV ROS1_WS /opt/ros1_msgs_ws

# Setup ROS2 msgs workspace folder
ENV ROS2_WS /opt/ros2_msgs_ws

# Set cyclone DDS ROS RMW
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Setup ROS1-2 bridge workspace
ENV BRIDGE_WS /opt/ros1_bridge_ws

# -----------------------------------------------------------------------

FROM base AS runtime

# Bring both msg files into the ROS1 and ROS2 workspaces
COPY ecal_to_ros/ros1/ $ROS1_WS/src/ecal_to_ros/
COPY ecal_to_ros/ros2/ $ROS2_WS/src/ecal_to_ros/

# Source ROS1 setup for dependencies and build our code
WORKDIR $ROS1_WS
RUN . /opt/ros/noetic/setup.sh && \
    catkin_make -DCMAKE_BUILD_TYPE=Release

# Source ROS2 setup for dependencies and build our code
WORKDIR $ROS2_WS
RUN unset ROS_DISTRO && \
    . /opt/ros/galactic/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Bring all ROS1-2 bridge files over
COPY bin $BRIDGE_WS/src/ros1_bridge/bin
COPY cmake $BRIDGE_WS/src/ros1_bridge/cmake
COPY include $BRIDGE_WS/src/ros1_bridge/include
COPY resource $BRIDGE_WS/src/ros1_bridge/resource
COPY ros1_bridge $BRIDGE_WS/src/ros1_bridge/ros1_bridge
COPY src $BRIDGE_WS/src/ros1_bridge/src
COPY test $BRIDGE_WS/src/ros1_bridge/test
COPY CMakeLists.txt package.xml $BRIDGE_WS/src/ros1_bridge/

# Source ROS setup for dependencies and build our code
WORKDIR $BRIDGE_WS
RUN . "$ROS1_WS"/devel/setup.sh && \
    . "$ROS2_WS"/install/setup.sh && \
    BUILD_WORKERS=$(($(nproc) - 2)) && \
    colcon build --executor parallel --parallel-workers "$BUILD_WORKERS" \
    --cmake-force-configure --cmake-args -DCMAKE_BUILD_TYPE=Release

# Add command to docker entrypoint to source newly compiled code when running docker container
RUN sed --in-place --expression \
      "\$isource \"$BRIDGE_WS/install/setup.bash\" " \
      /ros_entrypoint.sh

# launch ros package
CMD ["ros2", "run", "ros1_bridge", "dynamic_bridge"]

# -----------------------------------------------------------------------

FROM base AS dev

# Install basic dev tools (And clean apt cache afterwards)
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive \
        apt-get -y --quiet --no-install-recommends install \
        # Command-line editor
        nano \
        # Ping network tools
        inetutils-ping \
        # Bash auto-completion for convenience
        bash-completion \
    && rm -rf /var/lib/apt/lists/*

# Optional: To limit cpu since ros1_bridge is a big boii `export MAKEFLAGS="-j 4"`

# Add colcon build alias for convenience
RUN echo 'alias colcon_build="colcon build --symlink-install --cmake-args &&\
    -DCMAKE_BUILD_TYPE=Release &&\
     source install/setup.bash"' >> /root/.bashrc

# Enter bash for development
CMD ["bash"]
