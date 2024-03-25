FROM ros:noetic-ros-base-focal

# Install basic dev tools (And clean apt cache afterwards)
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive \
        apt -y --quiet --no-install-recommends install \
        # Command-line editor
        nano \
        # Ping network tools
        inetutils-ping \
        # Pip for Python3
        python3-pip \
        # Bash auto-completion for convenience
        bash-completion \
        # Follow instructions for ROS2 installation
        software-properties-common \
        curl \
    && rm -rf /var/lib/apt/lists/*

# Follow instructions for ROS2 installation
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 main debs
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive \
        apt -y --quiet --no-install-recommends upgrade \
    && DEBIAN_FRONTEND=noninteractive \
        apt -y --quiet --no-install-recommends install \
        ros-galactic-ros-base \
        ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS1 msgs workspace folder and copy files over
ENV ROS1_WS /opt/ros1_msgs_ws
RUN mkdir -p $ROS1_WS/src/ecal_to_ros
WORKDIR $ROS1_WS
ADD ecal_to_ros/ros1/ $ROS1_WS/src/ecal_to_ros/

# Source ROS1 setup for dependencies and build our code
RUN . /opt/ros/noetic/setup.sh && \
    catkin_make -DCMAKE_BUILD_TYPE=Release

# Setup ROS2 msgs workspace folder and copy files over
ENV ROS2_WS /opt/ros2_msgs_ws
RUN mkdir -p $ROS2_WS/src/ecal_to_ros
WORKDIR $ROS2_WS
ADD ecal_to_ros/ros2/ $ROS2_WS/src/ecal_to_ros/

# Source ROS2 setup for dependencies and build our code
RUN unset ROS_DISTRO && \
    . /opt/ros/galactic/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Setup ROS1-2 bridge workspace
ENV BRIDGE_WS /opt/ros1_bridge_ws
RUN mkdir -p $BRIDGE_WS/src/ros1_bridge
WORKDIR $BRIDGE_WS

# Bring all ROS1-2 bridge files over
ADD bin $BRIDGE_WS/src/ros1_bridge/bin
ADD cmake $BRIDGE_WS/src/ros1_bridge/cmake
ADD include $BRIDGE_WS/src/ros1_bridge/include
ADD resource $BRIDGE_WS/src/ros1_bridge/resource
ADD ros1_bridge $BRIDGE_WS/src/ros1_bridge/ros1_bridge
ADD src $BRIDGE_WS/src/ros1_bridge/src
ADD test $BRIDGE_WS/src/ros1_bridge/test
ADD CMakeLists.txt package.xml $BRIDGE_WS/src/ros1_bridge

# Optional: To limit cpu since ros1_bridge is a big boii `export MAKEFLAGS="-j 4"`

# # Source ROS setup for dependencies and build our code
RUN . $ROS1_WS/devel/setup.sh && \
    . $ROS2_WS/install/setup.sh && \
    colcon build --symlink-install --cmake-force-configure --cmake-args -DCMAKE_BUILD_TYPE=Release

# Add command to docker entrypoint to source newly compiled code when running docker container
RUN sed --in-place --expression \
      '$isource "$BRIDGE_WS/install/setup.bash"' \
      /ros_entrypoint.sh

# Add sourcing local workspace command to bashrc for convenience when running interactively
RUN echo "source $BRIDGE_WS/install/setup.bash" >> /root/.bashrc

# launch ros package
CMD ["ros2", "run", "ros1_bridge", "dynamic_bridge"]