#!/bin/bash
###############################################################################
# Build docker dev stage and add local code for live development              #
###############################################################################

# Build docker image up to dev stage
DOCKER_BUILDKIT=1 docker build \
-t av_bridge_noetic_galactic \
-f Dockerfile --target dev .

# Run docker image with local code volumes for development
docker run -it --rm --net host \
-v /dev/shm:/dev/shm \
-v ./ecal_to_ros/ros1:/opt/ros1_msgs_ws/src/ecal_to_ros \
-v ./ecal_to_ros/ros2:/opt/ros2_msgs_ws/src/ecal_to_ros \
-v ./bin:/opt/ros1_bridge_ws/src/ros1_bridge/bin \
-v ./cmake:/opt/ros1_bridge_ws/src/ros1_bridge/cmake \
-v ./include:/opt/ros1_bridge_ws/src/ros1_bridge/include \
-v ./resource:/opt/ros1_bridge_ws/src/ros1_bridge/resource \
-v ./ros1_bridge:/opt/ros1_bridge_ws/src/ros1_bridge/ros1_bridge \
-v ./src:/opt/ros1_bridge_ws/src/ros1_bridge/src \
-v ./test:/opt/ros1_bridge_ws/src/ros1_bridge/test \
-v ./CMakeLists.txt:/opt/ros1_bridge_ws/src/ros1_bridge/CMakeLists.txt \
-v ./package.xml:/opt/ros1_bridge_ws/src/ros1_bridge/package.xml \
av_bridge_noetic_galactic
