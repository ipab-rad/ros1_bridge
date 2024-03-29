#!/bin/bash
###############################################################################
# Build docker full image and run ROS code interactively for debugging        #
###############################################################################

# Build docker image only up to base stage
DOCKER_BUILDKIT=1 docker build \
-t av_bridge_noetic_galactic \
-f Dockerfile --target build .

# Run docker image without volumes
docker run -it --rm --net host \
-v /dev/shm:/dev/shm \
av_bridge_noetic_galactic
