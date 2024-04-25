#!/bin/bash
# ---------------------------------------------------------------------------
# Build docker image and run ROS code for runtime or interactively with bash
# ---------------------------------------------------------------------------

# Initialise CMD as empty
CMD=""

# If an arg is defined, start container with bash
if [ -n "$1" ]; then
    CMD="bash"
fi

# Build docker image only up to base stage
DOCKER_BUILDKIT=1 docker build \
    -t av_bridge_noetic_galactic \
    -f Dockerfile --target runtime .

# Run docker image without volumes
docker run -it --rm --net host --privileged \
    -v /dev/shm:/dev/shm \
    av_bridge_noetic_galactic $CMD
