#!/bin/bash
# ----------------------------------------------------------------
# Build docker dev stage and add local code for live development
# ----------------------------------------------------------------

CYCLONE_VOL=""
BASH_CMD=""

# Default cyclone_dds.xml path
CYCLONE_DIR=/home/$USER/cyclone_dds.xml

# Function to print usage
usage() {
    echo "
Usage: dev.sh [-b|bash] [-l|--local] [-h|--help]

Where:
    -b | bash       Open bash in docker container (Default in dev.sh)
    -l | --local    Use default local cyclone_dds.xml config
    -l | --local    Optionally point to absolute -l /path/to/cyclone_dds.xml
    -h | --help     Show this help message
    "
    exit 1
}

# Parse command-line options
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -b|bash)
            BASH_CMD=bash
            ;;
        -l|--local)
            # Avoid getting confused if bash is written where path should be
            if [ -n "$2" ]; then
                if [ ! $2 = "bash" ]; then
                    CYCLONE_DIR="$2"
                    shift
                fi
            fi
            CYCLONE_VOL="-v $CYCLONE_DIR:/opt/ros_ws/cyclone_dds.xml"
            ;;
        -h|--help)
            usage
            ;;
        *)
            echo "Unknown option: $1"
            usage
            ;;
    esac
    shift
done

# Verify CYCLONE_DIR exists
if [ ! -f "$CYCLONE_DIR" ]; then
    echo "$CYCLONE_DIR does not exist! Please provide a valid path to cyclone_dds.xml"
    exit 1
fi

# Build docker image up to dev stage
DOCKER_BUILDKIT=1 docker build \
    -t av_ros1_bridge:latest-dev \
    -f Dockerfile --target dev .

# Run docker image with local code volumes for development
docker run -it --rm --net host --privileged \
    -v /dev:/dev \
    -v /tmp:/tmp \
    -v /etc/localtime:/etc/localtime:ro \
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
    $CYCLONE_VOL \
    av_ros1_bridge:latest-dev
