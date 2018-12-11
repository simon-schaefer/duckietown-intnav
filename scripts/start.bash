# /bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: bash run_docker.bash DUCKIEBOT"
fi

DUCKIEBOT="$1"
DOCKER_NAME="seleschaefer"
PACKAGE="intnav"
TAG="0.5"

docker -H $DUCKIEBOT.local start lane_follower
docker -H $DUCKIEBOT.local start ros-picam
docker -H $DUCKIEBOT.local start joystick
docker -H $DUCKIEBOT.local exec -it intnav /bin/bash
