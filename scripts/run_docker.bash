# /bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: bash run_docker.bash DUCKIEBOT"
fi

DUCKIEBOT="$1"
DOCKER_NAME="seleschaefer"
PACKAGE="intnav"
TAG="0.4"

docker -H $DUCKIEBOT.local run -dit --name intnav --network=host -v /data:/data $DOCKER_NAME/$PACKAGE:$TAG
docker -H $DUCKIEBOT.local exec -it intnav /bin/bash
