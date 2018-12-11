# /bin/bash

DUCKIEBOT="sebot"
DOCKER_NAME="seleschaefer"
PACKAGE="intnav"
TAG="0.4"

docker -H $DUCKIEBOT.local run -dit --name intnav --network=host -v /data:/data $DOCKER_NAME/$PACKAGE:$TAG
docker -H $DUCKIEBOT.local exec -it intnav /bin/bash
