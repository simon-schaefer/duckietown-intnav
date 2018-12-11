# /bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: bash run_docker.bash DUCKIEBOT"
    exit 1
fi

DUCKIEBOT="$1"

docker -H $DUCKIEBOT.local create -it --net host --memory="800m" --memory-swap="1.8g" --privileged -v /data:/data --name lane_following duckietown/rpi-duckiebot-lanefollowing-demo:master18
docker -H $DUCKIEBOT.local create -it --name ros-picam --network=host  --device /dev/vchiq -v /data:/data  duckietown/rpi-duckiebot-ros-picam:master18
docker -H $DUCKIEBOT.local create -dit --name intnav --network=host -v /data:/data $DOCKER_NAME/$PACKAGE:$TAG