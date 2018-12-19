# /bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: bash deploy.bash DUCKIEBOT"
    exit 1
fi

DUCKIEBOT="$1"
CONTAINERS=( "ros-picam" "joystick" "intnav" )
IMAGE_NAME="seleschaefer/intnav:0.7"

# Stop and remove existing containers. 
echo "Stop and remove existing containers"
for name in "${CONTAINERS[@]}"; do
    started=$(docker -H $DUCKIEBOT.local ps --filter="name=$name" -q | xargs)
    [[ -n $started ]] && docker -H $DUCKIEBOT.local stop $started
    existing=$(docker -H $DUCKIEBOT.local ps -a --filter="name=$name" -q | xargs)
    [[ -n $existing ]] && docker -H $DUCKIEBOT.local rm $existing
done

# Create all needed containers. 
echo "Restart containers"
#docker -H $DUCKIEBOT.local create -it --name ros-picam --network=host  --device /dev/vchiq -v /data:/data  duckietown/rpi-duckiebot-ros-picam:master18
#docker -H $DUCKIEBOT.local start ros-picam
#docker -H $DUCKIEBOT.local run -dit --privileged --name joystick --network=host -v /data:/data duckietown/rpi-duckiebot-joystick-demo:master18
#docker -H $DUCKIEBOT.local start joystick
docker -H $DUCKIEBOT.local run -dit --name intnav --network=host -v /data:/data $IMAGE_NAME

# Execute bash in intnav container. 
echo "Executing intnav"
docker -H $DUCKIEBOT.local exec -it intnav /bin/bash

