# /bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: bash deploy_lane_follower.bash DUCKIEBOT"
    exit 1
fi

DUCKIEBOT="$1"
CONTAINERS=( "lane_follower" )
IMAGE_NAME="seleschaefer/intnav:0.7"

# Stop and remove existing containers. 
echo "Stop and remove existing container"
for name in "${CONTAINERS[@]}"; do
    started=$(docker -H $DUCKIEBOT.local ps --filter="name=$name" -q | xargs)
    [[ -n $started ]] && docker -H $DUCKIEBOT.local stop $started
    existing=$(docker -H $DUCKIEBOT.local ps -a --filter="name=$name" -q | xargs)
    [[ -n $existing ]] && docker -H $DUCKIEBOT.local rm $existing
done

# Create all needed containers. 
echo "Restart lane following container"
docker -H $DUCKIEBOT.local create -it --net host --memory="800m" --memory-swap="1.8g" --privileged -v /data:/data --name lane_following seleschaefer/intnav_lane_following:master2018
docker -H $DUCKIEBOT.local start lane_following

# Execute lane following container. 
echo "Execute: roslaunch duckietown_demos lane_following.launch"
docker -H $DUCKIEBOT.local exec -it lane_following /bin/bash



