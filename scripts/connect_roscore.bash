# /bin/bash
if [ "$#" -ne 1 ]; then
    echo "Usage: bash connect_roscore.bash DUCKIEBOT"
fi

DUCKIEBOT="$1"

export ROS_MASTER_URI=http://$DUCKIEBOT.local:11311