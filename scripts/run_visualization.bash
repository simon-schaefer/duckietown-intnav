# /bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: bash run_visualization.bash DUCKIEBOT"
fi
DUCKIEBOT="$1"

roslaunch duckietown-intnav visualization.launch duckiebot:=$DUCKIEBOT 