# /bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: bash rviz_config.bash DUCKIEBOT"
fi

DUCKIEBOT="$1"
SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"
cd $SCRIPTPATH/../ros-intnav/config/

sed "s/DUCKIEBOT/$DUCKIEBOT/g" rviz.rviz > rviz_bot.rviz