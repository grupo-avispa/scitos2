#!/bin/bash
set -e

######### ROS #########
if [ -d /opt/ros/$ROS_DISTRO ]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
    source ${OVERLAY_WS}/install/local_setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=/cyclonedds.xml
fi

######### MIRA #########
if [ -d $MIRA_WS/mira ]; then
    export MIRA_PATH=${MIRA_WS}
    export PATH=$PATH:${MIRA_WS}/bin
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${MIRA_WS}/lib
    source ${MIRA_WS}/scripts/mirabash
fi

exec "$@"