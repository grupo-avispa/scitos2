#!/bin/bash
set -e

######### ROS #########
source /opt/ros/${ROS_DISTRO}/setup.bash
source ${OVERLAY_WS}/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/cyclonedds.xml

######### MIRA #########
export MIRA_PATH=${MIRA_WS}
export PATH=$PATH:${MIRA_WS}/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${MIRA_WS}/lib
source ${MIRA_WS}/scripts/mirabash

exec "$@"