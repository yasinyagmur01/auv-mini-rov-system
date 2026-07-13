#!/bin/bash
set -e

# ROS 2 (apt kurulumu) + calisma alani ortamini kur
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "/root/ros2_ws/install/local_setup.bash" ]; then
    source "/root/ros2_ws/install/local_setup.bash"
fi

echo "[AUV] ROS ${ROS_DISTRO} hazir — ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"

exec "$@"
