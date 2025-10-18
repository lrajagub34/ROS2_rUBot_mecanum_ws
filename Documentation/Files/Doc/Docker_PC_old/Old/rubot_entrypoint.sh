#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# source additional workspace setup
source "/root/ROS2_rUBot_mecanum_ws/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "Executing the main command..."
exec "$@"

