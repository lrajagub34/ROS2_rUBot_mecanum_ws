#!/bin/bash
set -e

# Activa l'entorn de ROS 2
source "/opt/ros/humble/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Opcional: Si has construït un workspace al PC, activa'l també
# source "/root/pc_ws/install/setup.bash"

# Executa la comanda que li passi docker-compose (p.ex., "sleep infinity")
exec "$@"
