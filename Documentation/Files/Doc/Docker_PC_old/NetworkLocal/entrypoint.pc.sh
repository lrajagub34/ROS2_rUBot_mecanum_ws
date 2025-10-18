#!/bin/bash
set -e

# Activa l'entorn de ROS 2
source "/opt/ros/humble/setup.bash"
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Important per a descobriment manual (si multicast falla)
export CYCLONEDDS_URI=/root/cyclonedds_pc.xml

# Executa la comanda que li passi docker-compose (p.ex., "sleep infinity")
exec "$@"
