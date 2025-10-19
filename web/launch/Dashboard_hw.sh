#!/bin/bash

# Funció per tancar processos en segon pla
cleanup() {
  echo "Tancant rosbridge_server i servidor web..."
  pkill -P $ROSBRIDGE_PID 2>/dev/null
  kill $ROSBRIDGE_PID $WEBSERVER_PID 2>/dev/null
  exit
}

# Captura Ctrl+C
trap cleanup SIGINT

# Allibera el port 9090 si està ocupat
if lsof -i :9090 > /dev/null; then
  echo "⚠️ El port 9090 està ocupat. Tancant processos anteriors..."
  lsof -ti :9090 | xargs kill -9
fi

# Allibera el port 8000 si està ocupat
if lsof -i :8000 > /dev/null; then
  echo "⚠️ El port 8000 està ocupat. Tancant processos anteriors..."
  lsof -ti :8000 | xargs kill -9
fi

# Llança rosbridge
echo "Iniciant rosbridge_server..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ROSBRIDGE_PID=$!

sleep 8

# Canvia al directori web
echo "Canviant al directori web..."
cd ~/ROS2_rUBot_mecanum_ws/web || exit

# Llança servidor web
echo "Iniciant servidor web a http://localhost:8000..."
python3 -m http.server 8000 &
WEBSERVER_PID=$!

sleep 8

# Mostra adreça web en terminal separat
#xterm -hold -e ~/bin/webpage_address &

# Espera que els processos acabin
wait $ROSBRIDGE_PID $WEBSERVER_PID
