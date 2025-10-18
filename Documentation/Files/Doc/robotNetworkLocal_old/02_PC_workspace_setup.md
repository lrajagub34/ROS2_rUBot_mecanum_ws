# **PC Workspace setup**

The robots will be connectes to internet and we will connect to the robots usin a PC/labtop. Different possibilites appear:
- PC connection to my_robot within the same local network
- PC connection to a PC-server connected to my_robot in a VPN Internet network

## **1. PC connection to my_robot within the same local network**

This is the fastest method to connect to my_robot when PC and robot are in the same laboratory connected to a local network within a router.

We will proceed with:
- Install in the control PC Docker Desktop
- Create a "Docker_PC" folder in your control PC with the contents of "Docker_PC"
- A custon ``Dockerfile.pc`` is created for ROS2-Humble environment
- In case you want to create a custom "rubot_humble_image", type in a new terminal:
  ````shell
  docker build -t rubot_humble_image .
  ````
- If you want to create the image and the container at the same time, a custom ``docker-compose.pc.yaml`` file is created to setup `pc_humble_container` over the local network
  ````shell
  docker system prune
  docker compose -f docker-compose.pc.yaml up -d --build
  ````
- Open VScode in your PC, identify the running containers and Attach a ``VScode Workspace to this Container``
- Verify the `.bashrc`file:
  ````shell
  source /opt/ros/humble/setup.bash
  source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
  export ROS_DOMAIN_ID=0
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  #export GAZEBO_MODEL_PATH=/home/user/ROS2_rUBot_mecanum_ws/src/my_robot_bringup/models:$GAZEBO_MODEL_PATH
  #source /home/user/ROS2_rUBot_tutorial_ws/install/setup.bash
  #source /home/user/ROS2_rUBot_mecanum_ws/install/setup.bash
  #cd /home/user/ROS2_rUBot_tutorial_ws
  #cd /home/user/ROS2_rUBot_mecanum_ws
````
- Open a new ``terminal`` and type:
  ````shell
  ros2 node list
  ````
- Identify the bringup nodes ready when powering your robot.

You are now ready to control your robot!

## **WSL2 based setup local network**

📌 Instructions:
1. robot:
  ````shell
  chmod +x setup_robot.sh
  ./setup_robot.sh
  source ~/.bashrc
  ````
2. PC:
  ````shell
  chmod +x setup_pc.sh
  ./setup_pc.sh
  source ~/.bashrc
  ````
3. Verify the connection:
On robot:
  ```shell
  ros2 run demo_nodes_cpp talker
  ````
On PC (WSL2):
  ```shell
  ros2 run demo_nodes_cpp listener
  ````
If everything is correctly configured, the PC will receive messages from the robot.


## **2. PC connection to a PC-server connected to my_robot in a VPN Internet network**

With this guide we will show you how to configure your ROS system to work not only in LAN but also over the internet. We will have to create a Virtual Network (VPN)

We will scan 2 possibilities:
- Husarnet
- Zerotier


## **HUSARNET VPN**

Webgraphy:
- https://husarnet.com/docs/tutorial-ros2/
- https://husarnet.com/blog/ros2-docker

ROS 2 nodes can automatically discover each other when ROS 2 powered robots or computers are connected to the same Wi-Fi network. Doing the same over the internet is more challenging.

Follow the steps:
- On Raspberrypi:
  - Install Husarnet
  ````shell
  curl -s https://install.husarnet.com | sudo bash
  ````
  - Joint the raspberrypi to the Husarnet network
  ````shell
  husarnet join <CLAU_DE_LA_TEVA_XARXA>
  ````
  - verify the connection
  ````shell
  husarnet status
  ````
- On PC:
  - Install Husarnet: https://husarnet.com/docs/platform-windows-install
  - Joint the PC to the Husarnet network. Type on PowerShell as administrator
  ````shell
  husarnet join <CLAU_DE_LA_TEVA_XARXA>
  ````
  - verify the connection
  ````shell
  husarnet status
  ````
  - execute Container ROS2

  
## **2. ZEROTIER VPN**

Webgraphy:
- https://www.zerotier.com/
- https://www.youtube.com/watch?v=1BtkGzWkYE8

ROS 2 nodes