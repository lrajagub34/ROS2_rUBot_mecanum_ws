# Stage 1: Build stage
FROM ros:humble-ros-core AS build

# Install basic dependencies including X11 client libraries
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-pip \
    wget \
    unzip \
    git \
    # X11 client libraries and related tools
    x11-apps \
    libx11-6 \
    libxext6 \
    libxrender1 \
    libxtst6 \
    libgl1-mesa-glx \
    # libglu1-mesa \ # Descomenta si és necessari per a aplicacions OpenGL més complexes
    # mesa-utils \   # Descomenta per a 'glxinfo', útil per a debugging
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 and other ROS dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-magic-enum \
    nlohmann-json3-dev \
    libusb-1.0-0-dev \
    libeigen3-dev \
    ros-humble-tf2-geometry-msgs \
    ros-humble-eigen3-cmake-module \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-rqt-robot-steering \
    ros-humble-cv-bridge \
    ros-humble-rviz2 \
    ros-humble-tf-transformations \
    ros-humble-turtlesim \
    ros-humble-vision-msgs \
    ros-humble-nav2-amcl \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-map-server \
    ros-humble-nav2-controller \
    ros-humble-nav2-planner \
    ros-humble-nav2-bt-navigator \
    ros-humble-nav2-waypoint-follower \
    ros-humble-cartographer-ros \
    ros-humble-cartographer \
    ros-humble-teleop-twist-keyboard \
    ros-humble-image-geometry \
    ros-humble-image-publisher \
    ros-humble-image-transport \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-msgs \
    ros-humble-tf2-sensor-msgs \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-usb-cam \
    libgoogle-glog0v5 \
    && rm -rf /var/lib/apt/lists/*

# Set up the working directory for ROS workspace
WORKDIR /root

# Copy and unzip the ROS2_rUBot_mecanum_ws.zip file
#COPY ROS2_rUBot_mecanum_ws.zip /root/ROS2_rUBot_mecanum_ws.zip
#RUN unzip /root/ROS2_rUBot_mecanum_ws.zip -d /root/ && \
#    rm /root/ROS2_rUBot_mecanum_ws.zip

# Clone the rplidar_ros package
#RUN git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git /root/ROS2_rUBot_mecanum_ws/src/rplidar_ros

# Build the ROS workspace
#RUN bash -c "source /opt/ros/humble/setup.bash && cd /root/ROS2_rUBot_mecanum_ws && colcon build"

# Add workspace, ROS environment setup, and QT_QPA_PLATFORM to .bashrc
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'export QT_QPA_PLATFORM=xcb' >> /root/.bashrc 
    #echo 'cd /root/ROS2_rUBot_mecanum_ws' >> /root/.bashrc && \
    #echo 'source /root/ROS2_rUBot_mecanum_ws/install/setup.bash' >> /root/.bashrc

# Copy the rubot_entrypoint.sh into the image
COPY rubot_entrypoint.sh /root/rubot_entrypoint.sh
RUN cd /root &&\
    chown root:root /root/rubot_entrypoint.sh &&\
    chmod 755 /root/rubot_entrypoint.sh

# Set permissions for rubot_nano_driver.py (assuming it's in your workspace)
# Make sure the path to rubot_nano_driver.py is correct
RUN if [ -f /root/ROS2_rUBot_mecanum_ws/src/Robot_drivers/my_robot_driver/my_robot_driver/rubot_nano_driver.py ]; then \
        chown root:root /root/ROS2_rUBot_mecanum_ws/src/Robot_drivers/my_robot_driver/my_robot_driver/rubot_nano_driver.py && \
        chmod 755 /root/ROS2_rUBot_mecanum_ws/src/Robot_drivers/my_robot_driver/my_robot_driver/rubot_nano_driver.py; \
    else \
        echo "Warning: rubot_nano_driver.py not found, skipping chmod/chown." ; \
    fi

# Set the default command to run when the container starts
# Consider using ENTRYPOINT if rubot_entrypoint.sh is meant to always run first
# ENTRYPOINT ["/root/rubot_entrypoint.sh"]
CMD ["bash"]