# **Lab Activity: Lidar test**
Marc Font, Laia Raja and Alba Navarro


### **1. Lidar specifications in the .URDF file**
The Lidar specifications in the URDF file `rubot_mecanum_lidar.urdf` are in the *Laser Distance Sensor YDLIDAR X4 controller* section. Our lidar sensor is a RPLIDAR A1 with the following specifications:
- angle_min: -3.141593 (rad)
- angle_max: 3.141593 (rad)
- angle_increment: 0.008727 (rad)  -> 720 laser beams

In this section we changed angle_min from 0 rad to $-\pi$ rad (-3.141593 rad) and angle_max from $-2\pi$ rad to $\pi$ rad (3.141593 rad).
The angle_increment is defined by the number of laser beams which is set to 760 samples. We didn't change this parameter in the urdf file.

![](./Images/00_Activities/img_lidar_urdf.png)



### **2. Create the new launch and the new Python file**
We created the following new files to verify proper Lidar readings:
- New launch file name: `custom_rubot_lidar_test.launch.xml`
- New python file name: `custom_rubot_lidar_test.py`

The launch file is: 
![](./Images/00_Activities/img_lidar_launch.png)


We had to update the `setup.py` file in `my_robot_control` by adding:
    ```shell
    'custom_rubot_lidar_test_exec = my_robot_control.custom_rubot_lidar_test:main'
    ````

The `custom_rubot_lidar_test.py` is the same file as `my_robot_lidar_test.py` with these changes:
- Change the line 31 from `index_0_deg = int((0 - angle_min_deg -180)/ angle_increment_deg)` removing *-180* to `index_0_deg = int((0 - angle_min_deg)/ angle_increment_deg)` because we now defined the angle_min as $-\pi$
- The original version converted angles into the range -180º to 180º because angle_min was 0º and angle_max was 360º. We do not need it in the new code because angle_min is already -180º and angle_max is 180º.
- The original code filtered only the frontal angles from -150º to 150º but in the new version we use the full angle range.

![](./Images/00_Activities/img_lidar_py.png)



### 3. Launch and verify
We launch the bringup with the new `rubot_mecanum_lidar.urdf` to see robot the Gazebo:
    ```shell
    ros2 launch my_robot_bringup my_robot_bringup_sw.launch.xml x0:=1.0 y0:=-0.5 yaw0:=0.0 rubot:=rubot/rubot_mecanum_lidar.urdf custom_world:=square3m_walls.world
    ````
Here is the Gazebo bringup where we can see the robot in x=1, y=-0.5 i yaw=0
![](./Images/00_Activities/img_lidar_py.png)


When we launch the new file `custom_rubot_lidar_test.launch.xml`
    ````shell
    ros2 launch my_robot_control custom_rubot_lidar_test.launch.xml 
    ````


CAL RESPONDRE AQUESTES PREGUNTES UN COP FEM EL LAUNCH:

    - Are the Lidar readings correct?
    - what do you think it could hapen?

- Launch the `my_robot_lidar_test_rUBot.launch.xml` file and show:
    - The minimum distance and angle to a wall detected by the Lidar
    - The distances at 0º, 90º and -90º with respect to the robot front


- terminal running the `my_robot_lidar_test_rUBot.launch.xml` with distances readings
