# Project based on Object recognition

We will describe the Computer Vision based method to identify the Traffic Sign

## ROS packages installation

The needed packages installation instructions:
````shell
sudo apt update
sudo apt install python3-pip
sudo apt install python3-opencv
pip3 install numpy matplotlib
pip3 install keras tensorflow
sudo apt install ros-humble-cv-bridge
````
We will use Keras that is a high-level API that runs on top of TensorFlow. By using both TensorFlow and Keras, you get the best of both worlds: the ease of use and simplicity of Keras, combined with the power and flexibility of TensorFlow. 

We will create a new "my_robot_AI_identification" package with:
````shell
ros2 pkg create --build-type ament_python my_robot_AI_identification --dependencies rclpy std_msgs sensor_msgs geometry_msgs nav_msgs nav2_simple_commander tf_transformations cv_bridge
cd ..
colcon build
````

## Getting started

1. Bringup the rUBot: This is done when robot turn-on
   
2. Place the rUBot in front of the Traffic Signal

3. Take photos from USB camera
    ````bash
    ros2 run my_robot_AI_identification take_photo_exec
    ````
    You can use a more performand program to take photos continuously, add data in picture name for model generation and also to detect traffic signs when model is created.
    ````bash
    ros2 run my_robot_AI_identification takePhoto_detectSign_keras_exec
    ````
4. Open RVIZ to see the picture frame
    ````bash
    rviz
    ````
- Open the "teachablemachine" app to create a model for "Traffic Signs". Go to https://teachablemachine.withgoogle.com/, select "Get Started", "Image Project", "New Image Project", "Standard Image Model" and create an image project.  
    ![](./Images/06_Keras/01_Teachable_machine1.png)

- Collect images with rUBot USB_CAM for each sign and upload them to the project.  
- Train the model.  
- Export the model as a keras .h5 model. The model can be created with some pictures, but this would be improved with some more pictures.  
- The models will be uploaded in "models" folder  

8. Verify topics (theConstruct):
   ```bash
   ros2 topic list
   ````
   Verify the topic name: /usb_cam/image_raw to place it in "takePhoto_detectSign_keras.py" file
   
9. Launch classification node and take photos (theConstruct):
   ```bash
   ros2 launch my_robot_AI_identification takePhoto_detectSign_keras.launch.py
   ````
10.1. Stop doing photos (theConstruct) (in a new Terminal):
   ```bash
   ros2 topic pub /capture_toggle std_msgs/Bool "data: false"
   ````
   If you want to do photos again set false value to true 
   ```bash
   ros2 topic pub /capture_toggle std_msgs/Bool "data: true"
   ````

### Final Exercise:

#### Navigate to a target point taking care of traffic signals 

* Load the room map 
* Start the navigation stack. 
* Get the coordinates of all the traffic sign using Rviz.
* Write a Python node with the folowing behaviour:
	* Go to the target point. Nav2 finds an optimat trajectory and starts to move
    * robot takes photos and identifies if there is a traffic sign
    * if robot finds a traffic sign, it obey the traffic signal
        * Left: turn left
        * Right: turn right
        * STOP: stops
        * Forbidden: turn 180º  
    * the nav2 package finds a new optimal trajectory to reach the target point

