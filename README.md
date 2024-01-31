# IGUS RLDP5

## Indigenous State-of-the-art Ultrasound Scanner for Maternal and Fetal Healthcare

### Description:
This project represents a visionary endeavour geared towards crafting a revolutionary ultrasound scanner meticulously designed to cater to the distictive requisites and challenges faced within maternal and fetal healthcare among indigenous communities. Our ambition is to amalgamatecutting-edge ultrasound imaging technology with a user-friendly interface, incorporating culturally sensitive attributes to amplify the accessibility and efficacy of prenatal care withing these populations.

### Project Contents:

The follwing tree depicts the files  contained within this directory:
```
Moveit_Igus_RL_DP5
├── documents
├── igus_examples
├── pmd flex2
├── realsense2_description
├── realsense_gazebo_plugin
└── rl_dp_5
    ├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
    ├── dryve_config # Contains the configuration files of each motor
    ├── dryve_control # Contains the python scripts  used to control the motors
    │   ├── ac.py # Action client code that takes inputs from the command line
    │   ├── rl_dp_5_moveit_control.py # Python script that actuates the robot using interactive marker from RViz
    │   ├── rl_dp_5_robot_con_GUI.py # Python script that actuates the robot using GUI
    │   ├── rl_dp_5_robot_con_ROS_GUI.py # Python script that actuates the robot using ROS and have a GUI
    │   ├── rl_dp_5_robot_con_ROS.py # Python script that actuates the robot using ROS and is an action server
    ├── rldp5_description
    ├── rl_dp_5_moveit
    ├── rldp5_msgs
    │   ├── action # Contains the custom action messages for the robot
    │   │   ├── home.action
    │   │   ├── home_all.action
    │   │   ├── rldp5_robot.action
    │   │   └── set_des_pos.action
    └── rl_dp_5_pcl # Custom ros package for the robot integrated with PCL
        └── src
            ├── normal2cartesianpath.py # Computes normals of the segmented point cloud and plans a Cartesian Path
            ├── pixel2pcl.py # Converts image into point cloud data and segment specific region of interest
            ├── test_code.cpp # C++ code using PCL to calculate surface normals (not used at this point)
```
### Note: 
The above represented tree of the directory  is not an actual representation, but rather a visualization of how the directories are structured in this, only important and updated files have been depicted and there are more files withing this directory.

### Using the Repo:


Clone the project in the src folder of your project
```
cd path_to_your_ws/src

git clone https://github.com/niravatgit/Moveit_Igus_RL_DP5.git
```

Compile the project
```
catkin build

source devel/setup.bash
```

### Actuation of the robot:

- Follow the safety instructions and always keep you hand on the emergency stop button all the time

- Controlling the robot only with GUI

  ```
  cd path_to_ws/src/rl_dp_5/dryve_control/
  python3 rl_dp_5_robot_con_GUI.py
  ```
- Controlling the robot with ROS and GUI

  ```
  cd path_to_ws/src/rl_dp_5/dryve_control/
  python3 rl_dp_5_robot_con_ROS_GUI.py
  ```
- Controlling the robot with moveit interface

  - If you want to visualize the robot only in RViz
    ```
    roslaunch rl_dp_5_moveit demo.launch
    ```

  - If you want to visualize the robot in RViz and Gazebo
    ```
    roslaunch rl_dp_5_moveit demo_gazebo.launch
    ```

  In a new terminal run the following command
  ```
  cd path_to_ws/src/rl_dp_5/dryve_control/
  python3 rl_dp_5_moveit_control.py
  ```
- Contol the robot with action commands:
  
  ```
  cd path_to_ws/src/rl_dp_5/dryve_control/
  python3 rl_dp_5_robot_con_ROS.py
  ```

  In another terminal run the following command to use the action command

  ```
  rostopic pub 
  ```

### Camera and Simulation

- Launch either RViz or Gazebo as the RViz configuration file is pre-set for visualizing the TOF sensor data with the Royale Control panel, Image,    PointCloud2 types added. The rviz file is located in rl_dp_5_moveit/rviz/scene.rviz

- You need to have the libroyale-5.9.0.2568-LINUX-x86-64Bit SDK and copy the sampleROS package in your src.
  Now you need to launch the camera_driver file
  ```
  roslaunch royale_in_ros camera_driver.launch 
  ```
- Run the pixel2pcl node to segment the point cloud
  ```
  rosrun rl_dp_5_pcl pixel2pcl.py 
  ```
- Run the compute_normals node to compute the normals and visualize them as Marker Array
  ```
  rosrun rl_dp_5_pcl normals2cartesianpath.py
  ```