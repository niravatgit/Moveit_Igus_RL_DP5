# IGUS RLDP5

### Indigenous State-of-the-art Ultrasound Scanner for Maternal and Fetal Healthcare
This project represents a visionary endeavour geared towards crafting a revolutionary ultrasound scanner meticulously designed to cater to the distictive requisites and challenges faced within maternal and fetal healthcare among indigenous communities.
Out ambition is to amalgamatecutting-edge ultrasound imaging technology with a user-friendly interface, incorporating culturally sensitive attributes to amplify the accessibility and efficacy of prenatal care withing these populations.

## Project Contents

The follwing tree depicts the files  contained within this directory:
```
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
Note: The above represented tree of the directory  is not an actual representation, but rather a visualization of how the directories are structured in this, only important and updated files have been depicted and there are more files withing this directory.