# 🦾 RL DP5 Robot - Enhanced Pipeline

> ⚠️ All the following commands assume you're using updated scripts from:
`Moveit_Igus_RL_DP5/rl_dp_5/rl_dp_5_pcl/src/my_new_folder`

---

## 🔧 Compile the Project

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash

## 🧭 Launch Motion Planning in RViz
roslaunch rl_dp_5_moveit demo.launch

## 🏗️ Launch Robot in Gazebo + RViz
roslaunch rl_dp_5_moveit demo_gazebo.launch

## 🎥 Launch TOF Camera Driver
roslaunch royale_in_ros camera_driver.launch

**🧼 Segment the Point Cloud**
rosrun rl_dp_5_pcl pixel2pcl.py

🧠 Process Pre-segmented Point Cloud
rosrun rl_dp_5_pcl process_pointcloud.py

🦿 Run Motion Planning and Execution
rosrun rl_dp_5_pcl joint_trajectory.py


**compile the project:**
cd ~/catkin_ws
catkin_make
source devel/setup.bash

**Run the rviz launch file to setup Motion Planning and other topic:**
roslaunch rl_dp_5_moveit demo.launch

**If you want to see the robot in gazebo as well run the following command:**
roslaunch rl_dp_5_moveit demo_gazebo.launch

**You need to have the libroyale-5.9.0.2568-LINUX-x86-64Bit SDK and copy the sampleROS package in your src. Now you need to launch the camera_driver file:**
roslaunch royale_in_ros camera_driver.launch 

**Run the pixel2pcl node to segment the point cloud:**
rosrun rl_dp_5_pcl pixel2pcl.py 

**For preprocessed pointcloud, run the following:**
rosrun rl_dp_5_pcl process_pointcloud.py 

**For planning and execution, run:**
rosrun rl_dp_5_pcl joint_trajectory.py
