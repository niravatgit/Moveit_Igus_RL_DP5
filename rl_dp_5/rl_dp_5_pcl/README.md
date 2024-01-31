# rl_dp_5_pcl

## PCL ROS Package for the RLDP5

This ROS package provides support for the PMD Flexx2, Time-of-Flight (TOF) sensor stuck to a desk. The package includes files for interfacing with the TOF sensor, processing point cloud data, and visualizing the results.
(This package is a part of a huge project with other packages)

### Package Contents:
1. Nodes:

    - pixel2pcl.py: ROS node responsible for segmenting the point cloud from the Original point cloud. The point cloud is segmented by subscribing to /roayle_cam0/point_cloud and /royale_cam0/gray_image. Using OpenCV the 4 points near black markers are taken and the point cloud is segmented.
    - normals2cartesianpath.py: Node is responsible to calculate the normals and visualize in RViz as MarkerArray. To compute the normals, Open3D module is used. 

2. Launch Files:

    - camera_driver.launch: Launch file to start the TOF sensor node and configure its parameters.
    - demo.launch: Launch file to use the Motion Planning node

3. Rviz Config:

    - scene.rviz: RViz configuration file pre-set for visualizing the TOF sensor data with the Royale Control panel, Image, PointCloud2 types added. The rviz file is located in rl_dp_5_moveit/rviz/scene.rviz


### Using the Package:

Clone the package in the src folder of your project
```
git clone https://github.com/KarthikMothiki/rl_dp_5_pcl.git
```

Compile the project
```
catkin build
source devel/setup.bash
```
Run the rviz launch file to setup Motion Planning and other topic
```
roslaunch rl_dp_5_moveit demo.launch
```
If you want to see the robot in gazebo as well run the following command
```
roslaunch rl_dp_5_moveit demo_gazebo.launch
```

You need to have the libroyale-5.9.0.2568-LINUX-x86-64Bit SDK and copy the sampleROS package in your src.
Now you need to launch the camera_driver file
```
roslaunch royale_in_ros camera_driver.launch 
```
![pointcloud](https://github.com/KarthikMothiki/rl_dp_5_pcl/assets/62557178/57266682-f4e9-4f57-a79b-2b46bb6b27db)

Run the pixel2pcl node to segment the point cloud
```
rosrun rl_dp_5_pcl pixel2pcl.py 
```
![Segmented PC compared to Original PC](https://github.com/KarthikMothiki/rl_dp_5_pcl/assets/62557178/dde89c69-0962-424e-998b-5da50216b119)

### Point Cloud post segmentation of Region of Interest
![Screenshot from 2024-01-30 12-31-57](https://github.com/KarthikMothiki/rl_dp_5_pcl/assets/62557178/85f9dae8-2b2b-4a40-9d5b-4e8264d452f6)

Run the compute_normals node to compute the normals and visualize them as Marker Array
```
rosrun rl_dp_5_pcl normals2cartesianpath.py
```
![Screenshot from 2024-01-30 12-34-53](https://github.com/KarthikMothiki/rl_dp_5_pcl/assets/62557178/05470dbc-c32f-4139-8150-58d0597a4d61)



