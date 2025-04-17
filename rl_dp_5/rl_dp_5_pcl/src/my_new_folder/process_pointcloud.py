#!/usr/bin/env python3

import rospy
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import pcl_ros
import pcl

def filter_noise(cloud_array, nb_neighbors=30, std_ratio=1.5):
    """
    Remove statistical outliers from the point cloud.
    Args:
        cloud_array (np.ndarray): Input point cloud as a NumPy array.
        nb_neighbors (int): Number of neighbors to analyze for each point.
        std_ratio (float): Standard deviation multiplier for filtering.
    Returns:
        np.ndarray: Filtered point cloud as a NumPy array.
    """
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(cloud_array)
    filtered_cloud, _ = cloud.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    return np.asarray(filtered_cloud.points)

def downsample_pointcloud(cloud_array, leaf_size=0.01):
    """
    Downsample the point cloud using a voxel grid filter.
    Args:
        cloud_array (np.ndarray): Input point cloud as a NumPy array.
        leaf_size (float): Voxel grid leaf size.
    Returns:
        np.ndarray: Downsampled point cloud as a NumPy array.
    """
    pcl_cloud = pcl.PointCloud()
    pcl_cloud.from_array(np.array(cloud_array, dtype=np.float32))
    voxel = pcl_cloud.make_voxel_grid_filter()
    voxel.set_leaf_size(leaf_size, leaf_size, leaf_size)
    downsampled = voxel.filter()
    return np.asarray(downsampled.to_array())

def process_pointcloud(msg):
    try:
        # Convert the PointCloud2 to a list of XYZ points
        points = np.array([p[:3] for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)])

        # Log original points
        if len(points) > 0:
            rospy.loginfo("Original point (frame: %s): [%f, %f, %f]", 
                         msg.header.frame_id, points[0][0], points[0][1], points[0][2])

        # Apply noise filtering
        filtered_points = filter_noise(points)

        # Downsample the filtered point cloud
        downsampled_points = downsample_pointcloud(filtered_points)

        # Log processed points
        if len(downsampled_points) > 0:
            rospy.loginfo("Processed point (frame: %s): [%f, %f, %f]", 
                         msg.header.frame_id, downsampled_points[0][0], 
                         downsampled_points[0][1], downsampled_points[0][2])

        # Convert filtered points back to ROS PointCloud2 format
        filtered_msg = pc2.create_cloud_xyz32(msg.header, downsampled_points)

        # Publish the processed point cloud
        pub_smoothed.publish(filtered_msg)
        rospy.loginfo("Processed point cloud published with %d points.", len(downsampled_points))

    except Exception as e:
        rospy.logerr("Error processing point cloud: %s", str(e))

if __name__ == "__main__":
    rospy.init_node("pointcloud_processor", anonymous=True)

    pub_smoothed = rospy.Publisher("/smoothed_pointcloud", PointCloud2, queue_size=10)
    sub = rospy.Subscriber("/royale_cam0/segmented_point_cloud", PointCloud2, process_pointcloud)

    rospy.loginfo("PointCloud processing node started.")
    rospy.spin()
