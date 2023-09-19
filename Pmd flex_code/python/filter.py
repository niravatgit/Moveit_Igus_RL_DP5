#!/usr/bin/env python

import rospy
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

def point_cloud_callback(msg):
    # Convert the PointCloud2 message to a NumPy array
    pc_data = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    pc_array = np.array(list(pc_data))

    # Create an Open3D PointCloud from the NumPy array
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(pc_array)

    # Remove noise using StatisticalOutlierRemoval
    cl, ind = cloud.remove_statistical_outlier(nb_neighbors=50, std_ratio=1.0)
    cloud_filtered = cl

    # Downsample the point cloud using VoxelGrid
    downsampled_cloud = cloud_filtered.voxel_down_sample(voxel_size=0.01)

    # Convert the resulting Open3D point cloud back to a NumPy array
    downsampled_pc_array = np.asarray(downsampled_cloud.points)

    # Convert the NumPy array back to a PointCloud2 message
    downsampled_msg = point_cloud2.create_cloud_xyz32(msg.header, downsampled_pc_array)

    # Publish the filtered and downsampled point cloud
    pub.publish(downsampled_msg)

if __name__ == "__main__":
    rospy.init_node("point_cloud_processing")

    # Subscribe to the point cloud topic
    rospy.Subscriber("/translated_point_cloud", PointCloud2, point_cloud_callback)

    # Create a publisher for the filtered and downsampled point cloud
    pub = rospy.Publisher("/filtered_downsampled_point_cloud", PointCloud2, queue_size=10)

    rospy.spin()

