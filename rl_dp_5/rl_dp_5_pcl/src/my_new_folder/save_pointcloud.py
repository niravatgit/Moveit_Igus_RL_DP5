#!/usr/bin/env python3

import rospy
import open3d as o3d
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

def pointcloud_callback(msg):
    # Convert PointCloud2 message to Open3D format
    points = list(point_cloud2.read_points(msg, skip_nans=True))
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # Save the point cloud to a file
    output_file = "smoothed_pointcloud.ply"
    o3d.io.write_point_cloud(output_file, pcd)
    rospy.loginfo(f"Saved point cloud to {output_file}")

    # Shutdown the node after saving
    rospy.signal_shutdown("Point cloud saved")

def main():
    rospy.init_node("save_pointcloud_node")
    rospy.Subscriber("/smoothed_pointcloud", PointCloud2, pointcloud_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
