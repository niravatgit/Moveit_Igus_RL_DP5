#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
import open3d as o3d
import numpy as np

# Publisher for extracted waypoints
pub = None

def create_pointcloud(points):
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "world"  # Adjust frame as needed

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
    ]

    pointcloud = pc2.create_cloud(header, fields, points)
    return pointcloud

def extract_and_publish_waypoints():
    try:
        # Load the point cloud
        pcd = o3d.io.read_point_cloud("smoothed_pointcloud.ply")
        points = np.asarray(pcd.points)
        
        rospy.loginfo("Loaded point cloud with %d points", len(points))

        # Extract waypoints at specific indices
        required_indices = [437, 682, 697]
        if len(points) >= max(required_indices) + 1:
            waypoints = [tuple(points[i]) for i in required_indices]

            for i, waypoint in enumerate(waypoints):
                rospy.loginfo("Waypoint at index %d: (x: %f, y: %f, z: %f)",
                             required_indices[i], waypoint[0], waypoint[1], waypoint[2])

            # Create and publish point cloud of extracted waypoints
            waypoint_cloud = create_pointcloud(waypoints)
            pub.publish(waypoint_cloud)
            rospy.loginfo("Published waypoint point cloud")
        else:
            rospy.logwarn("Not enough points in the point cloud to extract all required waypoints.")

    except Exception as e:
        rospy.logerr("Error processing point cloud: %s", str(e))
        
if __name__ == "__main__":
    rospy.init_node("waypoint_extractor", anonymous=True)

    # Publisher for waypoints
    pub = rospy.Publisher("/extracted_waypoints", PointCloud2, queue_size=1)

    # Process the point cloud once at startup
    extract_and_publish_waypoints()

    rospy.loginfo("Waypoint extractor node started.")
    rospy.spin()
