#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3, Point
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
from tf.transformations import quaternion_from_euler, quaternion_matrix, quaternion_from_matrix, quaternion_multiply
from tf2_ros import TransformException
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

# Global variables for marker publishing
marker_pub = None

def compute_normals(point_cloud, waypoints, radius=0.05):
    rospy.loginfo("Starting normal computation...")
    
    # Convert ROS PointCloud2 to Open3D format
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=50))
    
    if not pcd.has_normals():
        rospy.logerr("Normal estimation failed!")
        return None, None

    points_array = np.asarray(pcd.points)
    normals_array = np.asarray(pcd.normals)
    rospy.loginfo("Number of normals: %d", len(normals_array))
    
    transformed_waypoints = []
    transformed_normals = []
    
    hardcoded_waypoint = np.array([0.291460, 0.094032, 0.413628])

    for i, waypoint in enumerate(waypoints):
        # Find closest point and normal in world frame
        distances = np.linalg.norm(points_array - waypoint, axis=1)
        closest_idx = np.argmin(distances)
        normal = normals_array[closest_idx]
        normal /= np.linalg.norm(normal)

        transformed_waypoints.append(waypoint)
        transformed_normals.append(normal)

        # Check if the current waypoint is close to the hardcoded waypoint
        distance_to_hardcoded = np.linalg.norm(waypoint - hardcoded_waypoint)
        if distance_to_hardcoded < 0.01:  # Tolerance for floating-point matching
            rospy.loginfo(f"Closest normal for hardcoded waypoint {waypoint}: {normal}")
            if i + 1 < len(waypoints):
                rospy.loginfo(f"Normal for consecutive waypoint {waypoints[i + 1]}: {transformed_normals[i + 1]}")
            if i + 6 < len(waypoints):
                rospy.loginfo(f"Normal for 6th waypoint from hardcoded {waypoints[i + 6]}: {transformed_normals[i + 6]}")

    rospy.loginfo("Number of transformed waypoints: %d", len(transformed_waypoints))
    rospy.loginfo("Number of transformed normals: %d", len(transformed_normals))
    
    # Print the first 10 waypoints and their normals
    rospy.loginfo("First 10 waypoints and their normals:")
    for i in range(min(10, len(transformed_waypoints))):
        rospy.loginfo(f"Waypoint {i}: {transformed_waypoints[i]}, Normal: {transformed_normals[i]}")

    # Derive orientation in quaternion format for the first 10 waypoints
    rospy.loginfo("Orientations in quaternion format for the first 10 waypoints:")
    for i in range(min(10, len(transformed_waypoints))):
        normal = transformed_normals[i]
        # Assuming the z-axis should align with the normal
        # Calculate the rotation needed to align the z-axis with the normal
        z_axis = np.array([0, 0, 1])
        rotation_axis = np.cross(z_axis, normal)
        rotation_axis /= np.linalg.norm(rotation_axis)
        rotation_angle = np.arccos(np.dot(z_axis, normal))
        
        # Convert the rotation axis and angle to a quaternion
        quaternion = quaternion_from_euler(rotation_axis[0] * rotation_angle,
                                           rotation_axis[1] * rotation_angle,
                                           rotation_axis[2] * rotation_angle)
        
        rospy.loginfo(f"Waypoint {i}: {transformed_waypoints[i]}, Quaternion: {quaternion}")

    return transformed_waypoints, transformed_normals
    
def publish_normals_rviz(waypoints, normals):
    global marker_pub
    if marker_pub is None or len(waypoints) == 0:
        return

    marker_array = MarkerArray()
    for i, (wp, normal) in enumerate(zip(waypoints, normals)):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "normals"
        marker.id = i
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.01  # Shaft diameter
        marker.scale.y = 0.02  # Head diameter
        marker.scale.z = 0.0   # Unused for arrows
        marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Green
        
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Arrow points
        start = Point(*wp)
        end = Point(*(wp + normal * 0.1))  # Scale normal for visibility
        marker.points = [start, end]

        marker_array.markers.append(marker)

    marker_pub.publish(marker_array)
    rospy.loginfo("Published %d normals", len(waypoints))

def pointcloud_callback(msg):
    try:
        # Extract points in world frame
        points = np.array([p[:3] for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)])
        rospy.loginfo("Number of points: %d", len(points))
        if len(points) == 0:
            return

        # Process all waypoints at once
        waypoints = points  # Using all points as waypoints for demonstration
        transformed_waypoints, transformed_normals = compute_normals(points, waypoints)
        
        if transformed_waypoints is None:
            return

        publish_normals_rviz(transformed_waypoints, transformed_normals)

        # Find the waypoint closest to the specified position
        target_position = np.array([0.291460, 0.094032, 0.413628])
        distances = np.linalg.norm(transformed_waypoints - target_position, axis=1)
        closest_idx = np.argmin(distances)
        closest_waypoint = transformed_waypoints[closest_idx]
        closest_normal = transformed_normals[closest_idx]

        rospy.loginfo(f"Closest waypoint to {target_position}: {closest_waypoint}")
        rospy.loginfo(f"Normal at closest waypoint: {closest_normal}")

        # Print the normal of the line [0, 0, 1]
        line_normal = np.array([0, 0, 1])
        rospy.loginfo(f"Normal of the line [0, 0, 1]: {line_normal}")

    except Exception as e:
        rospy.logerr("Pointcloud processing error: %s", str(e))

def main():
    global marker_pub
    rospy.init_node("normal_computation")
    marker_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=10)
    rospy.Subscriber("/smoothed_pointcloud", PointCloud2, pointcloud_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
