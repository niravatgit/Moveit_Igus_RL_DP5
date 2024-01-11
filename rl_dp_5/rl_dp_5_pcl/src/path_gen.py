#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import tf.transformations as tf_trans

# Global publisher for markers
global marker_pub

def visualize_normals(tangent_vectors, point_array):
    """
    Visualizes normals as markers in Rviz.

    Args:
        tangent_vectors (numpy.ndarray): Array of tangent vectors representing normals.
        point_array (numpy.ndarray): Array of 3D points corresponding to the point cloud.

    Returns:
        None
    """
    marker_array = MarkerArray()

    for i in range(len(point_array)):
        normal_marker = Marker()
        normal_marker.header.frame_id = "royale_camera_link"
        normal_marker.header.stamp = rospy.Time.now()
        normal_marker.id = i
        normal_marker.type = Marker.SPHERE
        normal_marker.action = Marker.ADD

        position_np = point_array[i]
        normal_np = tangent_vectors[i]

        normal_marker.pose.position.x = position_np[0]
        normal_marker.pose.position.y = position_np[1]
        normal_marker.pose.position.z = position_np[2]

        # Check for zero-length tangent vectors before normalization
        if np.linalg.norm(normal_np) != 0:
            normal_np /= np.linalg.norm(normal_np)

            z_axis_local = -1 * normal_np
            y_axis_global = np.array([0, 1, 0]).astype(float)
            x_axis_local = np.cross(z_axis_local, y_axis_global).astype(float)
            y_axis_local = np.cross(z_axis_local, x_axis_local).astype(float)

            x_axis_local /= np.linalg.norm(x_axis_local)
            y_axis_local /= np.linalg.norm(y_axis_local)
            z_axis_local /= np.linalg.norm(z_axis_local)

            rotation_matrix = np.column_stack((x_axis_local, y_axis_local, z_axis_local))
            quaternion_data = tf_trans.quaternion_from_matrix(np.vstack([np.hstack([rotation_matrix, np.zeros((3, 1))]), [0, 0, 0, 1]]))

            quaternion_data /= np.linalg.norm(quaternion_data)

            normal_marker.pose.orientation.x = quaternion_data[0]
            normal_marker.pose.orientation.y = quaternion_data[1]
            normal_marker.pose.orientation.z = quaternion_data[2]
            normal_marker.pose.orientation.w = quaternion_data[3]
        else:
            # If the tangent vector is zero, set an arbitrary orientation
            normal_marker.pose.orientation.w = 1.0

        normal_marker.scale.x = 0.01
        normal_marker.scale.y = 0.01
        normal_marker.scale.z = 0.01
        normal_marker.color.a = 1.0
        normal_marker.color.r = 1.0
        normal_marker.color.g = 1.0
        normal_marker.color.b = 0.0

        marker_array.markers.append(normal_marker)

    marker_pub.publish(marker_array)

def compute_normals(points):
    """
    Computes tangent vectors and normalizes them to obtain normals.

    Args:
        points (numpy.ndarray): Array of 3D points.

    Returns:
        numpy.ndarray: Array of normalized tangent vectors representing normals.

    Description:
        - The function compute_normals takes an array of 3D points (points) as input.
        - 'tangent_vectors' is calculated by taking the difference between consecutive points. 
        - np.roll is used to shift the array by one position to get the differences. 
        - This is a numerical approximation of the derivative of the points, representing tangent vectors along the curve.
        - Special handling is applied to the first and last points to ensure that tangent vectors are computed properly.
    """
    # Compute tangent vectors
    tangent_vectors = np.roll(points, shift=-1, axis=0) - np.roll(points, shift=1, axis=0)
    tangent_vectors[0] = points[1] - points[0]
    tangent_vectors[-1] = points[-1] - points[-2]

    # Normalize tangent vectors to get normals
    for i in range(len(tangent_vectors)):
        if np.linalg.norm(tangent_vectors[i]) != 0:
            tangent_vectors[i] /= np.linalg.norm(tangent_vectors[i])

    return tangent_vectors

def point_cloud_callback(cloud_msg):
    """
    Callback function to process incoming point cloud data.

    Args:
        cloud_msg (sensor_msgs.msg.PointCloud2): The input point cloud message.
    """
    points = np.asarray(list(pc2.read_points(cloud_msg, skip_nans=True)))

    # Ensure the array is 2D
    if len(points.shape) == 1:
        points = points.reshape(-1, 3)

    # Compute tangent vectors
    tangent_vectors = compute_normals(points)

    print(f"Size of segmented point cloud: {points.shape}\nShape of normals computed: {tangent_vectors.shape}")

    visualize_normals(tangent_vectors, points)

def main():
    global marker_pub
    rospy.init_node('visualize_normals', anonymous=True)

    marker_pub = rospy.Publisher('/normals_markers', MarkerArray, queue_size=1)

    rospy.Subscriber('/royale_cam0/segmented_point_cloud', PointCloud2, point_cloud_callback)

    rospy.spin()

if __name__ == "__main__":
    main()
