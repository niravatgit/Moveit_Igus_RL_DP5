#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import numpy as np

# Define the region of interest (ROI) limits
ROI_X_MIN = -0.2  # Minimum x-coordinate of the ROI
ROI_X_MAX = 0.2   # Maximum x-coordinate of the ROI
ROI_Y_MIN = -0.2  # Minimum y-coordinate of the ROI
ROI_Y_MAX = 0.2   # Maximum y-coordinate of the ROI
ROI_Z_MIN = 0.2   # Minimum z-coordinate of the ROI
ROI_Z_MAX = 1.5   # Maximum z-coordinate of the ROI

# Reference point coordinates for distance calculation
REFERENCE_X = 0.0
REFERENCE_Y = 0.0
REFERENCE_Z = 0.0

# Maximum distance from the reference point for filtering
MAX_DISTANCE = 1.2

# Rotation angle in degrees
ROTATION_ANGLE_X = 180.0  # Rotate 180 degrees along X-axis

# Translation values (adjust as needed)
TRANSLATION_X = 0.0
TRANSLATION_Y = 0.0
TRANSLATION_Z = 1.2

def calculate_distance(x, y, z):
    return np.sqrt((x - REFERENCE_X)**2 + (y - REFERENCE_Y)**2 + (z - REFERENCE_Z)**2)

def publish_point_cloud(filtered_points, pc_publisher):
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "royale_camera_link"  # Set the appropriate frame ID

    pc_data = pc2.create_cloud_xyz32(header, filtered_points)
    pc_publisher.publish(pc_data)

def rotate_point_cloud(point_cloud, angle_degrees):
    # Convert the angle to radians
    angle_rad = np.radians(angle_degrees)

    # Rotation matrix along the X-axis
    rotation_matrix = np.array([[1, 0, 0],
                                 [0, np.cos(angle_rad), -np.sin(angle_rad)],
                                 [0, np.sin(angle_rad), np.cos(angle_rad)]])

    # Apply the rotation to each point in the point cloud
    rotated_points = [np.dot(rotation_matrix, point) for point in point_cloud]

    return rotated_points

def translate_point_cloud(point_cloud, translation):
    translated_points = [point + translation for point in point_cloud]
    return translated_points

def point_cloud_callback(msg, pc_publisher):  # Pass pc_publisher as an argument
    # Extract point cloud data
    pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

    # Extract x, y, z coordinates from point cloud
    x = []
    y = []
    z = []
    for point in pc_data:
        x.append(point[0])
        y.append(point[1])
        z.append(point[2])

    # Filter points within the region of interest (ROI)
    filtered_points = [(x_val, y_val, z_val) for x_val, y_val, z_val in zip(x, y, z)
                       if ROI_X_MIN <= x_val <= ROI_X_MAX and
                       ROI_Y_MIN <= y_val <= ROI_Y_MAX and
                       ROI_Z_MIN <= z_val <= ROI_Z_MAX]

    # Calculate distances from reference point
    distances = [calculate_distance(x_val, y_val, z_val) for x_val, y_val, z_val in filtered_points]

    # Filter points within the desired distance range
    filtered_points = [point for point, dist in zip(filtered_points, distances) if dist <= MAX_DISTANCE]

    # Rotate the filtered point cloud by 180 degrees along X-axis
    rotated_points = rotate_point_cloud(filtered_points, ROTATION_ANGLE_X)

    # Translate the rotated point cloud
    translation_vector = [TRANSLATION_X, TRANSLATION_Y, TRANSLATION_Z]
    translated_points = translate_point_cloud(rotated_points, translation_vector)

    # Publish the translated point cloud for RViz visualization
    publish_point_cloud(translated_points, pc_publisher)

def main():
    # Initialize ROS node
    rospy.init_node('point_cloud_subscriber', anonymous=True)

    # Create a publisher for the translated point cloud
    pc_publisher = rospy.Publisher("/translated_point_cloud", PointCloud2, queue_size=10)

    # Subscribe to point cloud topic and pass pc_publisher as an argument
    rospy.Subscriber("/royale_cam0/point_cloud", PointCloud2, point_cloud_callback, pc_publisher)

    # Keep the program running until it's terminated
    rospy.spin()

if __name__ == "__main__":
    main()

