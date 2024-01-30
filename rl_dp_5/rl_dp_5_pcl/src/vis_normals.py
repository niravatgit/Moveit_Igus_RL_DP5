#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from tf.transformations import quaternion_from_matrix
from tf import transformations as tf_trans

class DataProcessor():
    def __init__(self):
        self.points = None
        self.normals = None

    def nor_callback(self, data):
        points = np.asarray(list(pc2.read_points(data, skip_nans=True)))
        self.normals = points[:, :3]
        self.visualize_normals()

    def pc_callback(self, cloud_msg):
        points = np.asarray(list(pc2.read_points(cloud_msg, skip_nans=True)))
        self.points = points

    def quaternion_from_normal(self, normal):
        # Assuming that the initial orientation is pointing upwards (e.g., [0, 0, 1])
        up_vector = np.array([0.0, 1.0, 0.0])
        
        # Compute the rotation axis using cross product
        axis = np.cross(up_vector, normal)
        
        # Compute the rotation angle using dot product
        angle = np.arccos(np.dot(up_vector, normal))

        # Create a quaternion from the axis and angle using tf
        # quaternion = tf_trans.quaternion_about_axis(angle, axis)
        quaternion = quaternion_from_matrix([[np.cos(angle/2), -np.sin(angle/2)*axis[0], -np.sin(angle/2)*axis[1], -np.sin(angle/2)*axis[2]],
                                            [np.sin(angle/2)*axis[0], np.cos(angle/2), 0, 0],
                                            [np.sin(angle/2)*axis[1], 0, np.cos(angle/2), 0],
                                            [np.sin(angle/2)*axis[2], 0, 0, np.cos(angle/2)]])

        quaternion /= np.linalg.norm(quaternion)
        return quaternion

    def visualize_normals(self):
        if self.points is not None and self.normals is not None:
            # Determine the minimum length between points and normals
            min_length = min(len(self.points), len(self.normals))

            # Create a MarkerArray to store arrows
            marker_array = MarkerArray()
            
            for i in range(min_length):
                point = self.points[i]
                normal = self.normals[i]
                
                # Compute quaternion from the normal vector
                quaternion = self.quaternion_from_normal(normal)
                
                # Create a marker for each normal arrow
                marker = Marker()
                marker.header.frame_id = "royale_camera_0_optical_frame"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "normals"
                marker.id = i
                marker.type = Marker.ARROW
                marker.action = Marker.ADD

                marker.pose.position.x = point[0]
                marker.pose.position.y = point[1]
                marker.pose.position.z = point[2]
                
                # Set the orientation using the computed quaternion
                marker.pose.orientation.x = quaternion[0]
                marker.pose.orientation.y = quaternion[1]
                marker.pose.orientation.z = quaternion[2]
                marker.pose.orientation.w = quaternion[3]

                marker.scale.x = 0.01  # Arrow shaft diameter
                marker.scale.y = 0.001  # Arrow head diameter
                marker.scale.z = 0.001  # Arrow head length
                marker.color.a = 1.0  # Alpha (transparency)
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0

                marker_array.markers.append(marker)

            # Publish the MarkerArray
            marker_pub.publish(marker_array)

def listener():
    rospy.init_node('visualize_normals', anonymous=True)

    data_processor = DataProcessor()

    rospy.Subscriber('/royale_cam0/segmented_point_cloud', PointCloud2, data_processor.pc_callback)

    rospy.Subscriber('/cloud_normals_something', PointCloud2, data_processor.nor_callback)

    rospy.spin()

if __name__ == '__main__':
    marker_pub = rospy.Publisher('/normals_markers', MarkerArray, queue_size=10)
    listener()
