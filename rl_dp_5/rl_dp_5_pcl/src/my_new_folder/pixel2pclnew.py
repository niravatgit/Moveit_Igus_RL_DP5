#!/usr/bin/env python3
import open3d as o3d
import numpy as np
import rospy 
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
import tf.transformations

class RealRobotValidation:
    def __init__(self):
        rospy.init_node('real_robot_validation')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Marker position relative to robot base
        self.T_base_marker = np.array([
            [1, 0, 0, 0.17],
            [0, 1, 0, 0],
            [0, 0, 1, 0.01],
            [0, 0, 0, 1]
        ])

        # Initialize transformation matrix (will be filled in dynamically)
        self.T_base_camera = None

        # Publishers and Subscribers
        self.segmented_pub = rospy.Publisher('/royale_cam0/segmented_point_cloud', PointCloud2, queue_size=10)
        rospy.Subscriber("/royale_cam0/point_cloud", PointCloud2, self.process_pointcloud)

    def lookup_marker_transform(self):
        """Lookup transform from ArUco marker to camera"""
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame="royale_camera_0_optical_frame",  # camera frame
                source_frame="aruco_marker_1",                 # marker frame
                time=rospy.Time(0),
                timeout=rospy.Duration(1.0)
            )

            # Build 4x4 transformation matrix
            trans = transform.transform.translation
            rot = transform.transform.rotation

            translation = np.array([trans.x, trans.y, trans.z])
            quaternion = np.array([rot.x, rot.y, rot.z, rot.w])
            rotation_matrix = tf.transformations.quaternion_matrix(quaternion)[:3, :3]

            T_camera_marker = np.eye(4)
            T_camera_marker[:3, :3] = rotation_matrix
            T_camera_marker[:3, 3] = translation

            rospy.loginfo(f"\n=== T_camera_marker ===\n{T_camera_marker}")

            # Update T_base_camera
            T_marker_camera = np.linalg.inv(T_camera_marker)
            self.T_base_camera = np.matmul(self.T_base_marker, T_marker_camera)

            rospy.loginfo(f"\n=== T_base_camera ===\n{self.T_base_camera}")

            return True
        except Exception as e:
            rospy.logwarn(f"TF lookup failed: {e}")
            return False

    def transform_to_robot_frame(self, points):
        """Transform points from camera frame to robot base frame"""
        if self.T_base_camera is None:
            if not self.lookup_marker_transform():
                rospy.logwarn("Cannot transform points - no valid transform yet.")
                return None

        # Convert to homogeneous coordinates
        points_homog = np.hstack([points, np.ones((len(points), 1))])
        points_robot_homog = np.matmul(self.T_base_camera, points_homog.T)
        points_robot_homog = points_robot_homog[:3, :].T

        rospy.loginfo(f"\n=== Transformed Points (Robot Frame) ===\n{points_robot_homog}")

        return points_robot_homog

    def process_pointcloud(self, data):
        try:
            points = np.array(list(point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)))
            if len(points) == 0:
                rospy.logwarn("Empty point cloud received")
                return

            points_robot_frame = self.transform_to_robot_frame(points)
            if points_robot_frame is None:
                return

            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(points_robot_frame)

            # Segment plane
            plane_model, inliers = cloud.segment_plane(
                distance_threshold=0.03, ransac_n=3, num_iterations=1000
            )
            filtered_cloud = cloud.select_by_index(inliers, invert=True)

            # Cluster
            labels = np.array(filtered_cloud.cluster_dbscan(eps=0.05, min_points=100, print_progress=False))
            if len(labels) == 0:
                rospy.logwarn("No clusters found!")
                return

            largest_cluster_idx = np.argmax(np.bincount(labels[labels >= 0]))
            largest_cluster = filtered_cloud.select_by_index(np.where(labels == largest_cluster_idx)[0])

            self.publish_cloud(largest_cluster)
            self.get_object_details(largest_cluster)

        except Exception as e:
            rospy.logerr(f"Error processing point cloud: {str(e)}")

    def get_object_details(self, cloud):
        points = np.asarray(cloud.points)
        if len(points) == 0:
            return

        aabb = cloud.get_axis_aligned_bounding_box()
        center = aabb.get_center()
        dimensions = aabb.get_extent()

        rospy.loginfo("\n=== Object in Robot Frame ===")
        rospy.loginfo(f"Center (x,y,z): {center}")
        rospy.loginfo(f"Dimensions (w,h,d): {dimensions}")
        rospy.loginfo(f"Number of points: {len(points)}")

    def publish_cloud(self, cloud):
        points = np.asarray(cloud.points)
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "base_link"

        msg = point_cloud2.create_cloud_xyz32(header, points)
        self.segmented_pub.publish(msg)

if __name__ == '__main__':
    try:
        node = RealRobotValidation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

