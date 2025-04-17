#The code is used to segment the ROI from pointcloud data and repositions it to a predefined target center.

#!/usr/bin/env python3
import open3d as o3d
import numpy as np
import rospy 
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

class Flexx2Segmentation:
    def __init__(self):
        rospy.init_node('flexx2_segmentation', anonymous=True)
        self.segmented_pub = rospy.Publisher('/royale_cam0/segmented_point_cloud', PointCloud2, queue_size=10)
        rospy.Subscriber("/royale_cam0/point_cloud", PointCloud2, self.segment_object)
        
        # Target center position
        self.target_center = np.array([0.3, 0, 0.3])

    def segment_object(self, data):
        # Convert ROS PointCloud2 to Open3D point cloud
        points = np.array(list(point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)))
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)

        # Process segmentation
        processed_cloud = self.process_cloud(cloud)

        # Transform to world frame if needed
        transformed_cloud = self.transform_to_world_frame(processed_cloud)

        # Get object details and move to target center
        self.get_object_details(transformed_cloud)
        moved_cloud = self.move_to_target_center(transformed_cloud)

        # Publish segmented point cloud
        self.publish_cloud(moved_cloud)

    def transform_to_world_frame(self, cloud):
        """Transform the point cloud to align with world frame coordinates"""
        # Create a rotation matrix (180° around X axis to flip Y and Z if needed)
        R = cloud.get_rotation_matrix_from_xyz((np.pi, 0, 0))  # Adjust these angles as needed
        cloud.rotate(R, center=(0, 0, 0))
        
        # You might also need a translation depending on your setup
        # cloud.translate([0, 0, 0])  # Adjust as needed
        
        rospy.loginfo("Applied transformation to world frame")
        return cloud

    def process_cloud(self, cloud):
        # Segment plane and get model
        plane_model, inliers = cloud.segment_plane(distance_threshold=0.03, ransac_n=3, num_iterations=1000)
        rospy.loginfo(f"\nPlane coefficients [a,b,c,d]: {plane_model}")
        filtered_cloud = cloud.select_by_index(inliers, invert=True)

        # Clustering
        labels = np.array(filtered_cloud.cluster_dbscan(eps=0.05, min_points=100, print_progress=False))
        if len(labels) == 0:
            rospy.logwarn("No clusters found!")
            return filtered_cloud

        largest_cluster_idx = np.argmax(np.bincount(labels[labels >= 0]))
        largest_cluster = filtered_cloud.select_by_index(np.where(labels == largest_cluster_idx)[0])
        
        return largest_cluster

    def get_object_details(self, cloud):
        try:
            # Calculate basic metrics
            points = np.asarray(cloud.points)
            if len(points) == 0:
                rospy.logwarn("Empty point cloud, cannot calculate metrics")
                return

            # Get bounding boxes
            aabb = cloud.get_axis_aligned_bounding_box()
            obb = cloud.get_oriented_bounding_box()
            
            # Calculate dimensions and metrics
            dimensions = aabb.get_extent()
            center = aabb.get_center()
            volume = dimensions[0] * dimensions[1] * dimensions[2]
            
            # Calculate point cloud density
            avg_distance = np.mean([np.linalg.norm(p - center) for p in points])
            density = len(points) / volume

            # Print comprehensive object details
            rospy.loginfo("\n=== Object Details ===")
            rospy.loginfo(f"Dimensions (width, height, depth): {dimensions} meters")
            rospy.loginfo(f"Current Center point (x,y,z): {center}")
            rospy.loginfo(f"Target Center point (x,y,z): {self.target_center}")
            rospy.loginfo(f"Volume: {volume:.6f} cubic meters")
            rospy.loginfo(f"Number of points: {len(points)}")
            rospy.loginfo(f"Point cloud density: {density:.2f} points/m³")
            rospy.loginfo(f"Average distance from center: {avg_distance:.4f} meters")
            
        except Exception as e:
            rospy.logerr(f"Error calculating object details: {str(e)}")

    def move_to_target_center(self, cloud):
        # Get current center
        aabb = cloud.get_axis_aligned_bounding_box()
        current_center = aabb.get_center()
        
        # Calculate translation vector
        translation = self.target_center - current_center
        
        # Translate the point cloud
        cloud.translate(translation)
        
        rospy.loginfo(f"\nPoint cloud moved by: {translation}")
        rospy.loginfo(f"New center: {np.add(current_center, translation)}")
        
        return cloud
    
    def publish_cloud(self, cloud):
        # Convert Open3D point cloud to numpy array
        points = np.asarray(cloud.points)
        
        # Create ROS message
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "world"  # Make sure this matches your TF tree
        
        # Create and publish the point cloud
        msg = point_cloud2.create_cloud_xyz32(header, points)
        self.segmented_pub.publish(msg)

if __name__ == '__main__':
    try:
        node = Flexx2Segmentation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
