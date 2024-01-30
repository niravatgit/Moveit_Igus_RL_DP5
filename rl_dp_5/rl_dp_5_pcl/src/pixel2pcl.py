#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from matplotlib import pyplot as plt

# Initialize OpenCV bridge
bridge = CvBridge()

# Store detected corners
sorted_points = []

# Global variable for depth data
depth_data = None

# Global variable for storing depths
current_depths = None

# Callback function for the image subscriber
def image_callback(msg):
    """
    Callback function for processing image messages.

    Args:
        msg (sensor_msgs.msg.Image): The input image message.
    """
    global sorted_points
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="mono16")
        img = cv2.convertScaleAbs(cv_image)

        # Uncomment the following lines to get corner points from the image
        # cv2.imshow('Image', img)
        # cv2.waitKey(1)

        # Hardcoded example of sorted corner points
        sorted_points = [[106, 68], [136, 68], [106, 98], [136, 98]]
    
    except Exception as e:
        print(e)

# Callback function for the depth subscriber
def depth_callback(msg):
    """
    Callback function for processing depth messages.

    Args:
        msg (sensor_msgs.msg.Image): The input depth image message.
    """
    global current_depths
    depth_data = np.array(bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough"))
    if depth_data is not None and sorted_points is not None:
        try:
            current_depths = get_depths(depth_data, sorted_points)
        except Exception as e:
            print(f"Error in depth_callback: {e}")

def get_depths(depth_data, pix):
    """
    Extract depths from depth data based on given corner points.

    Args:
        depth_data (numpy.ndarray): The depth data array.
        pix (List[List[int]]): List of corner points in the format [[x1, y1], [x2, y2], ...].

    Returns:
        numpy.ndarray: Extracted depths corresponding to the specified corner points.
    """
    try:
        if not pix or len(pix) < 2:
            return None

        x_coords, y_coords = zip(*pix)
        x_min, x_max = min(x_coords), max(x_coords)
        y_min, y_max = min(y_coords), max(y_coords)

        depth_data = np.asarray(depth_data)

        if y_max >= depth_data.shape[0] or x_max >= depth_data.shape[1]:
            print("Error: Corner points exceed image dimensions")
            return None
        depths = depth_data[y_min:y_max + 1, x_min:x_max + 1]

        return depths
    
    except ValueError as e:
        print(f"Error extracting depths: {e}")
        return None

# Callback function for the point cloud subscriber
def point_cloud_callback(msg):
    """
    Callback function for processing point cloud messages.

    Args:
        msg (sensor_msgs.msg.PointCloud2): The input point cloud message.
    """
    try:
        if msg is not None:
            segmented_point_cloud = extract_roi_from_point_cloud(msg, sorted_points)
            publish_segmented_point_cloud(segmented_point_cloud)
            # publish_segmented_point_cloud(msg)

    except Exception as e:
        print(e)

def extract_roi_from_point_cloud(msg, pix):
    """
    Extract a region of interest (ROI) from the point cloud based on given corner points.

    Args:
        msg (sensor_msgs.msg.PointCloud2): The input point cloud message.
        pix (List[List[int]]): List of corner points in the format [[x1, y1], [x2, y2], ...].

    Returns:
        numpy.ndarray: Extracted point cloud within the specified ROI.
    """
    pc_data = np.asarray(list(pc2.read_points(msg, skip_nans=True)))
    pc_array = pc_data.reshape(172, 224, 3)
    x_coords, y_coords = zip(*pix)
    x_min, x_max = min(x_coords), max(x_coords)
    y_min, y_max = min(y_coords), max(y_coords)
    
    roi_point_cloud = pc_array[y_min:y_max + 1, x_min:x_max + 1, :]
    # print(f"Size: {roi_point_cloud.shape}")
    roi_point_cloud = roi_point_cloud.reshape(roi_point_cloud.shape[0] * roi_point_cloud.shape[1], 3)

    return roi_point_cloud

def publish_segmented_point_cloud(segmented_point_cloud):
    """
    Publish the segmented point cloud with the corrected header.

    Args:
        segmented_point_cloud (numpy.ndarray): The segmented point cloud data.
    """

    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'royale_camera_0_optical_frame'

    cloud_msg = pc2.create_cloud_xyz32(header, segmented_point_cloud)
    pub_segmented_point_cloud.publish(cloud_msg)

def index_finder(points):
    """
    Find the product of indices for each point in the given list.

    Args:
        points (List[List[int]]): List of points in the format [[x1, y1], [x2, y2], ...].

    Returns:
        List[int]: List of products of indices for each point.
    """
    multiplied_points = []
    for point in points:
        product = 1
        for num in point:
            product *= num
        multiplied_points.append(product)

    return multiplied_points

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node("point_cloud_segmentation")

    # Subscribe to the image topic
    image_topic = "/royale_cam0/gray_image"
    image_sub = rospy.Subscriber(image_topic, Image, image_callback)

    # Subscribe to the depth topic
    # depth_topic = "/royale_cam0/depth_image"
    # depth_sub = rospy.Subscriber(depth_topic, Image, depth_callback)

    # Subscribe to the point cloud topic
    point_cloud_topic = "/royale_cam0/point_cloud"
    point_cloud_sub = rospy.Subscriber(point_cloud_topic, PointCloud2, point_cloud_callback)

    # Create a ROS publisher for the segmented point cloud
    pub_segmented_point_cloud = rospy.Publisher("/royale_cam0/segmented_point_cloud", PointCloud2, queue_size=10)

    # Spin ROS node
    rospy.spin()
