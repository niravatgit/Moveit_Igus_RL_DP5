import struct
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import tf2_ros
import tf2_sensor_msgs
from geometry_msgs.msg import TransformStamped

# Initialize OpenCV bridge
bridge = CvBridge()

# Store detected corners
sorted_points = []

# Callback function for the image subscriber
def image_callback(msg):

    global sorted_points

    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="mono16")
        corners = detect_corners(cv_image)
        result_image = cv_image.copy()

        # Label corners into different groups
        _, labeled_image = cv2.connectedComponents((corners > 0.25 * corners.max()).astype(np.uint8))

        centroid_list = []
        # Calculate centroid for each labeled group
        for label in range(1, np.max(labeled_image)):
            group_mask = np.where(labeled_image == label)
            group_corners = np.array([group_mask[1], group_mask[0]]).T  # Convert to (x, y) format
            centroid = list(np.mean(group_corners, axis=0))
            centroid_list.append(list(centroid))
            # print(f"Centroid for group {label}: {centroid}")

        # print(centroid_list)

        # Additional processing or visualization can be added here
        for label in range(1, np.max(labeled_image)):
            result_image[corners > 0.25* corners.max()] = 255
        cv2.imwrite('harris_corner.jpg', result_image)

        # print(f"result_image.shape[0]: {result_image.shape[0]}\nresult_image.shape[1]: {result_image.shape[1]} \n result_image.shape[2]: {result_image.shape[2]}")

        # Sort points based on x-coordinates
        sorted_points = sorted(centroid_list , key=lambda p: p[0]) 

        sorted_points = [[int(round(element)) for element in sublist] for sublist in sorted_points]

        # print("sorted_points", sorted_points)

        # Print the sorted points
        # for point in sorted_points:
            # print("Sorted list of points: ",point)

    except Exception as e:
        print(e)

def detect_corners(image):
    # Convert image to float32
    image = np.float32(image)
    print(f"image.shape: {image.shape}")

    # Apply Harris Corner Detector
    corners = cv2.cornerHarris(image, blockSize=3, ksize=3, k=0.05)

    #Threshold and find coordinates
    # thresh = 0.5* corners.max()
    # coords = np.where(corners > thresh)
    # corner_points = list(zip(coords[1], coords[0])) #(x,y) coordinates
    # print(f"corner points: {corner_points}")
    return corners

# Callback function for the point cloud subscriber
def point_cloud_callback(msg):
    try:
        # Convert Point Cloud message to a list of points
        pc_data = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

        points = []
        for data in pc_data:
            x, y, z = data
            points.append([x, y, z])

        # print("before process_point_cloud call")
        segmented_point_cloud = process_point_cloud(points)
        # print("after process_point_cloud call")

        # Check if the header attribute exists in the msg object
        # Create a header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "royale_camera_link"
        
        # Process the point cloud
        # segmented_point_cloud = process_point_cloud(points)
        
        # Publish the segmented point cloud with the corrected header
        publish_segmented_point_cloud(segmented_point_cloud, msg.header)

    except Exception as e:
        print(e)

def process_point_cloud(points):
    segmented_point_cloud = []
    print(sorted_points)
    try:

        points = np.array(points)
        # print("\n\n\n\n", points)
        if sorted_points:
            min_x, _ = sorted_points[0]
            max_x, _ = sorted_points[-1]

            print(f"min_x: {min_x}\nmax_x: {max_x}")


            return segmented_point_cloud
        else:
            print("Sorted points are not available.")
            return []

    except Exception as e:
        print("Error in process_point_cloud:", e)
        return []

def publish_segmented_point_cloud(segmented_point_cloud, msg_header):
    # Create a PointCloud2 message for the segmented point cloud
    cloud_msg = PointCloud2()
    cloud_msg.header = msg_header  # Use the provided header

    cloud_msg.height = 1
    cloud_msg.width = len(segmented_point_cloud)
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)]
    cloud_msg.is_bigendian = False
    cloud_msg.point_step = 12
    cloud_msg.row_step = 12 * len(segmented_point_cloud)
    cloud_msg.data = b''.join([struct.pack('<3f', *point) for point in segmented_point_cloud])

    # Publish the segmented point cloud
    pub_segmented_point_cloud.publish(cloud_msg)
    print("Published the new topic")

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node("corner_detection_and_point_cloud")

    # Subscribe to the image topic
    image_topic = "/royale_cam0/gray_image"
    image_sub = rospy.Subscriber(image_topic, Image, image_callback)

    # Subscribe to the point cloud topic
    point_cloud_topic = "/royale_cam0/point_cloud"
    point_cloud_sub = rospy.Subscriber(point_cloud_topic, PointCloud2, point_cloud_callback)

    # Create a ROS publisher for the segmented point cloud
    pub_segmented_point_cloud = rospy.Publisher("/royale_cam0/segmented_point_cloud", PointCloud2, queue_size=10)

    # Spin ROS node
    rospy.spin()