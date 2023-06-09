import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import matplotlib.pyplot as plt

# Define the callback function for depth image
def depth_image_callback(msg):
    bridge = CvBridge()
    # Convert ROS Image message to OpenCV image
    depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    # Normalize depth values to the range [0, 255]
    # normalized_depth_image = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
    # Convert the grayscale image to color
    color_depth_image = cv2.cvtColor(depth_image, cv2.COLOR_BGR2GRAY)
    # Display the depth image
    # cv2.imshow("Depth Image", color_depth_image)
    # cv2.waitKey(1)
    plt.imshow(color_depth_image)
    plt.show()


# Initialize the node
rospy.init_node("depth_image_subscriber", anonymous=True)

# Define the topic name and message type
topic_name = "/camera/color/image_raw"
message_type = Image

# Subscribe to the depth image topic
rospy.Subscriber(topic_name, message_type, depth_image_callback)

# Start the main loop
rospy.spin()

