#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ImageProcessor:
    def __init__(self):
        rospy.init_node('image_processor_node', anonymous=True)

        # Set the topic you are subscribed to
        self.image_topic = '/royale_cam0/gray_image'  # Replace with your actual image topic

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

        # Subscribe to the image topic
        rospy.Subscriber(self.image_topic, Image, self.image_callback)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            gray_image_mono16 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))
            return

        # Normalize the image to 8-bit
        normalized_image = cv2.normalize(gray_image_mono16, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

        # Apply thresholding to create a binary mask
        _, binary_mask = cv2.threshold(normalized_image, 1, 255, cv2.THRESH_BINARY)

        # Find contours in the binary mask
        contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Iterate through contours and filter based on area or other criteria
        for contour in contours:
            # Example: Filter contours based on minimum area
            min_contour_area = 10  # Adjust this threshold based on your requirement
            if cv2.contourArea(contour) > min_contour_area:
                # Draw the contour on the original image
                cv2.drawContours(normalized_image, [contour], -1, (0, 255, 0), 2)

        # Display the processed image
        cv2.imshow('Processed Image', normalized_image)
        cv2.imwrite('Processed_Image.jpg', normalized_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    image_processor = ImageProcessor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
