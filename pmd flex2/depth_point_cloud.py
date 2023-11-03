#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
from sensor_msgs.msg import PointCloud2,PointField
from sensor_msgs import point_cloud2
from sensor_msgs.msg import CameraInfo
import sensor_msgs.point_cloud2 as pc2
import pyrealsense2
from std_msgs.msg import Header
import open3d as o3d
import torch
import cv2
import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
# Define the callback function for the point cloud subscriber


# Initialize the ROS node
rospy.init_node('pc2_publisher')
pub = rospy.Publisher('depth_points', PointCloud2, queue_size=100)
rate = rospy.Rate(10)



def convert_depth_to_phys_coord_using_realsense(x, y, depth, cameraInfo):  
    _intrinsics = pyrealsense2.intrinsics()
    _intrinsics.width = cameraInfo.width
    _intrinsics.height = cameraInfo.height
    _intrinsics.ppx = cameraInfo.K[2]
    _intrinsics.ppy = cameraInfo.K[5]
    _intrinsics.fx = cameraInfo.K[0]
    _intrinsics.fy = cameraInfo.K[4]
    #_intrinsics.model = cameraInfo.distortion_model
    _intrinsics.model  = pyrealsense2.distortion.none 
    # print(cameraInfo.D) #output is [] empty 
    # _intrinsics.coeffs = [i for i in cameraInfo.D]  
    result = pyrealsense2.rs2_deproject_pixel_to_point(_intrinsics, [x, y], depth)  #result[0]: right, result[1]: down, result[2]: forward
    return result[2], -result[0], -result[1]
def publishPC2(points):

    rotation_matrix = np.array([[-1, 0, 0],
                            [0, -1, 0],
                            [0, 0, 1]])
    
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('intensity', 12, PointField.FLOAT32, 1)]

    header = Header()
    # header.frame_id = "world_flipped"
    header.frame_id = "world"
    header.stamp = rospy.Time.now()
    pc2 = point_cloud2.create_cloud(header, fields, points)
    pub.publish(pc2)

  
def get_3d_coord(depth_image,camera_info_msg):

    points=[]
    for i in range(0,depth_image.shape[0]):
        for j in range(0,depth_image.shape[1]):

            result = convert_depth_to_phys_coord_using_realsense(i,j,depth_image[i,j]*0.001,camera_info_msg)
            x = result[2]
            y = -result[1]
            z = -result[0]
            points.append([x,y,z,z])
    np_points = np.asarray(points)
    return np_points

def main():
    try:
        print("start")
        # Create a CvBridge object to convert ROS messages to OpenCV images
        bridge = CvBridge()
        # while not rospy.is_shutdown():

            
        camera_info_msg = rospy.wait_for_message('/camera1/color/camera_info', CameraInfo)

        depth_image_msg = rospy.wait_for_message('/camera1/depth/image_raw', Image)

        print('got depth image')
        depth_image = bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding="passthrough")
        print(depth_image.shape,type(depth_image),depth_image.dtype,depth_image[100,100])

        rgb_image_msg = rospy.wait_for_message('/camera1/color/image_raw', Image)   
        abdomen_points=get_3d_coord(depth_image,camera_info_msg)

        print("upto publish")
        while not rospy.is_shutdown():
            publishPC2(abdomen_points)
            rate.sleep()
            # '''
            # rospy.spin()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
if __name__ == "__main__":
    main()
