#!/usr/bin/env python3
import rospy
import numpy as np
import sys
from sklearn.neighbors import NearestNeighbors
from geometry_msgs.msg import Pose, TransformStamped
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf.transformations as tf_trans
from transformations import quaternion_from_matrix

bridge = CvBridge()
marker_pub = None

rospy.init_node('generating_custom_trajectory', anonymous=True)
rate = rospy.Rate(1)
start_time = rospy.Time.now()
show_image = True

box = True
path_following = False
test = False
view_frame = True
view_points = False
view_final_points = True
point_cloud_active = True
subroutine_checkup = False
last_publish_time = rospy.Time.now()

def visualize_array_with_normals(normals):
    global marker_pub, last_publish_time

    # print("came into visualize funciton")
    # print(f"\n\n\nNormals:\n{np.array(normals)}\n\n\n")

    marker_array = MarkerArray()
    for i, normal in enumerate(normals):
        marker = Marker()
        marker.header.frame_id = "royale_camera_link"
        marker.header.stamp = rospy.Time.now()
        marker.id = i
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = normal[0]
        marker.pose.position.y = normal[1]
        marker.pose.position.z = normal[2]

        # print(f"Normal: {normal}")

        # Calculate orientation based on the normal vector to always point upward
        z_axis_local = -1 * normal
        y_axis_local = np.array([0, 1, 0])
        # print("is it here?")
        # print(f"z_axis_local: {z_axis_local}\ny_axis_local: {y_axis_local}")
        x_axis_local = np.cross(z_axis_local, y_axis_local)
        # print("trying to debug!!")        

        rotation_matrix = np.column_stack((x_axis_local, y_axis_local, z_axis_local))
        quaternion_data = tf_trans.quaternion_from_matrix(np.vstack([np.hstack([rotation_matrix, np.zeros((3, 1))]), [0, 0, 0, 1]]))

        marker.pose.orientation.x = quaternion_data[0]
        marker.pose.orientation.y = quaternion_data[1]
        marker.pose.orientation.z = quaternion_data[2]
        marker.pose.orientation.w = quaternion_data[3]

        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker_array.markers.append(marker)

    marker_array_clear = MarkerArray()
    for i in range(len(marker_array.markers)):
        marker_clear = Marker()
        marker_clear.header.frame_id = "royale_camera_link"
        marker_clear.header.stamp = rospy.Time.now()
        marker_clear.id = i
        marker_clear.action = Marker.DELETE
        marker_array_clear.markers.append(marker_clear)

    if rospy.Time.now() - last_publish_time > rospy.Duration(0.1):  # Throttle the visualization (0.1 seconds interval)
        marker_pub.publish(marker_array)
        marker_pub.publish(marker_array_clear)
        last_publish_time = rospy.Time.now()

    return marker_array

def point_cloud_callback(cloud_points):
    global marker_pub, last_publish_time

    try:
        normals = np.array(list(pc2.read_points(cloud_points, skip_nans=True)))[:, :3]

        # print(f"The size of normals of the cloud: {normals.shape}")

        if len(normals) > 0:
            view_points = True
        else:
            rospy.loginfo("Provide a non-empty cloud")

        y_axis_global = np.array([0, 1, 0])

        if view_points:
            marker_pub = rospy.Publisher('normal_arrows', MarkerArray, queue_size=1, latch=True)
            marker_array = visualize_array_with_normals(list(normals))
            marker_pub.publish(marker_array)

        static_transforms = []
        waypoints = []

        for i, normal in enumerate(normals):
            z_axis_local = -1 * normal
            x_axis_local = np.cross(normal, y_axis_global)

            # Check if x_axis_local is too small before normalization
            if np.linalg.norm(x_axis_local) > 1e-6:
                x_axis_local /= np.linalg.norm(x_axis_local)
            else:
                rospy.logwarn("x_axis_local vector is too small, using default vector.")
                x_axis_local = np.array([1, 0, 0])  # Use a default vector if x_axis_local is too small
                continue

            y_axis_local = np.cross(z_axis_local, x_axis_local)

            # Check if y_axis_local is too small before normalization
            if np.linalg.norm(y_axis_local) > 1e-6:
                y_axis_local /= np.linalg.norm(y_axis_local)
            else:
                rospy.logwarn("y_axis_local vector is too small, using default vector.")
                y_axis_local = np.array([0, 1, 0])  # Use a default vector if y_axis_local is too small
                continue

            z_axis_local = np.cross(x_axis_local, y_axis_local)

            # Check if z_axis_local is too small before normalization
            if np.linalg.norm(z_axis_local) > 1e-6:
                z_axis_local /= np.linalg.norm(z_axis_local)
            else:
                rospy.logwarn("z_axis_local vector is too small, using default vector.")
                z_axis_local = np.array([0, 0, 1])  # Use a default vector if z_axis_local is too small
                continue

            rotation_matrix = np.column_stack((x_axis_local, y_axis_local, z_axis_local))
            quaternion_data = quaternion_from_matrix(np.vstack([np.hstack([rotation_matrix, np.zeros((3, 1))]), [0, 0, 0, 1]]))

            pose_goal = Pose()
            pose_goal.position.x = normals[i][0]
            pose_goal.position.y = normals[i][1]
            pose_goal.position.z = normals[i][2]
            pose_goal.orientation.x = quaternion_data[0]
            pose_goal.orientation.y = quaternion_data[1]
            pose_goal.orientation.z = quaternion_data[2]
            pose_goal.orientation.w = quaternion_data[3]
            waypoints.append(pose_goal)

            if(view_frame):
                static_transforms.append({
                    'parent_frame_id': 'royale_camera_link',
                    'child_frame_id': 'frame_{}'.format(i),
                    'translation': [pose_goal.position.x , pose_goal.position.y , pose_goal.position.z ],
                    'rotation': [pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]
                })

        if(view_frame):
            # Create a static transform broadcaster
            static_broadcaster = tf2_ros.StaticTransformBroadcaster()
            rate = rospy.Rate(10)
            t_list=[]
            for static_transform in static_transforms:
                t = TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = static_transform['parent_frame_id']
                t.child_frame_id = static_transform['child_frame_id']
                t.transform.translation.x = static_transform['translation'][0]
                t.transform.translation.y = static_transform['translation'][1]
                t.transform.translation.z = static_transform['translation'][2]
                t.transform.rotation.x = static_transform['rotation'][0]
                t.transform.rotation.y = static_transform['rotation'][1]
                t.transform.rotation.z = static_transform['rotation'][2]
                t.transform.rotation.w = static_transform['rotation'][3]
                t_list.append(t)
            static_broadcaster.sendTransform(t_list)

        if view_final_points:
            marker_pub = rospy.Publisher('trajectory_markers', MarkerArray, queue_size=1, latch=True)

    except rospy.ROSInterruptException as e:
        print(f"ROS Interrupt Exception: {e}", file=sys.stderr)

    except KeyboardInterrupt:
        print("Keyboard interrupt detected. Exiting...", file=sys.stderr)
        
    except Exception as e:
        print(f"An unexpected error occurred: {e}", file=sys.stderr)

def main():
    try:
        subscriber = rospy.Subscriber('/cloud_normals', PointCloud2, point_cloud_callback, queue_size=1)

        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()
