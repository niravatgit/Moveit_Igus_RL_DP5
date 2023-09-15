#!/usr/bin/env python3
import rospy
import moveit_commander
import moveit_msgs.msg
import numpy as np
from sklearn.neighbors import KDTree
from geometry_msgs.msg import Pose,TransformStamped
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
import math
from numpy.linalg import norm
import numpy as np
import copy
import tf2_ros
# import ros_numpy
#from tf.transformations import quaternion_from_matrix,euler_from_quaternion,quaternion_from_euler
from transformations import quaternion_from_matrix
from time import sleep
from visualization_msgs.msg import Marker,MarkerArray
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from sklearn.neighbors import NearestNeighbors
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import time
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
bridge = CvBridge()

# moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('generating_custom_trajectory', anonymous=True)
pub_depth_points1=rospy.Publisher('custom_points1', PointCloud2, queue_size=1)
# pub_depth_points2=rospy.Publisher('custom_points2', PointCloud2, queue_size=1)
# pub_depth_points3=rospy.Publisher('custom_points3', PointCloud2, queue_size=1)
# pub_depth_points4=rospy.Publisher('custom_points4', PointCloud2, queue_size=1)
rate = rospy.Rate(1)
# # Set up the robot arm group and planning scene
# robot = moveit_commander.RobotCommander()
# scene = moveit_commander.PlanningSceneInterface()
# arm_group = moveit_commander.MoveGroupCommander('manipulator')
# display_trajectory_publisher = rospy.Publisher(
#             "/move_group/display_planned_path",
#             moveit_msgs.msg.DisplayTrajectory,
#             queue_size=20,
#         )
# planning_frame = arm_group.get_planning_frame()
# arm_group.set_goal_orientation_tolerance(0.1)
# # Set the start state of the robot
# arm_group.set_start_state_to_current_state()
# arm_group.set_max_velocity_scaling_factor(0.05)
start_time = time.time()
show_image=True
image_center_x1=100
image_center_x2=426
image_center_y1=221
image_center_y2=563
# image_center_x1=150
# image_center_x2=350
# image_center_y1=200
# image_center_y2=400
box=True
path_following=False
# view_frame=True
test=False
view_frame=True
view_points=True
view_final_points=True
point_cloud_active=True
subroutine_checkup=False
# def visualize_trajectory(waypoints,r,g,b):
#     display_trajectory = DisplayTrajectory()
#     robot_trajectory = RobotTrajectory()
#     robot_trajectory.joint_trajectory.points = waypoints
#     display_trajectory.trajectory.append(robot_trajectory)

#     marker_array = MarkerArray()
#     for i, point in enumerate(waypoints):
#         marker = Marker()
#         # marker.header.frame_id = "tool_link_ee"
#         marker.header.frame_id = "world"
#         marker.header.stamp = rospy.Time.now()
#         marker.id = i
#         marker.type = Marker.SPHERE
#         marker.action = Marker.ADD
#         marker.pose.position.x = point.position.x
#         marker.pose.position.y = point.position.y
#         marker.pose.position.z = point.position.z
#         marker.pose.orientation.x = point.orientation.x
#         marker.pose.orientation.y = point.orientation.y
#         marker.pose.orientation.z = point.orientation.z
#         marker.pose.orientation.w = point.orientation.w
#         marker.scale.x = 0.002
#         marker.scale.y = 0.002
#         marker.scale.z = 0.002
#         marker.color.a = 1.0
#         marker.color.r = r
#         marker.color.g = g
#         marker.color.b = b
#         marker_array.markers.append(marker)
#         # print(" enumerating at {}".format(i))
#     return marker_array
def visualize_array(waypoints,r,g,b):
    # display_trajectory = DisplayTrajectory()
    # robot_trajectory = RobotTrajectory()
    # robot_trajectory.joint_trajectory.points = waypoints
    # display_trajectory.trajectory.append(robot_trajectory)
    world_origin=[0.0,0.0,0.0]
    marker_array = MarkerArray()
    for i, point in enumerate(waypoints):
        marker = Marker()
        # marker.header.frame_id = "tool_link_ee"
        marker.header.frame_id = "royale_camera_link"
        marker.header.stamp = rospy.Time.now()
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point[0]+world_origin[0]
        marker.pose.position.y = point[1]+world_origin[1]
        marker.pose.position.z = point[2]+world_origin[2]
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker_array.markers.append(marker)
        # print(" enumerating at {}".format(i))
    return marker_array
# Point normal Calculation
# def euler_from_quaternions():
#     # Get the current pose of the end-effector
#     current_pose = arm_group.get_current_pose()

#     # Convert the quaternion orientation to Euler angles
#     orientation_quat = (current_pose.pose.orientation.x,
#                         current_pose.pose.orientation.y,
#                         current_pose.pose.orientation.z,
#                         current_pose.pose.orientation.w)
#     (roll, pitch, yaw) = euler_from_quaternion(orientation_quat)
#     return (roll, pitch, yaw)

# def go_to_pose_goal(pose_goal,i):
#     arm_group.set_pose_target(pose_goal)
#     rospy.loginfo(f"Planning Trajectory to Pose {i}")
#     plan1=arm_group.plan()
#     success = arm_group.go(wait=True)

#     # rospy.loginfo(f"DONE EXECUTING Planning Trajectory success or failure : {success}")
#     arm_group.stop()
#     arm_group.clear_pose_targets()
#     return success

# exit(0)
def flat(lis):
    flatList = []
    # Iterate with outer list
    for element in lis:
        if type(element) is list:
            # Check if type is list than iterate through the sublist
            for item in element:
                flatList.append(item)
        else:
            flatList.append(element)
    return flatList


def calculate_normals(points, point_list, k=5):
    normals = []
    neigh = NearestNeighbors(n_neighbors=k)
    neigh.fit(points)
    avg_num_neighbors = np.mean([len(neigh.kneighbors([p], return_distance=False)[0]) for p in points])

    for p in point_list:
        indices = neigh.kneighbors([p], return_distance=False)[0]
        # print(indices)
        # if len(indices) < avg_num_neighbors:
        #     # Skip points with less than the average number of neighbors
        #     continue
        neighbors = points[indices]
        centroid = np.mean(neighbors, axis=0)
        covariance_matrix = np.cov(neighbors, rowvar=False)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance_matrix)
        normal = eigenvectors[:, np.argmin(eigenvalues)]
        if np.dot(normal, np.array([0,0,1.0])) < 0:
        # if np.dot(normal, centroid-p) < 0:

            normal *= -1
        normals.append(normal)
    return normals
# def publish_cloud(points1,points2,points3,points4):
def publish_cloud(points1):
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)
              ,PointField('intensity', 12, PointField.FLOAT32, 1)]

    header = Header()
    # header.frame_id = "world_flipped"
    # header.frame_id = "camera_color_optical_frame"
    header.frame_id = "royale_camera_link"
    header.stamp = rospy.Time.now()
    pc1 = pc2.create_cloud(header, fields, points1)
    # pc = pc2.create_cloud(header, fields, points2)
    # pc3 = pc2.create_cloud(header, fields, points3)
    # pc4 = pc2.create_cloud(header, fields, points4)
    pub_depth_points1.publish(pc1)
    # pub_depth_points2.publish(pc)
    # pub_depth_points3.publish(pc3)
    # pub_depth_points4.publish(pc4)


def point_cloud_callback(cloud_points):
    try:
        print("successfully subscribed to point_cloud_transformed")
        # pcl_data = list(pc2.read_points(cloud_points, skip_nans=True, field_names=("x", "y", "z")))
        # pcl_points = np.array([[p[0], p[1], p[2]] for p in pcl_data])
        pcl_points = np.array(list(pc2.read_points(cloud_points)))
        i=0
        
        # print("pcl_points.shape: ",pcl_points.shape)
        # pcl_points=np.flipud(pcl_points.reshape(480,640,-1))
        # print("origin.shape: ",pcl_points[240,320,:])
        

        # if test:
        #     print("pcl_points.shape: ",pcl_points.shape)
        #     pcl_points1=pcl_points[100:200,100:240,:].reshape(-1,3)
        #     new_column_data = 10*np.ones((pcl_points1.shape[0], 1))
        #     new_array = np.hstack((pcl_points1, new_column_data))
        #     publish_cloud(new_array)



        # if show_image:
        #     # pcl_points=pcl_points.reshape(640,480,-1)
        #     # pcl_points=pcl_points[100:200,300:710,0:100]
        #     new_height=image_center_y2-image_center_y1
        #     new_width=image_center_x2-image_center_x1
        #     pooling=10
        #     print("new_height: ",new_height,"new_width: ",new_width)
        #     rgb_image_msg=rospy.wait_for_message('/rgb', Image)
        #     rgb_image = bridge.imgmsg_to_cv2(rgb_image_msg, desired_encoding="bgr8")
        #     cv2.circle(rgb_image, (image_center_x1, image_center_y1), 5, (0, 0, 255), -1)
        #     cv2.circle(rgb_image, (image_center_x2, image_center_y2), 5, (0, 255, 0), -1)
        #     roi_image=cv2.rectangle(rgb_image, (image_center_x1, image_center_y1), (image_center_x2, image_center_y2), (255,0,0), 2)
        #     # cv2.imwrite("roi.jpg",roi_image)
        #     print("roi_image.shape: ",roi_image.shape)
        #     plt.imsave('roi_image_plt.jpg', roi_image)
        #     # plt.imshow(roi_image)
        #     # plt.show()
        #     selected_pcl_points = pcl_points[image_center_x1:image_center_x2,image_center_y1:image_center_y2,:].reshape(-1, 3)
        #     # selected_pcl_points = pcl_points.reshape(-1, 3)
        #     pcl_points = pcl_points[image_center_x1-20:image_center_x2+20,image_center_y1-20:image_center_y2+20].reshape(-1, 3)

        #     # selected_pcl_points=selected_pcl_points[500:len(selected_pcl_points):pooling]
        #     # selected_pcl_points=pcl_points
        #     if test:
        #         new_column_data = 10*np.ones((selected_pcl_points.shape[0], 1))
        #         new_array = np.hstack((selected_pcl_points, new_column_data))
        #         publish_cloud(new_array)

        # else:
        selected_pcl_points=pcl_points
            # selected_pcl_points = pcl_points.reshape(-1, 3)

            # selected_pcl_points=pcl_points[500:len(pcl_points)-500:10000]

        input("============ Press `Enter` to generate trajectory...")

        # while not rospy.is_shutdown():
        #     rospy.spin()
        # Extract x, y, z, and normal components from pcl_data

        # pcl_normals = np.array([[p[3], p[4], p[5]] for p in pcl_data])

        # print("selected_pcl_points example shape: ",selected_pcl_points[0].shape,"ex -1 :",selected_pcl_points[0:5])
        print("selected_pcl_points.shape : ",selected_pcl_points.shape,"pcl_points.shape :",pcl_points.shape)

        # Check if pcl_normals array is empty or has invalid dimensions
        selected_pcl_normals = calculate_normals(pcl_points,selected_pcl_points)
        print("Total points :", len(selected_pcl_points),"total normal points :",len(selected_pcl_normals))
        y_axis_global = np.array([0, 1, 0])
        if view_points:
            marker_pub = rospy.Publisher('smooth_trajectory_markers', MarkerArray, queue_size=1, latch=True)
            marker_array=visualize_array(list(selected_pcl_points),1.0,0.0,0.0)
            marker_pub.publish(marker_array)
        static_transforms=[]
        waypoints=[]
        print("calculating pose :")
        for i,normal in enumerate(selected_pcl_normals):

            z_axis_local=-1*normal
            x_axis_local = np.cross(normal, y_axis_global)
            x_axis_local/= np.linalg.norm(x_axis_local)
            y_axis_local= np.cross(z_axis_local,x_axis_local)
            y_axis_local/= np.linalg.norm(y_axis_local)
            z_axis_local/= np.linalg.norm(z_axis_local)
            # rotation_matrix = np.column_stack((-1*y_axis_local,x_axis_local,z_axis_local))
            rotation_matrix = np.column_stack((x_axis_local,y_axis_local,z_axis_local))
            quaternion_data = quaternion_from_matrix(np.vstack([np.hstack([rotation_matrix, np.zeros((3, 1))]), [0, 0, 0, 1]]))
            # print("active")
            pose_goal =Pose()
            pose_goal.position.x = selected_pcl_points[i][0]
            pose_goal.position.y = selected_pcl_points[i][1]
            pose_goal.position.z = selected_pcl_points[i][2]
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
            # out=go_to_pose_goal(pose_goal)

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

        # # if show_image:
        # #     print("Shape of waypoints before:",np.array(waypoints).shape)
        # #     if view_final_points:
        # #         marker_array=visualize_trajectory(list(waypoints),0.0,1.0,0.0)
        # #         marker_pub.publish(marker_array)
        #     final_waypoints=np.flipud(np.array(waypoints).reshape(new_width,new_height,-1))
        #     # final_waypoints=waypoints[image_center_x1:image_center_x2,image_center_y1:image_center_y2].reshape(new_width,new_height,-1)
        #     print("Shape of waypoints after:",final_waypoints.shape)
        #     if box:
        #         box_waypoints=[]
        #         for j in np.arange(0,new_height,20):
        #             box_waypoints.append(final_waypoints[0,j,0])
                
        #         for i in np.arange(0,new_width,20):
        #             box_waypoints.append(final_waypoints[i,new_height-1,0])
        #         for j in np.arange(0,new_height,20)[::-1]:
        #             box_waypoints.append(final_waypoints[new_width-1,j,0])
        #         for i in np.arange(0,new_width,20)[::-1]:
        #             box_waypoints.append(final_waypoints[i,0,0])
        #         print(waypoints)
        #         (plan, fraction) = arm_group.compute_cartesian_path(
        #         box_waypoints, 0.001, 10  # waypoints to follow  # eef_step
        #     ) 
        #         print("Plan for box iteration,:",fraction) 
        #         # # print(plan) 
        #         print("length of waypoints : ",len(box_waypoints))
        #         if view_final_points:
        #             marker_array=visualize_trajectory(list(box_waypoints),0.0,1.0,0.0)
        #             marker_pub.publish(marker_array)
        #         input("============ Press `Enter` to execute robot trajectory...")
            #     arm_group.execute(plan, wait=True)
            #     # for i,waypoint in enumerate(box_waypoints):
            #     #         out=go_to_pose_goal(waypoint,i)
            #     #         print("pose at ",i,"th :",arm_group.get_current_pose().pose) 
            #     # print(arm_group.get_current_pose().pose)

            #     arm_group.stop()
            #     arm_group.clear_pose_targets()
                
            # # '''
            # if path_following:
            #     for i in np.arange(2,new_width-5,20):

            #         # cartesi.shapean path planning
            #         points_to_be_followed=[]
            #         for j in np.arange(2,new_height-5,15):
            #             # print(final_waypoints[i,j,0])    # header.frame_id = "world_flipped"
            #             wpose=final_waypoints[i,j,0]
            #             if(wpose.position.x <0.0001 and wpose.position.y <0.0001 and wpose.position.z<0.0001):
            #                 print("zero point : ",i,j,"pose: ",wpose.position)
            #                 continue
            #             points_to_be_followed.append(wpose)
            #         (plan, fraction) = arm_group.compute_cartesian_path(
            #         points_to_be_followed, 0.01, 0  # waypoints to follow  # eef_step
            #     )   
                    
            #         print("Plan for ",i,"th iteration,:",fraction)
            #         if view_final_points:
            #             marker_array=visualize_trajectory(list(points_to_be_followed),0.0,1.0,0.0)
            #             marker_pub.publish(marker_array)
            #         # print(plan)
            #         input("============ Press `Enter` to execute robot trajectory...")
            #         # while not rospy.is_shutdown():
            #         #     rospy.spin()
            #         arm_group.execute(plan, wait=True)
            #         arm_group.stop()
            #         arm_group.clear_pose_targets()

        # '''
        '''
        # else:
        #     waypoints=np.array(waypoints).reshape(640,480)
        # for i,waypoint in enumerate(waypoints):
        #     # for wp in waypoint:

            if(i==1000):
                out=go_to_pose_goal(waypoint,i)
                break
            # print("done")
            # break
        # arm_group.stop()
        # arm_group.clear_pose_targets()

        # cartesian path planning
        start=0
        end=int(len(waypoints)/10)
        for i in range(1,10):
            (plan, fraction) = arm_group.compute_cartesian_path(
                waypoints[start:end], 0.001, 0  # waypoints to follow  # eef_step
            )
            start=end
            end=end*i
            print(f"fraction is : {fraction},{0}")
            # print(plan)
            arm_group.execute(plan, wait=True)
            arm_group.stop()
            arm_group.clear_pose_targets()
            print("going into sleep state")
            time.sleep(4)
            print("out of.shape sleep state")
            # if()
        '''
        print("finished code")
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
            return
    except KeyboardInterrupt:
        return

def main():
    try:

        print("connecting to /point_cloud_transformed")
        subscriber = rospy.Subscriber('/translated_point_cloud', PointCloud2, point_cloud_callback, queue_size=1)
        # subscriber = rospy.Subscriber('/depth_points', PointCloud2, point_cloud_callback, queue_size=1)
        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
            return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":

    main()
