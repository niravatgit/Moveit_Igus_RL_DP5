#!/usr/bin/env python3
"""
normal2cartesianpath.py

This script visualizes normals computed from a point cloud and moves a robot arm 
to follow a specified path defined by the normals.

It subscribes to a point cloud topic, computes normals using Open3D library, 
visualizes the normals as arrows in Rviz, transforms the normals into poses, 
plans a trajectory to follow the path defined by the normals, and moves the robot arm.

"""

from geometry_msgs.msg import Pose, TransformStamped
from sensor_msgs.msg import JointState, PointCloud2
from sklearn.neighbors import NearestNeighbors
from std_msgs.msg import String
from tf.transformations import quaternion_from_matrix, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray

import moveit_commander
import moveit_msgs.msg
import numpy as np
import open3d as o3d
import rospy
import sensor_msgs.point_cloud2 as pc2
import sys
import tf2_ros

try:
    from math import pi
except ImportError:
    pi = 3.141592653589793

# Initialize ROS node
rospy.init_node('visualize_normals_and_move_robot', anonymous=True)

# Initialize MoveIt
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "rldp5"
arm_group = moveit_commander.MoveGroupCommander(group_name)

rospy.sleep(1) 
# Publishers   
global marker_pub, display_trajectory_pub

marker_pub = rospy.Publisher('/normals_markers', MarkerArray, queue_size=10)

display_trajectory_pub = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

def disp_trajectory(plan):
    """
    Publishes a trajectory for visualization in Rviz.

    Args:
        plan (moveit_msgs.msg.RobotTrajectory): The planned trajectory to visualize.
    """
    global display_trajectory_pub

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()

    if isinstance(plan, tuple):
        print("Failed to plan a valid trajectory.")
        return

    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)

    display_trajectory_pub.publish(display_trajectory)
    rospy.sleep(5)

def go_to_joint_state():
    """
    Moves the robot arm to a predefined joint state (home position). Generally away from the singulatity points
    """
    # Print the current joint values
    print("Current joint values:", arm_group.get_current_joint_values())

    # Copy class variables to local variables to make the code clearer.
    joint_goal = JointState()
    joint_goal.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
    joint_goal.position = [0, 0, -pi/16, -pi/16, 0]
    # joint_goal.position = [1.243416462060054e-05, 6.248125811403327e-06, -1.5707127297497383, -0.7853212021615734, -8.443752722797626e-06]


    try:
        input("======== Upright Position of the robot is a Singularity, Press enter to move the robot to home position ========")
        arm_group.go(joint_goal, wait=True)
        arm_group.stop()
        print("Moved to home position\n")

    except moveit_commander.exception.MoveItCommanderException as e:
        print(f"Error setting joint target: {e}")

        try:
            input("======== Upright Position of the robot is a Singularity, Press enter to move the robot to home position ========")
            arm_group.go(joint_goal, wait=True)
            arm_group.stop()
            print("Moved to home position\n")
        except moveit_commander.exception.MoveItCommanderException as e:
            print(f"Error setting joint target: {e}")
    
    arm_group.clear_pose_targets()

def visualize_normals(points, normals):
    """
    Visualizes normals as arrows in Rviz and transforms them into poses.

    Args:
        points (numpy.ndarray): Array of 3D points.
        normals (numpy.ndarray): Array of corresponding normal vectors.

    Returns:
        list: List of poses generated from the normals.
    """

    waypoints_list = []

    if points is not None and normals is not None:
        min_length = min(len(points), len(normals))
        
        marker_array = MarkerArray()

        for i in range(min_length):
            point = points[i]
            normal = normals[i]
            quaternion = quaternion_from_normal(normal)

            pose_goal = Pose()
            pose_goal.position.x = point[0]
            pose_goal.position.y = point[1]
            pose_goal.position.z = point[2]
            pose_goal.orientation.x = quaternion[0]
            pose_goal.orientation.y = quaternion[1]
            pose_goal.orientation.z = quaternion[2]
            pose_goal.orientation.w = quaternion[3]

            waypoints_list.append(pose_goal)

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

            marker.pose.orientation.x = quaternion[0]
            marker.pose.orientation.y = quaternion[1]
            marker.pose.orientation.z = quaternion[2]
            marker.pose.orientation.w = quaternion[3]
            # marker.pose.orientation.w = -1.0

            marker.scale.x = 0.01  # Arrow shaft diameter
            marker.scale.y = 0.001  # Arrow head diameter
            marker.scale.z = 0.001  # Arrow head length
            marker.color.a = 1.0  # Alpha (transparency)
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            marker_array.markers.append(marker)

        marker_pub.publish(marker_array)
        # print("Published the markers of normals")

    return waypoints_list

def quaternion_from_normal(normal):
    """
    Computes quaternion from a given normal vector.

    Args:
        normal (numpy.ndarray): Normal vector.

    Returns:
        numpy.ndarray: Quaternion representing the orientation.

    Methods:
        Two methods are used to compute quaternions
        1. By mathematical Modelling
        2.  Using Gram-Schmidt process
    """
    # Method 1: Mathematical modelling
    # # Up vector
    # up_vector = np.array([0.0, 0.0, 1.0])

    # # Calculate the rotation axis
    # axis = np.cross(up_vector, normal)
    
    # # Calculate the rotation angle
    # angle = np.arccos(np.dot(up_vector, normal))

    # # Calculate quaternion components
    # quaternion = np.array([
    #     np.cos(angle / 2),
    #     -np.sin(angle / 2) * axis[0],
    #     -np.sin(angle / 2) * axis[1],
    #     -np.sin(angle / 2) * axis[2]
    # ])

    # quaternion = quaternion_from_matrix([[np.cos(angle / 2), -np.sin(angle / 2) * axis[0], -np.sin(angle / 2) * axis[1], -np.sin(angle / 2) * axis[2]],
    #                                     [np.sin(angle / 2) * axis[0], np.cos(angle / 2), 0, 0],
    #                                     [np.sin(angle / 2) * axis[1], 0, np.cos(angle / 2), 0],
    #                                     [np.sin(angle / 2) * axis[2], 0, 0, np.cos(angle / 2)]])

    # 2.  Using Gram-Schmidt Process
    y_axis_global = [0.0, 1.0, 0.0]
    z_axis_local = 1*normal
    x_axis_local = np.cross(normal, y_axis_global)
    x_axis_local/= np.linalg.norm(x_axis_local)
    y_axis_local= np.cross(z_axis_local,x_axis_local)
    y_axis_local/= np.linalg.norm(y_axis_local)
    z_axis_local/= np.linalg.norm(z_axis_local)
    rotation_matrix = np.column_stack((x_axis_local,y_axis_global,z_axis_local))
    quaternion = quaternion_from_matrix(np.vstack([np.hstack([rotation_matrix, np.zeros((3, 1))]), [0, 0, 0, 1]]))

    quaternion /= np.linalg.norm(quaternion)
    # print(f"Quaternion: {quaternion}")
    return quaternion

def go_to_pose_goal(fixed_pose):

    """
    Move the robot arm to a specified pose.

    Args:
        fixed_pose: The target pose for the end-effector.

    Returns:
        None
    """
    eef = arm_group.get_end_effector_link()
    # print(eef)
    # print(f"in the go to pose goal function: {fixed_pose}")

    # Set the target pose for the end-effector
    print(type(fixed_pose))
    pose = [fixed_pose]
    print(type(pose))
    arm_group.set_pose_target(pose, end_effector_link = 'tool0')

    # Plan the motion
    ik_plan = arm_group.plan()

    # Check if the plan is valid
    if isinstance(ik_plan, tuple) or not ik_plan:
        print("Failed to plan a valid trajectory.")
        # print(f"IK Plan: {ik_plan}")
        # print(f"Type of IK Plan: {type(ik_plan)}")
        arm_group.clear_pose_targets()
        return

    print("Planning success!")

    try:
        # Execute the planned motion
        arm_group.execute(ik_plan, wait=True)

        # Alternatively, you can visualize the planned trajectory
        disp_trajectory(ik_plan)
    except Exception as e:
        print(f"Error during execution: {e}")
    finally:
        # Clear the pose targets
        arm_group.clear_pose_targets()

def compute_normals(point_cloud, k_neighbors=30):
    """
    Computes normals for a point cloud using Open3D library.

    Args:
        point_cloud (numpy.ndarray): Array of 3D points.
        k_neighbors (int): Number of neighbors for normal estimation.

    Returns:
        numpy.ndarray: Array of normal vectors.
    
    Methods:
        1. Using Open3D module
        2. Normal Estimation from PCL (Check other codes probably testcode.cpp & move_robot.py) for reference
    """
    # Convert point cloud to Open3D format
    o3d_cloud = o3d.geometry.PointCloud()
    o3d_cloud.points = o3d.utility.Vector3dVector(point_cloud)

    # Compute normals using Open3D (CUDA version)
    o3d_cloud.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=k_neighbors),
        fast_normal_computation=True  # Enable CUDA acceleration
    )

    # Get the normals as a numpy array
    normals = np.asarray(o3d_cloud.normals)
    
    return normals


def pc_callback(cloud_msg):
    """
    Callback function for processing point cloud messages.

    Args:
        cloud_msg (sensor_msgs.msg.PointCloud2): Point cloud message.
    """

    point_cloud = np.asarray(list(pc2.read_points(cloud_msg, skip_nans=True)))

    # print(f"Shape of point cloud: {.point_cloud.shape}")

    computed_normals = compute_normals(point_cloud)

    waypoints = visualize_normals(point_cloud, computed_normals)
    # print("pose onbject 1 :",waypoints[0])
    # print(f"Shape of waypoints: {np.asarray(waypoints).shape}")

    # Can be found via uncommenting the print statement in the extract_roi_from_point_cloud function of pixel2pcl.py file
    height = 31
    width = 31

    try:
        # Attempt to reshape the waypoints array into a 3D array
        waypoint_list = np.asarray(waypoints).reshape(width, height, -1)
        
    except ValueError as e:
        # Handle the ValueError if the array cannot be reshaped
        print(f"Error reshaping the array: {e}")
        return
        
    # Initialize lists to store points and grids to be followed
    points_to_be_followed = []
    grid_to_be_followed = []

    # Initialize toggle variable for grid following
    toggle = False

    # Set path_following flag to control the loop
    path_following = True

    if path_following:
        # Iterate over the height of the waypoint_list
        for j in np.arange(0, height):
            # Initialize a list to store points in a line to be followed
            line_to_be_followed = []

            # Iterate over the width of the waypoint_list
            for i in np.arange(0, width):
                # Append the point to the line_to_be_followed list
                line_to_be_followed.append(waypoint_list[i, j, 0])

                # Append the point to the points_to_be_followed list
                points_to_be_followed.append(waypoint_list[i, j, 0])

            # Toggle the line_to_be_followed list if necessary
            if toggle:
                line_to_be_followed = line_to_be_followed[::-1]
                toggle = False
            else:
                toggle = True

            # Append the line_to_be_followed list to the grid_to_be_followed list
            grid_to_be_followed.append(line_to_be_followed)

    # print(f"Shape of grid_to_be_followed: {np.asarray(grid_to_be_followed).shape}")
    grid = np.asarray(grid_to_be_followed)  
    # points_to_follow = grid.reshape(31*31, -1).tolist()
    
    # Create static transformations for each pose goal in points_to_be_followed
    static_transforms = []
    for i, pose_goal in enumerate(points_to_be_followed):
        # Extract translation and rotation components from the pose goal
        translation = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z]
        rotation = [pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]

        # Construct a static transform dictionary
        static_transforms.append({
            'parent_frame_id': 'royale_camera_0_optical_frame',
            'child_frame_id': 'frame_{}'.format(i),
            'translation': translation,
            'rotation': rotation
        })

    # Create a TF2 static transform broadcaster
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Create a list to store TransformStamped messages
    t_list = []

    # Iterate over each static transform and create a TransformStamped message
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

    # Send the list of TransformStamped messages using the static transform broadcaster
    static_broadcaster.sendTransform(t_list)

    # Extract the shape dimensions from the grid
    rows, cols = grid.shape

    # print(type(points_to_be_followed[0]))
    # ref_frame = 'royale_camera_0_optical_frame'
    # arm_group.set_pose_reference_frame(ref_frame)

    # arm_group.set_planning_pipeline_id('ompl')
    # arm_group.set_goal_tolerance(0.1)
    # arm_group.set_num_planning_attempts(10)
    # arm_group.set_planning_time(5)
    # arm_group.set_pose_targets(points_to_be_followed, end_effector_link = 'tool0')

    # plan = arm_group.plan()
    # rospy.sleep(5)

    # disp_trajectory(plan)

    # arm_group.go(wait = True)

    # Moving from singularity
    # go_to_joint_state()
    
    # for i in range(rows):
    #     for j in range(cols):
    #         # Access the element at position (i, j)
    #         pose = grid[i, j]
    #         # print(f"Element at ({i}, {j}): {pose}")
    #         go_to_pose_goal(pose)

    # target_row = 15

    # Loop through the elements in the specified row
    # for j in range(cols):
    #     pose = grid[target_row, j]
    #     print(f"Element at ({target_row}, {j}): {pose}")
    #     # Assuming you have a function go_to_pose_goal to perform actions with the pose
    #     go_to_pose_goal(pose)

    
    cartesian_plan, fraction = compute_cartesion(points_to_be_followed)
    # print(cartesian_plan,"\n\n\n\n")
    disp_trajectory(cartesian_plan)
    arm_group.execute(cartesian_plan, wait=True)

    # Cartesian path planning doesn't work please read this: https://picknik.ai/cartesian%20planners/moveit/motion%20planning/2021/01/07/guide-to-cartesian-planners-in-moveit.html
    
def compute_cartesion(waypoints):

    """
    Computes a Cartesian path for the robot arm to follow.

    Args:
        waypoints (list): List of poses to follow.

    Returns:
        moveit_msgs.msg.RobotTrajectory: Planned Cartesian path.
        float: Fraction of the path that was successfully planned.
    """
    plan, fraction = arm_group.compute_cartesian_path(
        waypoints,  # waypoints to follow
        0.005,          # eef_step
        1.0         # jump_threshold
    )
    return plan, fraction
def listener():

    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.Subscriber('/royale_cam0/segmented_point_cloud', PointCloud2, pc_callback)
    # rospy.Subscriber('/cloud_normals_something', PointCloud2, data_processor.nor_callback)


    rospy.spin()

if __name__ == '__main__':
    listener()
