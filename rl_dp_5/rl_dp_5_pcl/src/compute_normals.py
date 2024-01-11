#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
import numpy as np
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from tf.transformations import quaternion_from_matrix
import tf2_ros
import time
import moveit_commander, moveit_msgs.msg


rospy.init_node('visualize_normals_and_path_planning', anonymous=True)
tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(12))
tf_listener = tf2_ros.TransformListener(tf_buffer)

box_flag = False
path_following = True

# Set up the robot arm group and planning scene
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

# Name of the group can be found at the "/home/inspire-0/moveit_igus_ws/src/Moveit_Igus_RL_DP5/rl_dp_5/rl_dp_5_moveit/config/igus_robolink_rl_dp_5.srdf" file
arm_group = moveit_commander.MoveGroupCommander('rldp5')
display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)
planning_frame = arm_group.get_planning_frame()
arm_group.set_goal_orientation_tolerance(0.1)

# Set the start state of the robot
arm_group.set_start_state_to_current_state()
arm_group.set_max_velocity_scaling_factor(0.05)
start_time = time.time()

# Global publisher for markers
global marker_pub

def visualize_trajectory(waypoints, r, g, b):
    """
    Visualizes a trajectory using markers.

    Parameters:
        waypoints (list): List of Pose points representing the trajectory.
        r (float): Red component of the RGB color.
        g (float): Green component of the RGB color.
        b (float): Blue component of the RGB color.

    Returns:
        marker_array (MarkerArray): Visualization of the trajectory as a MarkerArray.
    """

    display_trajectory = DisplayTrajectory()
    robot_trajectory = RobotTrajectory()
    robot_trajectory.joint_trajectory.points = waypoints
    display_trajectory.trajectory.append(robot_trajectory)

    marker_array = MarkerArray()
    for i, point in enumerate(waypoints):
        marker = Marker()
        # marker.header.frame_id = "tool_link_ee"
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point.position.x
        marker.pose.position.y = point.position.y
        marker.pose.position.z = point.position.z
        marker.pose.orientation.x = point.orientation.x
        marker.pose.orientation.y = point.orientation.y
        marker.pose.orientation.z = point.orientation.z
        marker.pose.orientation.w = point.orientation.w
        marker.scale.x = 0.008
        marker.scale.y = 0.008
        marker.scale.z = 0.008
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker_array.markers.append(marker)
        # print(" enumerating at {}".format(i))
    return marker_array

def visualize_normals(normal_vectors, point_array):
    """
    Visualizes normals as markers in Rviz.

    Args:
        normal_vectors (numpy.ndarray): Array of tangent vectors representing normals.
        point_array (numpy.ndarray): Array of 3D points corresponding to the point cloud.

    Returns:
        None

    Frame:
        Setting the frame to royale_camera_0_optical_frame is important because, 
        similar to our robot's TF, royale_camera_0_optical_frame is base_link and royale_camera_link is the world frame
    """
    display_trajectory = DisplayTrajectory()
    robot_trajectory = RobotTrajectory()
    robot_trajectory.joint_trajectory.points = normal_vectors
    display_trajectory.trajectory.append(robot_trajectory)
    marker_array = MarkerArray()

    for i in range(len(point_array)):
        normal_marker = Marker()
        normal_marker.header.frame_id = "royale_camera_0_optical_frame"
        normal_marker.header.stamp = rospy.Time.now()
        normal_marker.id = i
        normal_marker.type = Marker.ARROW
        normal_marker.action = Marker.ADD

        position_np = point_array[i]
        normal_np = normal_vectors[i]

        normal_marker.pose.position.x = position_np[0]
        normal_marker.pose.position.y = position_np[1]
        normal_marker.pose.position.z = position_np[2]

        # print(f"Marker_{i} position: \n{normal_marker.pose.position}")

        # Check for zero-length tangent vectors before normalization
        if np.linalg.norm(normal_np) != 0:
            normal_np /= np.linalg.norm(normal_np)

            z_axis_local = 1 * normal_np
            y_axis_global = np.array([0, 1, 0]).astype(float)
            x_axis_local = np.cross(z_axis_local, y_axis_global).astype(float)
            y_axis_local = np.cross(z_axis_local, x_axis_local).astype(float)

            x_axis_local /= np.linalg.norm(x_axis_local)
            y_axis_local /= np.linalg.norm(y_axis_local)
            z_axis_local /= np.linalg.norm(z_axis_local)

            rotation_matrix = np.column_stack((x_axis_local, y_axis_local, z_axis_local))
            quaternion_data = quaternion_from_matrix(np.vstack([np.hstack([rotation_matrix, np.zeros((3, 1))]), [0, 0, 0, 1]]))

            quaternion_data /= np.linalg.norm(quaternion_data)

            normal_marker.pose.orientation.x = quaternion_data[0]
            normal_marker.pose.orientation.y = quaternion_data[1]
            normal_marker.pose.orientation.z = quaternion_data[2]
            normal_marker.pose.orientation.w = quaternion_data[3]
        else:
            # If the tangent vector is zero, set an arbitrary orientation
            normal_marker.pose.orientation.w = 1.0

        normal_marker.scale.x = 0.01
        normal_marker.scale.y = 0.001
        normal_marker.scale.z = 0.001
        normal_marker.color.a = 1.0
        normal_marker.color.r = 1.0
        normal_marker.color.g = 1.0
        normal_marker.color.b = 0.0

        marker_array.markers.append(normal_marker)

    marker_pub.publish(marker_array)

def compute_normals(points):
    """
    Computes tangent vectors and normalizes them to obtain normals.

    Args:
        points (numpy.ndarray): Array of 3D points.

    Returns:
        numpy.ndarray: Array of normalized tangent vectors representing normals.

    Description:
        - The function compute_normals takes an array of 3D points (points) as input.
        - 'normal_vectors' is calculated by taking the difference between consecutive points. 
        - np.roll is used to shift the array by one position to get the differences. 
        - This is a numerical approximation of the derivative of the points, representing tangent vectors along the curve.
        - Special handling is applied to the first and last points to ensure that tangent vectors are computed properly.
    """
    # Compute tangent vectors
    normal_vectors = np.roll(points, shift=-1, axis=0) - np.roll(points, shift=1, axis=0)
    normal_vectors[0] = points[1] - points[0]
    normal_vectors[-1] = points[-1] - points[-2]

    # Normalize tangent vectors to get normals
    for i in range(len(normal_vectors)):
        if np.linalg.norm(normal_vectors[i]) != 0:
            normal_vectors[i] /= np.linalg.norm(normal_vectors[i])

        else:
            # If the norm is zero, set an arbitrary vector
            normal_vectors[i] = np.array([0, 0, 1.0])

    return normal_vectors

# Function to move the robot to a specified pose goal
def go_to_pose_goal(pose_goal, i):
    """
    Moves the robot to a specified pose goal.

    Parameters:
        pose_goal (Pose): Target Pose for the end-effector.
        i (int): Index indicating the position in the trajectory (for logging purposes).
    """
    arm_group.set_pose_target(pose_goal)
    rospy.loginfo(f"Planning Trajectory to Pose {i}")
    plan1=arm_group.plan()
    success = arm_group.go(wait=True)

    # rospy.loginfo(f"DONE EXECUTING Planning Trajectory success or failure : {success}")

    # current_joints = arm_group.get_current_joint_values()
    current_joints = arm_group.get_current_pose()
    # return all_close(pose_goal, current_joints, 0.01)

def display_trajectory(plan):
    """
    Displays a MoveIt trajectory.

    Parameters:
        plan (MoveIt plan): MoveIt plan to be displayed.
    """
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)

def point_cloud_callback(cloud_msg):
    """
    Callback function to process incoming point cloud data.

    Args:
        cloud_msg (sensor_msgs.msg.PointCloud2): The input point cloud message.
    """
    try:
        points = np.asarray(list(pc2.read_points(cloud_msg, skip_nans=True)))

        # print(f"size: {points.shape}")
        height = 58
        width = 54

        # Ensure the array is 2D
        if len(points.shape) == 1:
            points = points.reshape(-1, 3)

        # Compute tangent vectors
        normal_vectors = compute_normals(points)
        # print(f"Normal Vectors: \n{normal_vectors}")

        # print(f"Size of segmented point cloud: {points.shape}\nShape of normals computed: {normal_vectors.shape}")

        visualize_normals(normal_vectors, points)

        waypoints = []

        for i, normal in enumerate(normal_vectors):
            z_axis_local = 1 * normal
            y_axis_global = np.array([0, 1, 0]).astype(float)
            x_axis_local = np.cross(z_axis_local, y_axis_global).astype(float)
            x_axis_local /= np.linalg.norm(x_axis_local)
            y_axis_local = np.cross(z_axis_local, x_axis_local).astype(float)
            y_axis_local /= np.linalg.norm(y_axis_local)

            rotation_matrix = np.column_stack((x_axis_local, y_axis_local, z_axis_local))
            quaternion_data = quaternion_from_matrix(np.vstack([np.hstack([rotation_matrix, np.zeros((3, 1))]), [0, 0, 0, 1]]))

            quaternion_data /= np.linalg.norm(quaternion_data)
            # print(f"Quaternion Data: \n{quaternion_data}")

            pose_goal =Pose()
            pose_goal.position.x = normal[0]
            pose_goal.position.y = normal[1]
            pose_goal.position.z = normal[2]
            pose_goal.orientation.x = quaternion_data[0]
            pose_goal.orientation.y = quaternion_data[1]
            pose_goal.orientation.z = quaternion_data[2]
            pose_goal.orientation.w = quaternion_data[3]
            waypoints.append(pose_goal)
        
        # print(f"Shape of way points: {np.array(waypoints)}")
        i = 0
        pos = []
        orien = []
        for i, waypoint in enumerate(waypoints):
            # print(f"Waypoint_{i}: {waypoint}")
            pos.append(waypoint.position)
            orien.append(waypoint.orientation)

        # print(f"\nWay points: \n{waypoints}\n")
        final_waypoints=np.array(waypoints).reshape(width, height, -1)
        print(f"SHape: {final_waypoints.shape}")
        marker_pub_trajectory = rospy.Publisher('trajectory_markers', MarkerArray, queue_size=1, latch=True)
                
        points_to_be_followed = []   
        box_waypoints = []

        box_flag = True

        # Define box waypoints if required
        # if box_flag:
        #     for j in np.arange(0, height, 8):
        #         box_waypoints.append(final_waypoints[0, j, 0])            
                
        #     for i in np.arange(0, width, 8):
        #         box_waypoints.append(final_waypoints[i, height - 1, 0])

        #     for j in np.arange(0, height, 8)[::-1]:
        #         box_waypoints.append(final_waypoints[width - 1, j, 0])
                
        #     for i in np.arange(0, width, 8)[::-1]:
        #         box_waypoints.append(final_waypoints[i, 0, 0])
                
        #     points_to_be_followed = box_waypoints 

        grid_to_be_followed = []   
        toggle = False

        # Define grid waypoints if required
        if path_following:
            for j in np.arange(0, height):
                line_to_be_followed = []
                for i in np.arange(0, width):
                    line_to_be_followed.append(final_waypoints[i, j, 0])
                    points_to_be_followed.append(final_waypoints[i, j, 0])

                if toggle:
                    line_to_be_followed = line_to_be_followed[::-1]
                    toggle = False
                else:
                    toggle = True
                    
                grid_to_be_followed.append(line_to_be_followed)

        print(f"Shape of grid_to_be_followed: {np.array(grid_to_be_followed).shape}")
                    
        marker_array1 = visualize_trajectory(points_to_be_followed, 1.0, 2.0, 0.0)
        marker_pub_trajectory.publish(marker_array1)
        
        print("len(grid_to_be_followed):", len(grid_to_be_followed))
        print(f"Shape of Points to be followed: {np.array(points_to_be_followed).shape}")
            
        static_transforms = []
        normal_points = points_to_be_followed

        for i, pose_goal in enumerate(normal_points):
            static_transforms.append({
                'parent_frame_id': 'royale_camera_0_optical_frame',
                # 'parent_frame_id': 'world',
                'child_frame_id': 'frame_{}'.format(i),
                'translation': [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z],
                'rotation': [pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]
                })

        # Create a static transform broadcaster
        static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        rate = rospy.Rate(10)
        t_list = []

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

        print("Change the position if you want")
        input("******Press enter to display start trajectory ********")

        # print(f"Shape of box_waypoints: {np.array(box_waypoints).shape}")
        # print(f"Shape of grid_to_be_followed: {np.array(grid_to_be_followed).shape}")

        if box_flag:
            normal_points[0].position.z = normal_points[0].position.z + 0.2
            (plan, fraction) = arm_group.compute_cartesian_path(
                [normal_points[0]], 0.01, 0  # waypoints to follow  # eef_step
                )
        else:
            grid_to_be_followed[0][0].position.z = grid_to_be_followed[0][0].position.z + 0.2

            (plan, fraction) = arm_group.compute_cartesian_path(
                [grid_to_be_followed[0][0]], 0.01, 0  # waypoints to follow  # eef_step
                )
            
        print(f"Grid to be followed: {len(grid_to_be_followed[1])}")
        print(f"Fraction: {fraction}")

        input("******Press enter to execute trajectory ********")
        print(f"normal_points[0]: {normal_points[0]}")

        if fraction < 0.1:
            go_to_pose_goal(normal_points[0], -1)
        else:
            st = time.time()

            # Convert the MoveIt plan to a moveit_msgs.msg.RobotTrajectory message
            robot_trajectory = RobotTrajectory()
            robot_trajectory.joint_trajectory = plan.joint_trajectory
            velocity_scaling_factor = 0.05  # Adjust this value as desired
            retime_trajectory = arm_group.retime_trajectory(robot.get_current_state(), robot_trajectory, velocity_scaling_factor)
            print(f"Total time taken after retime: {st - time.time()}")

            arm_group.execute(retime_trajectory, wait=True)
            arm_group.stop()
            arm_group.clear_pose_targets()

        arm_group.stop()
        arm_group.clear_pose_targets()

        if box_flag:
            print("Display trajectory(cartesian_plan)")
            (plan, fraction) = arm_group.compute_cartesian_path(box_waypoints, 0.01, 0, avoid_collisions=True) 
            # waypoints to follow, eef_step, jump_threshold, avoid_collisions = True, path_constraints = None
            display_trajectory(plan)
            print("Fraction:", fraction)
            input("******Press enter to execute trajectory ********")
            st = time.time()

            # Convert the MoveIt plan to a moveit_msgs.msg.RobotTrajectory message
            robot_trajectory = RobotTrajectory()
            robot_trajectory.joint_trajectory = plan.joint_trajectory

            # Adjust the trajectory timing based on velocity scaling
            velocity_scaling_factor = 0.05  # Adjust this value as desired
            retime_trajectory = arm_group.retime_trajectory(robot.get_current_state(), robot_trajectory, velocity_scaling_factor)
            print(f"Total time taken after retime: {st - time.time()}")

            arm_group.execute(retime_trajectory, wait=True)
            arm_group.stop()
            arm_group.clear_pose_targets()

        else:
            input("******Press enter to execute trajectory one by one********")

            for i, waypoint_list in enumerate(grid_to_be_followed):
                (plan, fraction) = arm_group.compute_cartesian_path(waypoint_list, 0.01, 0)  # waypoints to follow  # eef_step

                input("******Press enter to display trajectory ******")
                display_trajectory(plan)
                print("Fraction for plan", i, ":", fraction)

                input("******Press enter to execute trajectory ******")
                st = time.time()

                # Convert the MoveIt plan to a moveit_msgs.msg.RobotTrajectory message
                robot_trajectory = RobotTrajectory()
                robot_trajectory.joint_trajectory = plan.joint_trajectory
                velocity_scaling_factor = 0.05  # Adjust this value as desired
                retime_trajectory = arm_group.retime_trajectory(robot.get_current_state(), robot_trajectory, velocity_scaling_factor)
                print(f"Total time taken after retime: {st - time.time()}")

                arm_group.execute(retime_trajectory, wait=True)
                arm_group.stop()
                arm_group.clear_pose_targets()

            print("Finished code")

        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

    

def main():
    global marker_pub

    marker_pub = rospy.Publisher('/normals_markers', MarkerArray, queue_size=1)

    rospy.Subscriber('/royale_cam0/segmented_point_cloud', PointCloud2, point_cloud_callback)

    rospy.spin()

if __name__ == "__main__":
    main()
