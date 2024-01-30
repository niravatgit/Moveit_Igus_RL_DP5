#!/usr/bin/env python3
import rospy
import sys
from sklearn.neighbors import NearestNeighbors
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
import numpy as np
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from tf.transformations import quaternion_from_matrix
import tf2_ros
import time
import moveit_commander
import moveit_msgs.msg
import open3d as o3d


## First initialize `moveit_commander`_ and a `rospy`_ node:
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('generating_custom_trajectory', anonymous=True)
tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(12))
tf_listener = tf2_ros.TransformListener(tf_buffer)

box_flag = False
path_following = True

# Set up the robot arm group and planning scene
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

# Name of the group can be found at the "/home/inspire-0/moveit_igus_ws/src/Moveit_Igus_RL_DP5/rl_dp_5/rl_dp_5_moveit/config/igus_robolink_rl_dp_5.srdf" file
group_name = 'rldp5'
arm_group = moveit_commander.MoveGroupCommander(group_name)
# arm_group.set_named_target("upright")
# plan1 = arm_group.go()
# rospy.sleep(5)

display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)

planning_frame = arm_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

arm_group.set_goal_orientation_tolerance(0.1)

# Set the start state of the robot
arm_group.set_start_state_to_current_state()
arm_group.set_max_velocity_scaling_factor(0.05)

# Sometimes for debugging it is useful to print the entire state of the robot:
print("============ Printing robot state")
# print(robot.get_current_state())
print("")

start_time = time.time()

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

    marker_array = MarkerArray()
    for i, point in enumerate(waypoints):
        marker = Marker()
        # marker.header.frame_id = "tool_link_ee"
        marker.header.frame_id = "royale_camera_0_optical_frame"
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
        marker.scale.x = 0.001
        marker.scale.y = 0.001
        marker.scale.z = 0.001
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker_array.markers.append(marker)
        # print(" enumerating at {}".format(i))
    return marker_array



def compute_normals(points, epsilon=1e-3):
    """
    Compute normals for a point cloud using numerical differentiation.

    Parameters:
        points (numpy.ndarray): Array of 3D points representing the point cloud.
        epsilon (float): Step size for numerical differentiation.

    Returns:
        numpy.ndarray: Array of normalized tangent vectors representing normals.

    Description:
        This function estimates normals for each point in the input point cloud using
        numerical differentiation. It approximates the tangent vector at each point
        and normalizes it to obtain the normal vector.

    Args:
        points (numpy.ndarray): Array of 3D points.
        epsilon (float, optional): Step size for numerical differentiation (default is 1e-3).

    Returns:
        numpy.ndarray: Array of normalized tangent vectors representing normals.

    Note:
        - The function uses a simple numerical differentiation approach by selecting
          three neighboring points for each point in the point cloud.
        - The `epsilon` parameter controls the step size in numerical differentiation.
    """
    normals = []

    for i in range(len(points)):
        # Select three points for numerical differentiation
        p0 = points[max(0, i - 1)]
        p1 = points[i]
        p2 = points[min(len(points) - 1, i + 1)]

        # Numerical differentiation to estimate tangent vector
        tangent_vector = (p2 - p0) / (2 * epsilon)

        # Check for zero-length tangent vectors before normalization
        if np.linalg.norm(tangent_vector) != 0:
            # Normalize tangent vector to obtain normal
            normal = tangent_vector / np.linalg.norm(tangent_vector)
            normals.append(normal)
        else:
            # If the tangent vector is zero, set an arbitrary normal vector
            normals.append(np.array([0.0, 0.0, 1.0]))

    return np.array(normals)


def go_to_joint_state():
    # Copy class variables to local variables to make the code clearer.
    joint_goal = JointState()
    joint_goal.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
    joint_goal.position = [0, 0, np.deg2rad(-90), np.deg2rad(-45), 0]

    try:
        input("======== Upright Position of the robot is a Singularity, Press enter to move the robot to home position ========")
        arm_group.go(joint_goal, wait=True)
        arm_group.stop()
        print("Moved to home position\n")
    except moveit_commander.exception.MoveItCommanderException as e:
        print(f"Error setting joint target: {e}")

# Function to move the robot to a specified pose goal
def go_to_pose_goal(waypoint):
    """
    Moves the robot to a specified pose goal.

    Parameters:
        waypoint (Pose): The pose goal for the end-effector.
    """

    arm_group.set_pose_target(waypoint)

    success = arm_group.go(wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()

    rospy.loginfo("Finished executing trajectory.")

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

def execute_plan(cartesian_plan):
    arm_group.execute(cartesian_plan, wait=True)

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
            print("Reshaping the Point Cloud np array")
            points = points.reshape(-1, 3)

        # Compute tangent vectors
        normal_vectors = compute_normals(points)

        # Moving to home position
        go_to_joint_state()

        print(f"Size of segmented point cloud: {points.shape}\nShape of normals computed: {normal_vectors.shape}\n")

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

            pose_goal = Pose()
            pose_goal.position.x = points[i][0]
            pose_goal.position.y = points[i][1]
            pose_goal.position.z = points[i][2]
            pose_goal.orientation.x = quaternion_data[0]
            pose_goal.orientation.y = quaternion_data[1]
            pose_goal.orientation.z = quaternion_data[2]
            pose_goal.orientation.w = quaternion_data[3]

            # print(f"Type of pose_goal: {type(pose_goal)}")
            waypoints.append(pose_goal)
        
        print(f"First point is : {waypoints[0]}")
        print(f"Shape of way points: {np.array(waypoints).shape}")

        # print(f"\nWay points: \n{waypoints}\n")
        final_waypoints=np.array(waypoints).reshape(width, height, -1)
        # print(f"SHape: {final_waypoints.shape}\nfinal way points: {final_waypoints}")
        marker_pub_trajectory = rospy.Publisher('trajectory_markers', MarkerArray, queue_size=1, latch=True)
                
        points_to_be_followed = []   

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

        way_points_gen = np.array(grid_to_be_followed).reshape(width*height, -1)
            
        static_transforms = []
        normal_points = points_to_be_followed

        for i, pose_goal in enumerate(normal_points):
            translation = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z]
            rotation = [pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]

            static_transforms.append({
                'parent_frame_id': 'royale_camera_0_optical_frame',
                'child_frame_id': 'frame_{}'.format(i),
                'translation': translation,
                'rotation': rotation
            })

        input("******Press enter to display start trajectory ********")
        marker_array1 = visualize_trajectory(normal_points, 1.0, 2.0, 0.0)
        marker_pub_trajectory.publish(marker_array1)

        print(f"Length of points to be followed: {len(list((way_points_gen)))} & type: {type(way_points_gen)}")

        way_points_gen = way_points_gen.tolist()

        
        for pose in way_points_gen:
            go_to_pose_goal(pose)

        box_flag = True
        if box_flag:
            # for i, normal_point in enumerate(normal_points):
            #     normal_point.position.z += 0.2
                # print("ajdfl: ", normal_points[i].position.z)
            print("Computing Cartesian Path")
            (plan, fraction) = arm_group.compute_cartesian_path(points_to_be_followed, 0.01, 0.1)
            print("Computed Cartesian Path")

        else:
            grid_to_be_followed[0][0].position.z = grid_to_be_followed[0][0].position.z + 0.2

            (plan, fraction) = arm_group.compute_cartesian_path(
                [grid_to_be_followed[0][0]], 0.01, 0  # waypoints to follow  # eef_step
                )

        # input("******Press enter to execute trajectory ********")
        if fraction < 0.1:
            display_trajectory(plan)
            execute_plan(plan)
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
            (plan, fraction) = arm_group.compute_cartesian_path(points_to_be_followed, 0.01, 0, avoid_collisions=True) 
            # waypoints to follow, eef_step, jump_threshold, avoid_collisions = True, path_constraints = None
            display_trajectory(plan)
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
                (plan, fraction) = arm_group.compute_cartesian_path(waypoint_list, 0.01, 0.1)  # waypoints to follow  # eef_step

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
                print(f"Total time taken after retime: {st - time.time()}\n")

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

    