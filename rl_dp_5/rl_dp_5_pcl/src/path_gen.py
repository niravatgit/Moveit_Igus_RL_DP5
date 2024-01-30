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
import copy
import moveit_commander
import moveit_msgs.msg
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
from moveit_commander import conversions
import open3d as o3d

try:
    from math import pi
except ImportError:
    pi = 3.141592653589793

from std_msgs.msg import String

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("arm_group_python_interface_tutorial", anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "rldp5"
arm_group = moveit_commander.MoveGroupCommander(group_name)

global display_trajectory_publisher
display_trajectory_publisher = None  # Initializing to None

def visualize_normals(normal_vectors, point_array):

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

def visualize_trajectory(plan):
    global display_trajectory_publisher
    if display_trajectory_publisher is not None:
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)
    else:
        rospy.logerr("Display trajectory publisher is not initialized.")

    return display_trajectory.trajectory

def calculate_normals(points):
    # Create an Open3D point cloud
    o3d_cloud = o3d.geometry.PointCloud()
    o3d_cloud.points = o3d.utility.Vector3dVector(points[:, :3])

    # Estimate normals using Open3D (CPU version)
    o3d_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))

    # Get the normals as a numpy array
    normals = np.asarray(o3d_cloud.normals)

    normal_vectors = np.asarray(normals)

    return normal_vectors

def compute_normals(points, epsilon=1e-3):
    """
    For each point p1 in the input array:
      Compute tangent vector: t = (p2 - p0) / (2 * epsilon)

      If ||t|| != 0:
          Normalize tangent vector to obtain normal vector: n = t / ||t||
      Else:
          Set an arbitrary normal vector: n = [0.0, 0.0, 1.0]

    Reference: https://math.libretexts.org/Bookshelves/Calculus/Supplemental_Modules_(Calculus)/Vector_Calculus/2%3A_Vector-Valued_Functions_and_Motion_in_Space/2.4%3A_The_Unit_Tangent_and_the_Unit_Normal_Vectors
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

def go_to_pose_goal(fixed_pose):
    move_group = arm_group
    eef = move_group.get_end_effector_link()
    # print("End Effector Link:", eef)

    # print(f"in the go to pose goal function: {type(fixed_pose)}")

    pose_goal = Pose()
    pose_goal.position.x = fixed_pose.position.x
    pose_goal.position.y = fixed_pose.position.y
    pose_goal.position.z = fixed_pose.position.z
    pose_goal.orientation.x = fixed_pose.orientation.x
    pose_goal.orientation.y = fixed_pose.orientation.y
    pose_goal.orientation.z = fixed_pose.orientation.z
    pose_goal.orientation.w = fixed_pose.orientation.w

    move_group.set_pose_target(pose_goal)
    success = move_group.go(wait=True)
    # print("Moved to the pose")

    move_group.stop()
    move_group.clear_pose_targets()


def plan_cartesian_path(waypoints):

    print("Entered plan_cartesian_path")

    (plan, fraction) = arm_group.compute_cartesian_path(
            waypoints,
            0.01,  # eef_step
            0.0,   # jump_threshold
            avoid_collisions=True,
            )
    print(f"Fraction: {fraction}")
    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

def execute_plan(plan):
    # arm_group.set_planner_id("geometric::RRTConnect")
    # arm_group.set_planner_id("RRTConnectkConfigDefault")
    arm_group.set_num_planning_attempts(10)
    arm_group.set_max_velocity_scaling_factor(0.1)
    arm_group.set_goal_tolerance(0.01)

    # Replace arm_group.go(plan, wait=True) with arm_group.execute(plan, wait=True)
    try:
        arm_group.execute(plan, wait=True)
    
    except moveit_commander.exception.MoveItCommanderException as e:
        print(f"Error executing the plan: {e}")
    arm_group.stop()
    arm_group.clear_pose_targets()


def transform_point_to_world(tf_buffer, frame_id):
    try:
        transform = tf_buffer.lookup_transform("world", frame_id, rospy.Time(), rospy.Duration(1.0))
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        # print(f"Translation: {translation}\nRotation: {rotation}")

        pose = Pose()
        pose.position.x = translation.x
        pose.position.y = translation.y
        pose.position.z = translation.z
        pose.orientation.x = rotation.x
        pose.orientation.y = rotation.y
        pose.orientation.z = rotation.z
        pose.orientation.w = rotation.w

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr("TF lookup failed: %s", str(e))
        return None
    
    return pose

def point_cloud_callback(cloud_msg):
    try:
        points = np.asarray(list(pc2.read_points(cloud_msg, skip_nans=True)))

        # print(f"size: {points.shape}")
        height = 51
        width = 43

        # Ensure the array is 2D
        if len(points.shape) == 1:
            print("Reshaping the Point Cloud np array")
            points = points.reshape(-1, 3)

        # Compute tangent vectors
        normal_vectors = calculate_normals(points)
            
        # Compute normals using Open3D
        # normal_vectors = calculate_normals(points)

        # Moving to home position
        # go_to_joint_state()

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
        
        # print(f"First point is : {waypoints[0]}")
        # print(f"Shape of way points: {np.array(waypoints).shape}")
            
        final_waypoints=np.array(waypoints).reshape(width, height, -1)
        # print(f"SHape: {final_waypoints.shape}\nfinal way points: {final_waypoints}")
                
        points_to_be_followed = []   
        grid_to_be_followed = []   
        toggle = False
        path_following = True

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

        # Convert waypoints to a list of lists
        waypoints_list = way_points_gen.tolist()
        print(f'length of waypoints_list: {len(waypoints_list)}')

        flat_list = [item for sublist in waypoints_list for item in sublist]

        # print(len(flat_list)) 
            
        static_transforms = []
        normal_points = flat_list

        for i, pose_goal in enumerate(normal_points):
            # pose_goal = pose_goal[0]
            # print(pose_goal)
            translation = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z]
            rotation = [pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]

            static_transforms.append({
                'parent_frame_id': 'royale_camera_0_optical_frame',
                'child_frame_id': 'frame_{}'.format(i),
                'translation': translation,
                'rotation': rotation
            })

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

        # new_waypoints = []
        # for i, normal in enumerate(normal_vectors):
        #     frame_id = "frame_{}".format(i)
        #     # Transform point from the camera frame to the world frame
        #     pose_goal = transform_point_to_world(tf_buffer, frame_id)

        #     if pose_goal is not None:
        #         new_waypoints.append(pose_goal)
                # go_to_pose_goal(pose_goal)

        # for waypoint in normal_points:
        #     #  print(len(waypoint))
        #      go_to_pose_goal(waypoint)

        pose1 = Pose()
        pose1.position.x = 0.1
        pose1.position.y = 0.0
        pose1.position.z = 0.0
        pose1.orientation.x = 0.0
        pose1.orientation.y = 0.0
        pose1.orientation.z = 0.0
        pose1.orientation.w = 1.0

        arm_group.set_pose_target(pose1)
        plan1=arm_group.go(wait=True)

        cartesian_plan, fraction = plan_cartesian_path([pose1])

        input("******Press enter to display start trajectory ********")
        traj = visualize_trajectory(cartesian_plan)

        execute_plan(cartesian_plan)

        

    except rospy.ROSInterruptException as e:
         print(f"Exiting with the exception {e}")


if __name__ == "__main__":
    global marker_pub

    marker_pub = rospy.Publisher('/normals_markers', MarkerArray, queue_size=1)
    display_trajectory_publisher = rospy.Publisher(
        "/arm_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
    )
    rospy.Subscriber('/royale_cam0/segmented_point_cloud', PointCloud2, point_cloud_callback)

    rospy.spin()