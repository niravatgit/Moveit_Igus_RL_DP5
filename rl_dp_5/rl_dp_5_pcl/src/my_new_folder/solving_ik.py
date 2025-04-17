#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

# Initialize MoveIt! and ROS node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('get_ik_solutions', anonymous=True)

# Initialize robot and MoveIt! interface
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "rldp5"
move_group = moveit_commander.MoveGroupCommander(group_name)

# Load point cloud and extract surface normals
pcd = o3d.io.read_point_cloud("smoothed_pointcloud.ply")
if len(pcd.points) == 0:
    raise ValueError("Error: Loaded point cloud is empty!")

# Compute normals
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30))
normals = np.asarray(pcd.normals)
points = np.asarray(pcd.points)

# Define the target position
target_position = np.array([0.35, 0.07, 0.35])

# Find the closest point in the point cloud
distances = np.linalg.norm(points - target_position, axis=1)
closest_idx = np.argmin(distances)
surface_normal = normals[closest_idx]

# Define orientation search ranges (in degrees)
roll_range = np.arange(-180, 180, 60)
pitch_range = np.arange(-90, 90, 60)
yaw_range = np.arange(-180, 180, 60)

all_solutions = []
iteration_counter = 0

# Iterate over orientations
for roll in roll_range:
    for pitch in pitch_range:
        for yaw in yaw_range:
            iteration_counter += 1
            if iteration_counter % 100 == 0:
                rospy.loginfo(f"{iteration_counter} iterations completed")

            # Convert RPY to quaternion using scipy
            rotation = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)
            quaternion = rotation.as_quat()  # Returns [x, y, z, w]

            # Create a Pose object
            target_pose = geometry_msgs.msg.Pose()
            target_pose.position.x = target_position[0]
            target_pose.position.y = target_position[1]
            target_pose.position.z = target_position[2]
            
            # Set quaternion (order is [x, y, z, w] from scipy)
            target_pose.orientation.x = quaternion[0]
            target_pose.orientation.y = quaternion[1]
            target_pose.orientation.z = quaternion[2]
            target_pose.orientation.w = quaternion[3]

            # Compute IK
            move_group.set_pose_target(target_pose)
            plan_result = move_group.plan()
            
            # Check if planning was successful
            success = plan_result[0] if isinstance(plan_result, tuple) else plan_result
            if success:
                plan = plan_result[1] if isinstance(plan_result, tuple) else plan_result
                if plan.joint_trajectory.points:
                    joint_angles = plan.joint_trajectory.points[-1].positions
                    
                    # Store both joint angles and the original quaternion
                    all_solutions.append({
                        'joint_angles': joint_angles,
                        'quaternion': quaternion
                    })


# Function to get approach vector from quaternion
def quaternion_to_approach_vector(q):
    rotation = R.from_quat([q[0], q[1], q[2], q[3]])
    return rotation.apply([0, 0, 1])  # Z-axis is the approach vector

best_solution = None
best_quaternion = None
best_dot_product = -1

# Evaluate solutions against surface normal
for solution in all_solutions:
    approach_vector = quaternion_to_approach_vector(solution['quaternion'])
    dot_product = np.dot(approach_vector, surface_normal)
    
    if dot_product > best_dot_product:
        best_dot_product = dot_product
        best_solution = solution['joint_angles']
        best_quaternion = solution['quaternion']

if best_solution is not None:
    rospy.loginfo("Best IK solution (aligned to surface normal): {}".format(best_solution))
    rospy.loginfo("Best orientation (quaternion) [x, y, z, w]: {}".format(best_quaternion))
    rospy.loginfo("Dot product with surface normal: {}".format(best_dot_product))
    
    # Convert quaternion to Euler angles for better human readability
    rot = R.from_quat(best_quaternion)
    euler_angles = rot.as_euler('xyz', degrees=True)
    rospy.loginfo("Euler angles (roll, pitch, yaw) in degrees: {}".format(euler_angles))
    
    #You can execute this solution if desired
    move_group.go(best_solution, wait=True)
else:
    rospy.logwarn("No valid IK solution found.")

moveit_commander.roscpp_shutdown()
