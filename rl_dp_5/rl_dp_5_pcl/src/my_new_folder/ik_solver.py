#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
from scipy.spatial import KDTree

# Initialize MoveIt! and ROS node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ik_solver_with_pointcloud_perturbations', anonymous=True)

# Initialize robot interface
robot = moveit_commander.RobotCommander()
move_group = moveit_commander.MoveGroupCommander("rldp5")

# Configure IK solver settings
move_group.set_goal_position_tolerance(0.02)  # 2 cm tolerance
move_group.set_goal_orientation_tolerance(0.2)  # ~11 degrees tolerance

# Target position that needs solving (from smoothed_pointcloud.ply)
target_position = np.array([0.39, 0.04, 0.33])

# Known good orientation from nearby successful pose
good_orientation = [0.26736, 0.6589, -0.27338, 0.64779]  # x,y,z,w

# Load point cloud and build KDTree for nearest neighbor search
def load_point_cloud_and_build_kdtree(filename):
    pcd = o3d.io.read_point_cloud(filename)
    if len(pcd.points) == 0:
        raise ValueError("Error: Loaded point cloud is empty!")
    points = np.asarray(pcd.points)
    kdtree = KDTree(points)
    return points, kdtree

try:
    cloud_points, point_kdtree = load_point_cloud_and_build_kdtree("rounded_pointcloud.ply")
except Exception as e:
    rospy.logerr(f"Failed to load point cloud: {str(e)}")
    sys.exit(1)

def print_orientation(quat):
    """Print orientation in both quaternion and Euler angles"""
    euler = R.from_quat(quat).as_euler('xyz', degrees=True)
    print("Orientation (Quaternion): x: %.4f, y: %.4f, z: %.4f, w: %.4f" % 
          (quat[0], quat[1], quat[2], quat[3]))
    print("Orientation (Euler angles - degrees): Roll: %.1f, Pitch: %.1f, Yaw: %.1f" %
          (euler[0], euler[1], euler[2]))

def solve_with_pointcloud_perturbations(position, orientation, max_attempts=25, radius=0.08):
    """Try IK solutions using nearby points from the point cloud"""
    # Find all points within radius of target position
    nearby_indices = point_kdtree.query_ball_point(position, radius)
    
    if not nearby_indices:
        rospy.logwarn(f"No points found within {radius}m of target position")
        return None, None
        
    # Try the exact target first if it exists in the point cloud
    exact_match_idx = point_kdtree.query(position)[1]
    if np.linalg.norm(cloud_points[exact_match_idx] - position) < 0.001:
        nearby_indices.insert(0, exact_match_idx)
    
    # Try points in order of distance from target
    nearby_indices.sort(key=lambda idx: np.linalg.norm(cloud_points[idx] - position))
    
    for attempt, idx in enumerate(nearby_indices[:max_attempts]):
        perturbed_pos = cloud_points[idx]
        
        # Create pose with fixed orientation
        pose = geometry_msgs.msg.Pose()
        pose.position.x = perturbed_pos[0]
        pose.position.y = perturbed_pos[1]
        pose.position.z = perturbed_pos[2]
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]
        
        # Try to solve IK
        move_group.set_pose_target(pose)
        success, plan, _, _ = move_group.plan()
        
        if success:
            joint_solution = plan.joint_trajectory.points[-1].positions
            rospy.loginfo(f"\nAttempt {attempt+1} SUCCESS")
            rospy.loginfo(f"Used point cloud position: {perturbed_pos}")
            print_orientation(orientation)
            return joint_solution, perturbed_pos
        else:
            rospy.loginfo(f"\nAttempt {attempt+1} failed with position: {perturbed_pos}")
    
    return None, None

# Print initial target information
rospy.loginfo("\n=== IK SOLVER WITH POINT CLOUD PERTURBATIONS ===")
rospy.loginfo(f"Target position: {target_position}")
rospy.loginfo("Using fixed orientation from nearby successful pose:")
print_orientation(good_orientation)
rospy.loginfo(f"Loaded {len(cloud_points)} points from point cloud")

# Attempt to solve with point cloud perturbations
solution, used_position = solve_with_pointcloud_perturbations(target_position, good_orientation)

if solution is not None:
    rospy.loginfo("\n=== FINAL SOLUTION ===")
    rospy.loginfo(f"Original target position: {target_position}")
    rospy.loginfo(f"Used position from point cloud: {used_position}")
    rospy.loginfo("Used orientation:")
    print_orientation(good_orientation)
    rospy.loginfo(f"Joint solution: {solution}")
    
    # Execute the solution
    rospy.loginfo("Executing movement...")
    move_group.go(solution, wait=True)
    rospy.loginfo("Movement executed successfully")
else:
    rospy.logwarn("\n=== FAILED TO FIND SOLUTION ===")
    rospy.loginfo(f"After {max_attempts} attempts, no IK solution found")
    rospy.loginfo("Suggestions:")
    rospy.loginfo("1. Increase max_attempts")
    rospy.loginfo("2. Increase search radius")
    rospy.loginfo("3. Try slight orientation adjustments")

moveit_commander.roscpp_shutdown()
