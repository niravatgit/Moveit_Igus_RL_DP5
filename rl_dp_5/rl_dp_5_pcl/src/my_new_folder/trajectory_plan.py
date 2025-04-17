#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
import open3d as o3d

# Initialize MoveIt! and ROS node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('get_ik_solutions', anonymous=True)

# Initialize robot and MoveIt! interface
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "rldp5"
move_group = moveit_commander.MoveGroupCommander(group_name)

# Load point cloud and extract surface normals
pcd = o3d.io.read_point_cloud("rounded_pointcloud.ply")
if len(pcd.points) == 0:
    raise ValueError("Error: Loaded point cloud is empty!")

# Compute normals
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30))
points = np.asarray(pcd.points)

# Convert point cloud to a numpy array
points = np.asarray(pcd.points)

# Get the five specific points from the point cloud
target_position1 = points[283]    # First target position
target_position2 = points[564]   # Second target position
target_position3 = points[208]    
target_position4 = points[470]   # Fourth target position
target_position5 = points[725]  # Fifth target position
target_position6 = points[451] 
target_position7 = points[486]
target_position8 = points[342]

# Print the selected points for verification
print("Target position 1 (index 283):", target_position1)
print("Target position 2 (index 564):", target_position2)
print("Target position 3 (index 208):", target_position3)
print("Target position 4 (index 470):", target_position4)
print("Target position 5 (index 725):", target_position5)
print("Target position 6 (index 451):", target_position6)
print("Target position 7 (index 486):", target_position7)
print("Target position 8 (index 342):", target_position8)

def get_ik_solution(target_position, position_number):
    """Get IK solution for a given target position with specific orientation"""
    target_pose = geometry_msgs.msg.Pose()
    # Use the coordinates directly - don't try to use them as indices
    target_pose.position.x = target_position[0]  # x coordinate
    target_pose.position.y = target_position[1]  # y coordinate
    target_pose.position.z = target_position[2]  # z coordinate
    
    # Set orientation based on position number
    if position_number == 1:  # Special orientation for position 1
        target_pose.orientation.x = 0.36451
        target_pose.orientation.y = 0.60911
        target_pose.orientation.z = -0.36741
        target_pose.orientation.w = 0.60094
    elif position_number == 2:  # Special orientation for position 5
        target_pose.orientation.x = 0.36476 
        target_pose.orientation.y = 0.60904
        target_pose.orientation.z = -0.36769
        target_pose.orientation.w = 0.60069 
    elif position_number == 3:
        target_pose.orientation.x = 0.34865
        target_pose.orientation.y = 0.64899
        target_pose.orientation.z = -0.38111
        target_pose.orientation.w = 0.55858
    elif position_number == 4:
        target_pose.orientation.x = -0.093722 
        target_pose.orientation.y = 0.46028
        target_pose.orientation.z = 0.1046
        target_pose.orientation.w = 0.87659
    elif position_number == 6:
        target_pose.orientation.x = -0.0014799
        target_pose.orientation.y = 0.54635
        target_pose.orientation.z = 0.0015155
        target_pose.orientation.w = 0.83756
    elif position_number == 7:
        target_pose.orientation.x = 0.23481
        target_pose.orientation.y = 0.67093
        target_pose.orientation.z = -0.24105
        target_pose.orientation.w = 0.66077
    elif position_number == 8:
        target_pose.orientation.x = -0.087025
        target_pose.orientation.y = 0.54356
        target_pose.orientation.z = 0.13301
        target_pose.orientation.w = 0.82419
    else:  # Default orientation for other positions
        target_pose.orientation.x = -0.0007
        target_pose.orientation.y = 0.5111
        target_pose.orientation.z = 0.0003
        target_pose.orientation.w = 0.8596

    # Compute IK
    move_group.set_pose_target(target_pose)
    plan_result = move_group.plan()
    
    # Check if planning was successful
    success = plan_result[0] if isinstance(plan_result, tuple) else plan_result
    if success:
        plan = plan_result[1] if isinstance(plan_result, tuple) else plan_result
        if plan.joint_trajectory.points:
            return plan.joint_trajectory.points[-1].positions
    return None
    
def move_to_position(position_number, target_position):
    """Helper function to move to a specific position"""
    rospy.loginfo(f"Planning movement to target position {position_number}...")
    solution = get_ik_solution(target_position, position_number)

    if solution is not None:
        rospy.loginfo(f"IK solution for position {position_number}: {solution}")
        
        # Execute the movement
        rospy.loginfo(f"Moving to target position {position_number}...")
        move_group.go(solution, wait=True)
        rospy.loginfo(f"Movement to position {position_number} completed.")
        return True
    else:
        rospy.logwarn(f"No valid IK solution found for target position {position_number}.")
        return False

# Move through all five positions
positions = [
    (1, target_position1),
    (2, target_position2),
    (3, target_position3),
    (4, target_position4),
    (5, target_position5),
    (6, target_position6),
    (7, target_position7),
    (8, target_position8)
]

for position_number, target_position in positions:
    success = move_to_position(position_number, target_position)
    if not success:
        moveit_commander.roscpp_shutdown()
        sys.exit(1)

moveit_commander.roscpp_shutdown()
