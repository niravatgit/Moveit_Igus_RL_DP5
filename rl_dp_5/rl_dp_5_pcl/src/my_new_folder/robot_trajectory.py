#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
import open3d as o3d
import dryve_D1 as dryve
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

# Initialize MoveIt! and ROS node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('real_robot_trajectory_execution')

# Initialize robot and MoveIt! interface
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "rldp5"
move_group = moveit_commander.MoveGroupCommander(group_name)

# Initialize Dryve D1 controllers for real robot
class RealRobotController:
    def __init__(self):
        self.axes = [
            dryve.D1("169.254.0.1", 502, 'Axis 1', -140, -140, 140),
            dryve.D1("169.254.0.2", 502, 'Axis 2', -100, -100, 50),
            dryve.D1("169.254.0.3", 502, 'Axis 3', -115, -115, 115),
            dryve.D1("169.254.0.4", 502, 'Axis 4', -100, -100, 100),
            dryve.D1("169.254.0.5", 502, 'Axis 5', -180, -179, 179)
        ]
        self.speed = 5  # Default speed
        self.accel = 100  # Default acceleration
        
    def execute_trajectory_point(self, positions_radians):
        """Execute a single trajectory point on real robot"""
        positions_degrees = np.rad2deg(positions_radians)
        for i, axis in enumerate(self.axes):
            axis.profile_pos_mode(positions_degrees[i], self.speed, self.accel)
            
    def execute_trajectory(self, trajectory):
        """Execute full trajectory on real robot"""
        for point in trajectory.points:
            self.execute_trajectory_point(point.positions)
            rospy.sleep(point.time_from_start.to_sec())

real_robot = RealRobotController()

def set_pose_orientation(position_number, target_pose):
    """Set orientation based on position number"""
    if position_number == 1:
        target_pose.orientation.x = 0.36451
        target_pose.orientation.y = 0.60911
        target_pose.orientation.z = -0.36741
        target_pose.orientation.w = 0.60094
    elif position_number == 2:
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
    else:
        target_pose.orientation.x = -0.0007
        target_pose.orientation.y = 0.5111
        target_pose.orientation.z = 0.0003
        target_pose.orientation.w = 0.8596

def get_current_joint_state():
    """Get current joint state of the robot"""
    return move_group.get_current_joint_values()

def create_robot_state(joint_positions):
    """Create a RobotState from joint positions"""
    robot_state = RobotState()
    joint_state = JointState()
    joint_state.name = move_group.get_active_joints()
    joint_state.position = joint_positions
    robot_state.joint_state = joint_state
    return robot_state

def get_trajectory(target_position, position_number, start_joints=None):
    """Get joint trajectory for a given target position"""
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = target_position[0]
    target_pose.position.y = target_position[1]
    target_pose.position.z = target_position[2]
    set_pose_orientation(position_number, target_pose)
    
    if start_joints is not None:
        move_group.set_start_state(create_robot_state(start_joints))
    else:
        move_group.set_start_state_to_current_state()
    
    move_group.set_pose_target(target_pose)
    plan = move_group.plan()
    
    if plan[0]:
        return plan[1].joint_trajectory
    return None

def main():
    # Load point cloud
    pcd = o3d.io.read_point_cloud("rounded_pointcloud.ply")
    points = np.asarray(pcd.points)
    
    positions = [
        (1, points[283]), (2, points[564]), (3, points[208]),
        (4, points[470]), (5, points[725]), (6, points[451]),
        (7, points[486]), (8, points[342])
    ]

    rospy.sleep(2.0)
    last_joint_positions = None
    
    for position_number, target_position in positions:
        rospy.loginfo(f"Planning for position {position_number}")
        
        trajectory = get_trajectory(target_position, position_number, start_joints=last_joint_positions)
        
        if trajectory:
            rospy.loginfo(f"Executing trajectory to position {position_number} on real robot")
            
            # Execute on real robot
            real_robot.execute_trajectory(trajectory)
            
            # Update last known joint positions
            last_joint_positions = trajectory.points[-1].positions
            
            # Add some safety margin
            rospy.sleep(1.0)
        else:
            rospy.logerr(f"No valid trajectory for position {position_number}")
            break

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
