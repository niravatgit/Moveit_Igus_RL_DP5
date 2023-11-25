#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import ExecuteTrajectoryActionResult
import dryve_D1 as dryve
import numpy as np

speed = 5
accel = 100
homespeed = 5
homeaccel = 100

class TrajectoryHandler:
    def __init__(self, robot):
        self.robot = robot

    def execute_trajectory(self, trajectory_points):
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = [f'Axis {i + 1}' for i in range(len(self.robot.axis_controller))]

        for point in trajectory_points:
            joint_trajectory_point = JointTrajectoryPoint()
            joint_trajectory_point.positions = point
            joint_trajectory.points.append(joint_trajectory_point)

        rospy.loginfo("Executing trajectory...")
        # Assuming you have a publisher for the trajectory execution result
        result_publisher = rospy.Publisher('/move_group/result', ExecuteTrajectoryActionResult, queue_size=10)

        # Publish the result (you may need to adjust this based on your actual result)
        result = ExecuteTrajectoryActionResult()
        result_publisher.publish(result)

class Rl_DP_5:
    def __init__(self):
        # Initialize your 5 D1 axes here
        Aaxis = dryve.D1("169.254.0.1", 502, 'Axis 1', -140, -140, 140)
        Baxis = dryve.D1("169.254.0.2", 502, 'Axis 2', -100, -100, 50)
        Caxis = dryve.D1("169.254.0.3", 502, 'Axis 3', -115, -115, 115)
        Daxis = dryve.D1("169.254.0.4", 502, 'Axis 4', -100, -100, 100)
        Eaxis = dryve.D1("169.254.0.5", 502, 'Axis 5', -180, -179, 179)

        self.axis_controller = [Aaxis, Baxis, Caxis, Daxis, Eaxis]
        print('Created dryve interfaces')

        # ROS publisher for fake joint states
        self.fake_joint_states_publisher = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=10)

        # ROS subscriber for joint states
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)

    def set_target_position(self, axis, desired_absolute_position):
        if 0 <= axis < len(self.axis_controller):
            self.axis_controller[axis].profile_pos_mode(desired_absolute_position, speed, accel)

    def home(self, axis):
        print(f"Started homing Axis {axis + 1}")
        self.axis_controller[axis].homing(homespeed, homeaccel)

    def home_all(self):
        for axis in self.axis_controller:
            print(f"Started homing {axis.Axis}")
            axis.homing(homespeed, homeaccel)

    def get_current_position(self, axis):
        return self.axis_controller[axis].getPosition()

    def joint_states_callback(self, joint_states_msg):
        # Extract joint positions from joint_states_msg and set them as the target positions
        for i, position in enumerate(joint_states_msg.position):
            self.set_target_position(i, position)

        # Log or print the received joint states for verification
        rospy.loginfo("Received Joint States: %s", joint_states_msg.position)

        # Assuming you have a MoveIt interface to get planned trajectory points
        # Example random trajectory
        random_trajectory = np.random.rand(len(self.axis_controller)) * 2.0 - 1.0  # Adjust as needed
        planned_trajectory_points = [random_trajectory]  # Replace this with your actual planned trajectory

        # Execute the trajectory
        trajectory_handler.execute_trajectory(planned_trajectory_points)

if __name__ == "__main__":
    rospy.init_node('robot_controller_node')
    print('Initialized an object for the robot')
    robot = Rl_DP_5()

    trajectory_handler = TrajectoryHandler(robot)

    rate = rospy.Rate(1)  # Adjust the rate as needed

    while not rospy.is_shutdown():
        # 1. Get the robot's position and publish it to /move_group/fake_controller_joint_states
        current_position = [robot.get_current_position(i) for i in range(len(robot.axis_controller))]

        joint_state_msg = JointState()
        joint_state_msg.position = current_position
        robot.fake_joint_states_publisher.publish(joint_state_msg)

        # 2. Sleep to control the publishing rate
        rate.sleep()
