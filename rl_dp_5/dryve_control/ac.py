#! /usr/bin/env python
from __future__ import print_function
import rospy
import actionlib
import sys
from rldp5_msgs.msg import rldp5_robotAction, rldp5_robotGoal

class RLDP5RobotClient:
    def __init__(self):
        """
        Initializes the RLDP5RobotClient.

        The client is used to send goals to the 'ros_action_commands' action server.

        Attributes:
        - client: SimpleActionClient for the 'ros_action_commands' action server.
        - available_commands: List of valid commands that the robot can execute.
        """
        self.client = actionlib.SimpleActionClient('ros_action_commands', rldp5_robotAction)
        self.available_commands = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'home_all', 'set_shutdn']
        print("The avaialble commands to send to the Action Server:\n", self.available_commands)

    def wait_for_server(self):
        
        # Waits for the action server to come up before sending goals.

        rospy.loginfo("Waiting for action server to come up...")
        self.client.wait_for_server()

    def create_goal(self, command):
        """
        Creates a goal for the robot action based on the provided command.

        Args:
        - command (str): The command for the robot action.

        Returns:
        - rldp5_robotGoal: The goal for the robot action.
        """
        
        if command in self.available_commands:
            return rldp5_robotGoal(command)
        else:
            print("Invalid command. Please check the available commands.")
            sys.exit(1)

    def send_goal(self, goal):
        """
        Sends the provided goal to the action server.

        Args:
        - goal (rldp5_robotGoal): The goal to be sent to the action server.
        """
        rospy.loginfo("Sending goal...")
        self.client.send_goal(goal)
        rospy.loginfo("Goal has been sent to the action server.")

    def wait_for_result(self):
        """
        Waits for the action server to finish performing the action.
        """
        rospy.loginfo("Waiting for result...")
        self.client.wait_for_result()

    def get_result(self):
        """
        Retrieves and returns the result of executing the action.

        Returns:
        - result: The result of executing the action.
        """
        return self.client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('rldp5_Robot_ac_py')
        if len(sys.argv) == 2:
            goal_command = sys.argv[1]

            # Create and initialize the RLDP5RobotClient
            robot_client = RLDP5RobotClient()
            robot_client.wait_for_server()

            # Create a goal based on the provided command
            goal = robot_client.create_goal(goal_command)

            # Send the goal to the action server
            robot_client.send_goal(goal)

            # Wait for the action server to finish performing the action
            robot_client.wait_for_result()

            # Get and print the result of executing the action
            result = robot_client.get_result()
            rospy.loginfo(result)
        else:
            print('Pass some kind of command to the robot.')
    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)
