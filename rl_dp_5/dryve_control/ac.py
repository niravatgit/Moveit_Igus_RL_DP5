#! /usr/bin/env python
from __future__ import print_function
import rospy
import actionlib
import sys
from rldp5_msgs.msg import rldp5_robotAction, rldp5_robotGoal

def rldp5_robot_action_client(goal_command):
    # Creates the SimpleActionClient, passing the type of the action
    # (rldp5_robotAction) to the constructor.
    rospy.loginfo("client action...")
    client = actionlib.SimpleActionClient('RLDP5_Robot_Action', rldp5_robotAction)

    # Waits until the action server has started up and started
    # listening for goals.
    rospy.loginfo("Waiting for action server to come up...")
    client.wait_for_server()

    # Creates a goal to send to the action server.

    available_commands = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'home_all', 'set_shutdn']
    print("The avaialble commands to send to the Action Server:\n", available_commands)

    if goal_command in available_commands:
        rospy.loginfo("sending goal...")
        # Sends the goal to the action server.
        print("Given goal command : ", goal_command)
        client.send_goal(goal_command)
        rospy.loginfo("Goal has been sent to the action server.")

    else:
        print("Invalid command. Please check the available commands.")
        sys.exit(1)

    # Waits for the server to finish performing the action.
    rospy.loginfo("Waiting for result...")
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A CounterWithDelayResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('rldp5_Robot_ac.py')
        if len(sys.argv) == 2:
            goal_command = sys.argv[1]
            
            result = rldp5_robot_action_client(goal_command)
            rospy.loginfo(result)

        else:
            rospy.loginfo("Provide the arguments properly!!!")
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)