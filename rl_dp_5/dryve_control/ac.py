#! /usr/bin/env python
from __future__ import print_function
import rospy
import actionlib
import sys
from rldp5_msgs import rldp5_robotAction, rldp5_robotGoal

def rldp5_robot_action_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (rldp5_robotAction) to the constructor.
    client = actionlib.SimpleActionClient('RLDP5 Robot Action', rldp5_robotAction)

    # Waits until the action server has started up and started
    # listening for goals.
    rospy.loginfo("Waiting for action server to come up...")
    client.wait_for_server()

    command = input("")

    # Creates a goal to send to the action server.
    goal = rldp5_robotGoal(command)

    # Sends the goal to the action server.
    client.send_goal(goal)

    rospy.loginfo("Goal has been sent to the action server.")

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A CounterWithDelayResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('rldp5_Robot_ac')
        result = rldp5_robot_action_client()
        rospy.loginfo(result.result_message)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)