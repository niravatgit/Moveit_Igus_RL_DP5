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

    available_commands = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'home_all', 'set_shutdn', 'set_swon', 'set_op_en']
    print("The avaialble commands to send to the Action Server:\n", available_commands)

    rospy.loginfo("sending goal...")
    # Sends the goal to the action server.
    print(goal_command, type(goal_command))
    goal = rldp5_robotGoal(goal_command)
    print("Goal: ", goal, type(goal))
    client.send_goal(goal_command)
    rospy.loginfo("Goal has been sent to the action server.")

    # Waits for the server to finish performing the action.
    rospy.loginfo("Waiting for result...")
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A CounterWithDelayResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        if len(sys.argv) == 2:
            rospy.init_node('rldp5_Robot_ac_py')
            # goal_command = sys.argv[1]
            a = sys.argv[1]
            available_commands = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'home_all', 'set_shutdn', 'set_swon', 'set_op_en', 'upright']
            if a in available_commands:
                goal_command = rldp5_robotGoal(command=[a])

            else:
                print("as a", a)
                n = len(sys.argv[1])
                print("n: ", n) 
                a = sys.argv[1][1:n-1]
                print(a)
                a = a.split(',') 
                goal_command = rldp5_robotGoal(command=a)

            print(goal_command)
            result = rldp5_robot_action_client(goal_command)
            rospy.loginfo(result)

        else:
            rospy.loginfo("Provide the arguments properly!!!")
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)