#! /usr/bin/env python

from __future__ import print_function
import rospy
import actionlib
import sys
from rldp5_msgs.msg import home_allAction, home_allGoal
from rldp5_msgs.msg import homeAction, homeGoal
from rldp5_msgs.msg import set_des_posAction, set_des_posGoal


def rldp5_robot_action_client(goal_command, joint_index = None, joint_positions = None):

    rospy.loginfo("Multi Action Client Starting....")
    print("Goal Command:", goal_command)
    print("Joint Positions:",joint_positions)
    print("Joint Index:",joint_index)

    if goal_command == 'home_all':
        client = actionlib.SimpleActionClient('home_all_action', home_allAction)
        goal = home_allGoal()
        print(goal, type(goal.home_all))
        post_proc_goal(client, goal)

    elif goal_command == 'home_joint' and joint_index is not None:
        print("Came into home_joint conditional statement")
        client = actionlib.SimpleActionClient('home_joint_action', homeAction)
        print('joint_number', joint_index)
        goal = homeGoal()
        goal.joint_index = joint_index
        print(goal, type(goal.joint_index))
        post_proc_goal(client, goal)
        
    elif goal_command == 'joint_positions' and joint_positions is not None:
        print(joint_positions)
        print("Came into joint_positions conditional statement")
        client = actionlib.SimpleActionClient('joint_positions_action', set_des_posAction)
        goal = set_des_posGoal()
        goal.joint_states = joint_positions
        post_proc_goal(client, goal)
        
    else:
        rospy.loginfo("Invalid goal command")

def post_proc_goal(client, goal):

    client.wait_for_server()
    client.send_goal(goal)
    rospy.loginfo("Sent %s goal" %goal)

    # Wait for the server to finish performing the action
    client.wait_for_result()

    # Print the result of executing the action
    return client.get_result()
    

if __name__ == '__main__':

    try: 
        rospy.init_node('rldp5_Robot_ac_py')
        if len(sys.argv) == 2:
            goal_command = sys.argv[1]
            print(goal_command)
            result = rldp5_robot_action_client(goal_command, None, None)
            rospy.loginfo(result)

        elif len(sys.argv) == 3 and sys.argv[1] == 'home_joint':
            goal_command = sys.argv[1]
            joint_index = int(sys.argv[2])
            print('goal:', goal_command, 'joint_index:', joint_index)
            result = rldp5_robot_action_client(goal_command, joint_index, None)
            rospy.loginfo(result)

        elif len(sys.argv) == 3 and sys.argv[1] == 'joint_positions':
            try:
                joint_positions = eval(sys.argv[2])
                print('joint_positions:', joint_positions, type(joint_positions))
                if isinstance(joint_positions, list):
                    goal_command = sys.argv[1]         
                    print('goal:', goal_command, 'joint_positions:', joint_positions)
                    result = rldp5_robot_action_client(goal_command, None, joint_positions)
                    rospy.loginfo(result)
            except Exception as e:
                rospy.loginfo(f"Error parsing joint_state arguments: {e}")
        else:
            rospy.loginfo("Provide the arguments properly!!!")
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
