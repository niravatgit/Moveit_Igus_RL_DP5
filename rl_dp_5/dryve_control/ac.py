#! /usr/bin/env python
from __future__ import print_function
import rospy
import actionlib
import sys
from rldp5_msgs.msg import rldp5_robotAction, rldp5_robotGoal

def rldp5_robot_action_client(command):
    # Creates the SimpleActionClient, passing the type of the action
    # (rldp5_robotAction) to the constructor.
    rospy.loginfo("client action...")
    client = actionlib.SimpleActionClient('ros_action_commands', rldp5_robotAction)

    # Waits until the action server has started up and started
    # listening for goals.
    
    rospy.loginfo("Waiting for action server to come up...")
    client.wait_for_server()

    # Creates a goal to send to the action server.
    
    if len(sys.argv)!= []:
        num = len(sys.argv)-1
        print("There are 3 arguments available. They are:")
        print("1.'joint_1' '\n2. 'joint_2' \n3. 'joint_3' \n4. 'joint_4' \n5. 'joint_5' \n6. 'home_all'")
    else:
        print('Pass some kind of command to robot')
        
    if command == 'joint_1':
        goal = rldp5_robotGoal(command)
    
    elif command == 'joint_2':
        goal = rldp5_robotGoal(command)
        
    elif command == 'joint_3':
        goal = rldp5_robotGoal(command)
        
    elif command == 'joint_4':
        goal = rldp5_robotGoal(command)
        
    elif command == 'joint_5':
        goal = rldp5_robotGoal(command)
        
    elif command == 'home_all':
        goal = rldp5_robotGoal(command)

    else:
        print("Invalid command. Please check the above available commands")
        sys.exit(1)

    
    rospy.loginfo("sending goal...")
    
    

    # Sends the goal to the action server.
    client.send_goal(goal)

    rospy.loginfo("Goal has been sent to the action server.")

    # Waits for the server to finish performing the action.
    rospy.loginfo("Waiting for result...")
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A CounterWithDelayResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS
            
        rospy.init_node('rldp5_Robot_ac_py')
        if len(sys.argv) == 2:
            goal_command = sys.argv[1]
            result = rldp5_robot_action_client(goal_command)
            rospy.loginfo(result)
            
        else:
            pass
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)