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
    var = isinstance(goal_command, str)
    if var == True:
        goal = rldp5_robotGoal(goal_command)

    else:
        goal = goal_command + [0.0] * (5 - len(goal_command))

    client.send_goal(goal)
    rospy.loginfo("Goal has been sent to the action server.")

    # Waits for the server to finish performing the action.
    rospy.loginfo("Waiting for result...")
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A CounterWithDelayResult

if __name__ == '__main__':

    rospy.init_node('rldp5_Robot_ac_py')
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        if len(sys.argv) > 1:
            goal_command = sys.argv[1]
            if len(sys.argv) > 2:
                goal_command = [float(arg) for arg in sys.argv[1:]]
                
            result = rldp5_robot_action_client(goal_command)
            rospy.loginfo(result)
        
        else:
            rospy.loginfo("Provide the arguments properly!!!")

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
        
        
"""

[INFO] [1701165468.769550]: client action...
[INFO] [1701165468.777712]: Waiting for action server to come up...
The avaialble commands to send to the Action Server:
 ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'home_all', 'set_shutdn', 'set_swon', 'set_op_en']
[INFO] [1701165468.820650]: sending goal...
Traceback (most recent call last):
  File "ac.py", line 55, in <module>
    result = rldp5_robot_action_client(goal_command)
  File "ac.py", line 29, in rldp5_robot_action_client
    goal = rldp5_robotGoal(goal_command)
  File "/home/inspire_igus/catkin_ws/devel/lib/python3/dist-packages/rldp5_msgs/msg/_rldp5_robotGoal.py", line 38, in __init__
    super(rldp5_robotGoal, self).__init__(*args, **kwds)
  File "/opt/ros/noetic/lib/python3/dist-packages/genpy/message.py", line 354, in __init__
    raise TypeError('Invalid number of arguments, args should be %s' % str(self.__slots__) + ' args are' + str(args))
TypeError: Invalid number of arguments, args should be ['command', 'desired_position'] args are('home_all',)


Traceback (most recent call last):
  File "ac.py", line 53, in <module>
    goal_command = [float(arg) for arg in sys.argv[1:]]
  File "ac.py", line 53, in <listcomp>
    goal_command = [float(arg) for arg in sys.argv[1:]]
ValueError: could not convert string to float: 'home_all'


"""
