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
                goal_command = a

            else:
                print("as a", a)
                n=len(sys.argv[1])
                print("n: ", n) 
                a=sys.argv[1][1:n-1]
                print(a)
                a=a.split(',') 
                goal_command = a

            print(goal_command)
            result = rldp5_robot_action_client(goal_command)
            rospy.loginfo(result)

        else:
            rospy.loginfo("Provide the arguments properly!!!")
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
        
        
"""
inspire_igus@inspireigus:~/catkin_ws/src/Moveit_Igus_RL_DP5/rl_dp_5/dryve_control$ python3 ac.py [1,0,1,0,0]
as a [1,0,1,0,0]
n:  11
1,0,1,0,0
['1', '0', '1', '0', '0']
[INFO] [1701255558.923054]: client action...
[INFO] [1701255558.935096]: Waiting for action server to come up...
The avaialble commands to send to the Action Server:
 ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'home_all', 'set_shutdn', 'set_swon', 'set_op_en']
[INFO] [1701255559.040127]: sending goal...
['1', '0', '1', '0', '0'] <class 'list'>
Goal:  command: 
  - '1'
  - '0'
  - '1'
  - '0'
  - '0' <class 'rldp5_msgs.msg._rldp5_robotGoal.rldp5_robotGoal'>
Traceback (most recent call last):
  File "ac.py", line 61, in <module>
    result = rldp5_robot_action_client(goal_command)
  File "ac.py", line 29, in rldp5_robot_action_client
    client.send_goal(goal_command)
  File "/opt/ros/noetic/lib/python3/dist-packages/actionlib/simple_action_client.py", line 92, in send_goal
    self.gh = self.action_client.send_goal(goal, self._handle_transition, self._handle_feedback)
  File "/opt/ros/noetic/lib/python3/dist-packages/actionlib/action_client.py", line 561, in send_goal
    return self.manager.init_goal(goal, transition_cb, feedback_cb)
  File "/opt/ros/noetic/lib/python3/dist-packages/actionlib/action_client.py", line 466, in init_goal
    self.send_goal_fn(action_goal)
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py", line 882, in publish
    self.impl.publish(data)
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py", line 1066, in publish
    serialize_message(b, self.seq, message)
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/msg.py", line 152, in serialize_message
    msg.serialize(b)
  File "/home/inspire_igus/catkin_ws/devel/lib/python3/dist-packages/rldp5_msgs/msg/_rldp5_robotActionGoal.py", line 119, in serialize
    _x = self.goal.command
AttributeError: 'list' object has no attribute 'command'
"""
