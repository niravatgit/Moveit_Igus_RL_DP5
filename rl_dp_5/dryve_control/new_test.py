#!/usr/bin/env python
import rospy
from std_msgs.msg import String 

import sys

# ROS node
rospy.init_node('command_node')

pub_mode = rospy.Publisher('/robot/mode', String, queue_size=10)
pub_cmd = rospy.Publisher('/robot/cmd', String, queue_size=10)  

def print_usage():
    print("Greetings! Here is how to use this program:")
    print("  python robot_control.py command arguments")
    print("The available commands are:")
    print("  home - Requests homing of the robot") 
    print("  mode OPERATION_MODE - Respectfully asks to set operation mode")
    print("  cmd JOINT_POSITION - Politely commands joint position")

if __name__ == "__main__":

    if len(sys.argv) < 2:
        print_usage()
        print("Please provide a valid command.")
        sys.exit(1)

    command = sys.argv[1]

    if command == "home":
        print("Requesting the robot to home itself.")
        pub_cmd.publish("home")

    elif command == "mode":
        if len(sys.argv) < 3:
            print("My apologies, the operation mode was not specified.")
            print_usage()
            sys.exit(1)

        mode = sys.argv[2]
        print("Asking to set mode to", mode)
        pub_mode.publish(mode)

    elif command == "cmd":
        if len(sys.argv) < 3:
            print("Sorry, the joint position was not specified.")
            print_usage()
            sys.exit(1)
            
        position = float(sys.argv[2])
        print("Politely commanding joint to", position)
        pub_cmd.publish(str(position))

    else:
        print("I do not understand this command. Please use one of the valid commands.")
        print_usage()
        sys.exit(1)


------------------------------------------------------------------------------- 15-11-2023 ----------------------------------------------------------------------------------------------------
Subscriber code:

#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

rospy.init_node('joint_state_subscriber')

last_msg = None

def test_msg(msg):
	global last_msg
	last_msg = msg

def print_msg(msg):
  if(msg is not None):
    print('The Joint States of the robot is:', msg)

sub = rospy.Subscriber('/joint_states', JointState, test_msg, queue_size=1)

while not rospy.is_shutdown():
	print_msg(last_msg)





---------------------------------------18-11-2023-----------------------------------
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import socket
import time
import tkinter as tk
import struct
import dryve_D1 as dryve

speed = 5
accel = 100
homespeed = 10
homeaccel = 100

class Rl_DP_5:
    # ... (unchanged)

class JointStatesSubscriber:
    def __init__(self, robot):
        self.robot = robot
        rospy.init_node('joint_states_subscriber', anonymous=True)
        self.joint_states_pub = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=10)
        self.position_history = []

    def publish_current_positions(self):
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
        joint_state.position = [self.robot.get_current_position(i) for i in range(5)]

        # Publish joint state
        self.joint_states_pub.publish(joint_state)

    def joint_states_callback(self, data):
        # (unchanged)

    def check_repeated_values(self, current_values, threshold):
        # (unchanged)

if __name__ == "__main__":
    robot = Rl_DP_5()
    joint_states_subscriber = JointStatesSubscriber(robot)

    try:
        while not rospy.is_shutdown():
            joint_states_subscriber.publish_current_positions()
            rospy.sleep(1)
    except rospy.ROSInterruptException:
        pass

