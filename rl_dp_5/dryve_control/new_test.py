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
