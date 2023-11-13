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
