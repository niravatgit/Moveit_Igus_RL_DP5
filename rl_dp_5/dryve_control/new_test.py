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
    def __init__(self):
        # Initialize your 5 D1 axes here
        Aaxis = dryve.D1("169.254.0.1", 502, 'Axis 1', -140, -140, 140)
        Baxis = dryve.D1("169.254.0.2", 502, 'Axis 2', -100, -100, 50)
        Caxis = dryve.D1("169.254.0.3", 502, 'Axis 3', -115, -115, 115)
        Daxis = dryve.D1("169.254.0.4", 502, 'Axis 4', -100, -100, 100)
        Eaxis = dryve.D1("169.254.0.5", 502, 'Axis 5', -180, -179, 179)

        self.axis_controller = [Aaxis, Baxis, Caxis, Daxis, Eaxis]

    def setTargetPosition(self, axis, desired_absolute_position):
        if 0 <= axis < len(self):
            self.axis_controller[axis].profile_pos_mode(desired_absolute_position, 5, 50)

    def home(self, axis):
        print(f"Started homing Axis {axis + 1}")
        self.axis_controller[axis].homing(homespeed, homeaccel)

    def homeAll(self):
        for axis in self.axis_controller:
            print(f"Started homing {axis.Axis}")
            axis.homing(homespeed, homeaccel)

    def get_current_position(self, axis):
        return self.axis_controller[axis].getPosition()

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
        joint_state = JointState()
        joint_state.header = data.header
        joint_state.name = data.name
        joint_state.position = data.position

        # Publish joint state
        self.joint_states_pub.publish(joint_state)

        # Check for repeated values
        if self.check_repeated_values(joint_state.position, 20):
            rospy.loginfo("Trajectory planned after 20 repeated positions.")
            rospy.signal_shutdown("Trajectory planned.")

    def check_repeated_values(self, current_values, threshold):
        self.position_history.append(current_values)
        if len(self.position_history) >= threshold:
            recent_positions = self.position_history[-threshold:]
            return all(positions == current_values for positions in recent_positions)
        return False
if __name__ == "__main__":
    robot = Rl_DP_5()
    joint_states_subscriber = JointStatesSubscriber(robot)

    try:
        while not rospy.is_shutdown():
            joint_states_subscriber.publish_current_positions()
            rospy.sleep(1)
    except rospy.ROSInterruptException:
        pass
------------------------------------------------------------------------------------------------------------------------------------------------
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import dryve_D1 as dryve

# Constants for motion control
speed = 5
accel = 100
homespeed = 10
homeaccel = 100

class Rl_DP_5:
    def __init__(self):
        # Initialize your 5 D1 axes here
        self.axis_controller = [
            dryve.D1("169.254.0.1", 502, 'Axis 1', -140, -140, 140),
            dryve.D1("169.254.0.2", 502, 'Axis 2', -100, -100, 50),
            dryve.D1("169.254.0.3", 502, 'Axis 3', -115, -115, 115),
            dryve.D1("169.254.0.4", 502, 'Axis 4', -100, -100, 100),
            dryve.D1("169.254.0.5", 502, 'Axis 5', -180, -179, 179)
        ]

    def set_target_position(self, axis, desired_absolute_position):
        if 0 <= axis < len(self.axis_controller):
            self.axis_controller[axis].profile_pos_mode(desired_absolute_position, 5, 50)

    def home(self, axis):
        print(f"Started homing Axis {axis + 1}")
        self.axis_controller[axis].homing(homespeed, homeaccel)

    def home_all(self):
        for axis in self.axis_controller:
            print(f"Started homing {axis.Axis}")
            axis.homing(homespeed, homeaccel)

    def get_current_position(self, axis):
        return self.axis_controller[axis].getPosition()

class JointStatesSubscriber:
    def __init__(self, robot):
        self.robot = robot
        rospy.init_node('joint_states_subscriber', anonymous=True)
        self.joint_states_pub = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=10)
        self.position_history = []

    def publish_current_positions(self):
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = [f"joint_{i+1}" for i in range(len(self.robot.axis_controller))]
        joint_state.position = [self.robot.get_current_position(i) for i in range(len(self.robot.axis_controller))]

        # Publish joint state
        self.joint_states_pub.publish(joint_state)

    def joint_states_callback(self, data):
        joint_state = JointState()
        joint_state.header = data.header
        joint_state.name = data.name
        joint_state.position = data.position

        # Publish joint state
        self.joint_states_pub.publish(joint_state)

        # Check for repeated values
        if self.check_repeated_values(joint_state.position, 20):
            rospy.loginfo("Trajectory planned after 20 repeated positions.")
            rospy.signal_shutdown("Trajectory planned.")

    def check_repeated_values(self, current_values, threshold):
        self.position_history.append(current_values)
        if len(self.position_history) >= threshold:
            recent_positions = self.position_history[-threshold:]
            return all(positions == current_values for positions in recent_positions)
        return False

if __name__ == "__main__":
    robot = Rl_DP_5()
    joint_states_subscriber = JointStatesSubscriber(robot)

    try:
        while not rospy.is_shutdown():
            joint_states_subscriber.publish_current_positions()
            rospy.sleep(1)
    except rospy.ROSInterruptException:
        pass

