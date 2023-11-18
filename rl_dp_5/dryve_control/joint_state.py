# Created by Laugesen, Nichlas O.; Dam, Elias Thomassen.
# built from "Manual/Operating Manual dryve D1 EN V3.0.1.pdf"

#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import socket
import time
import struct
import dryve_D1 as dryve
speed=5
accel=100
homespeed=10
homeaccel=100


#following are the fucntions that we want to expose through ROS
#workspace will be /rl_dp_5
#1. publisher: /status for a joint such as {mode of operation, current position, is_initialized }, this will require calling multiple functiosn from dryve_D1.py
#2. service: /setMode : integer as an input passed on to function set_mode from dryve_D1.py -> check the arguments
#3. service: /home : this will call homing from dryve_D1.py -> check the arguments
#4. subsriber: /cmd/set_joint_position : this will set desired joint position by calling profile_pos_mode -> check arguments
#5. 
#
#
#
#
#
#
#start ROS Node code here


class Rl_DP_5:
    def __init__(self):
        # Initialize your 5 D1 axes here
        Aaxis = dryve.D1("169.254.0.1", 502, 'Axis 1', -140, -140, 140)
        Baxis = dryve.D1("169.254.0.2", 502, 'Axis 2', -100, -100, 50)
        Caxis = dryve.D1("169.254.0.3", 502, 'Axis 3', -115, -115, 115 )
        Daxis = dryve.D1("169.254.0.4", 502, 'Axis 4', -100, -100, 100)
        Eaxis = dryve.D1("169.254.0.5", 502, 'Axis 5', -180, -179, 179)

        self.axis_controller = [Aaxis, Baxis, Caxis, Daxis, Eaxis]


    def setTargetPosition(self, axis, desired_absolute_position):
        if 0 <= axis < len(self):
            self.axis_controller[axis].profile_pos_mode(desired_absolute_position, 5,50)

    def home(self, axis):
        print(f"Started homing Axis {axis + 1}")
        self.axis_controller[axis].homing(homespeed, homeaccel)

    def homeAll(self):
        for axis in self.axis_controller:
            print(f"Started homing {axis.Axis}")
            axis.homing(homespeed, homeaccel)

    def get_current_position(self, axis):
        return self.axis_controller[axis].getPosition()
    printJointStates = False

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

