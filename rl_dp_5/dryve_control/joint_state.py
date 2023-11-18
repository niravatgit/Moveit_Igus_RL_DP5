# Created by Laugesen, Nichlas O.; Dam, Elias Thomassen.
# built from "Manual/Operating Manual dryve D1 EN V3.0.1.pdf"
import socket
import rospy
import time
import tkinter as tk
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

#    def setTargetVelocity(self, axis, velocity):
#        if 0 <= axis < len(self):
#            self.axis_controller[axis].targetVelocity(velocity)

    def setTargetPosition(self, axis, desired_absolute_position):
        if 0 <= axis < len(self):
            self.axis_controller[axis].profile_pos_mode(desired_absolute_position, 5,50)

#    def jog(self, event, axis, direction):
#        cur_position = self.axis_controller[axis].getPosition()
#        print('Curent position =', self.axis_controller[axis].getPosition())
#        desired_position = cur_position + direction*1
#        self.axis_controller[axis].profile_pos_mode(desired_position, 5,50)
#        print(f"Started jogging axis ", axis, "From current postion=", cur_position, " To desired position = ", desired_position)

#    def start_clockwise(self, event, axis):
#        self.axis_controller.setTargetVelocity(axis, 500)
#        print(f"Started holding Axis {axis + 1} Clockwise")

#    def start_anticlockwise(self, event, axis):
#        self.axis_controller.setTargetVelocity(axis, -500)
#        print(f"Started holding Axis {axis + 1} Anti-Clockwise")

#    def stop_jogging(self, event, axis):
#        self.axis_controller.setTargetVelocity(axis, 0)
#        print(f"Stopped holding Axis {axis + 1}")

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
    
    def joint_state_callback(data):
        global printJointStates
        printJointStates = data
        #rospy.Header.frame_id = "Joint_States"
        time = rospy.get_rostime()
        if printJointStates:
            print("Position of joints at timestamp " + str(time) + " :", list(data.position))
            printJointStates = list(data.position)
        return printJointStates

    def listener():
        rospy.init_node('listener', anonymous=True)
        print("Printing Joint states of all joints")
        joint_state_positions = rospy.Subscriber("/joint_states", JointState, joint_state_callback, queue_size=1)

        pub = rospy.Publisher()


if __name__ == "__main__":
    robot = Rl_DP_5()
    robot.homeAll()
    while True:
        for i in range(0,5):
            print('Current Position of joint ', i, ': ', robot.get_current_position(i))
        

