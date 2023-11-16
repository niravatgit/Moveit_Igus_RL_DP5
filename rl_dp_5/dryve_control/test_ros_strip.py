#!/usr/bin/env python3

# Created by Laugesen, Nichlas O.; Dam, Elias Thomassen.
# built from "Manual/Operating Manual dryve D1 EN V3.0.1.pdf"
import socket
import time
import tkinter as tk
import struct
import dryve_D1 as dryve
import rospy
from std_msgs.msg import Float64
import math
import rospy
from std_msgs.msg import String 
import sys

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


speed=5
accel=100
homespeed=10
homeaccel=100

# Publishers for velocity commands
pubs = []
for i in range(5):
  rospy.init_node(f'/status/joint_{i+1}', anonymous=True)
  topic = f'/axis_{i+1}/cmd_vel'
  pub = rospy.Publisher(topic, Float64, queue_size=10)
  rate = rospy.Rate(1)
  pubs.append(pub)

def Command_Input():
    print("Greetings! Here is how to use this program:")
    print("  python rl_dp_5_robot_con_ROS.py command arguments")
    print("The available commands are:")
    print("  => 'home' - Requests homing of the robot") 
    print("  => 'mode' OPERATION_MODE - Set the robot operation mode")
    print("  => 'status' JOINT_POSITION - Shows the joint position")


class ROS_Nodes:
    def __init__(self, axis_controller):
      self.axis_controller = axis_controller

    for axis in range(5):
            

            anticlockwise_button.bind("<ButtonPress-1>", lambda event, axis=axis: self.start_anticlockwise(event, axis))
            anticlockwise_button.bind("<ButtonRelease-1>", lambda event, axis=axis: self.stop_jogging(event, axis))

            clockwise_button = tk.Button(frame, text=f"Axis {axis + 1} Clockwise")
            clockwise_button.grid(row=axis + 1, column=4, padx=10, pady=5)
            clockwise_button.bind("<ButtonPress-1>", lambda event, axis=axis: self.start_clockwise(event, axis))
            clockwise_button.bind("<ButtonRelease-1>", lambda event, axis=axis: self.stop_jogging(event, axis))


            homing_button = tk.Button(frame, text=f"Axis {axis + 1} Homing", command=lambda axis=axis: self.start_individual_homing(axis))
            homing_button.grid(row=axis + 1, column=5, padx=10, pady=5)

            jog_plus_button = tk.Button(frame, text=f"Axis {axis + 1} Jog+")
            jog_plus_button.grid(row=axis + 1, column=6, padx=10, pady=5)
            jog_plus_button.bind("<ButtonPress-1>", lambda event, axis=axis: self.jog(event, axis, 1))
            jog_plus_button.bind("<ButtonRelease-1>", lambda event, axis=axis: self.stop_jogging(event, axis))

            jog_minus_button = tk.Button(frame, text=f"Axis {axis + 1} Jog-")
            jog_minus_button.grid(row=axis + 1, column=7, padx=10, pady=5)
            jog_minus_button.bind("<ButtonPress-1>", lambda event, axis=axis: self.jog(event, axis, -1))
            jog_minus_button.bind("<ButtonRelease-1>", lambda event, axis=axis: self.stop_jogging(event, axis))

            homing_all_button = tk.Button(frame, text="Homing All", command=self.start_homing)
            homing_all_button.grid(row=6, column=0, columnspan=6, pady=20)

            self.update_timer()

    def jog(self, event, axis, direction):
        cur_position = self.axis_controller.axes[axis].getPosition()
        print('Curent position =', self.axis_controller.axes[axis].getPosition())
        desired_position = cur_position + direction*1
        self.axis_controller.axes[axis].profile_pos_mode(desired_position, 5,50)
        print(f"Started jogging axis ", axis, "From current postion=", cur_position, " To desired position = ", desired_position)

    def start_clockwise(self, event, axis):
        self.axis_controller.setTargetVelocity(axis, 500)
        print(f"Started holding Axis {axis + 1} Clockwise")

    def start_anticlockwise(self, event, axis):
        self.axis_controller.setTargetVelocity(axis, -500)
        print(f"Started holding Axis {axis + 1} Anti-Clockwise")

    def stop_jogging(self, event, axis):
        self.axis_controller.setTargetVelocity(axis, 0)
        print(f"Stopped holding Axis {axis + 1}")

    def start_individual_homing(self, axis):
        print(f"Started homing Axis {axis + 1}")
        self.axis_controller.axes[axis].homing(homespeed, homeaccel)


    def start_homing(self):
        for axis in self.axis_controller.axes:
            print(f"Started homing {axis.Axis}")
            axis.homing(homespeed, homeaccel)

    def update_timer(self):
        for axis, label in zip(self.axis_controller.axes, self.position_labels):
            position = "{:.2f}".format(axis.getPosition())
            label.config(text=f"{axis.Axis} Position: {position}")
        self.root.after(5, self.update_timer)

class D1AxisController:
    def __init__(self):
        # Initialize your 5 D1 axes here
        Aaxis = dryve.D1("169.254.0.1", 502, 'Axis 1')
        Baxis = dryve.D1("169.254.0.2", 502, 'Axis 2')
        Caxis = dryve.D1("169.254.0.3", 502, 'Axis 3')
        Daxis = dryve.D1("169.254.0.4", 502, 'Axis 4')
        Eaxis = dryve.D1("169.254.0.5", 502, 'Axis 5')

        self.axes = [Aaxis, Baxis, Caxis, Daxis, Eaxis]

    def setTargetVelocity(self, axis, velocity):
        if 0 <= axis < len(self.axes):
            self.axes[axis].targetVelocity(velocity)

    def setTargetPosition(self, axis, desired_position):
        if 0 <= axis < len(self.axes):
            self.axes[axis].targetPosition(desired_position)




# Callback for anti-clockwise button  
def anticlockwise_callback(axis):
  velocity = -500 # negative for anticlockwise
  pubs[axis].publish(velocity)
  
# Callback for stop jogging  
def stop_callback(axis):
  velocity = 0
  pubs[axis].publish(velocity)
  
# # Subscribers for button presses  
# for i in range(5):
#   rospy.Subscriber(f'cmd/axis_{i+1}/anticlockwise', std_msgs.msg.Empty, 
#                   lambda msg, axis=i: anticlockwise_callback(axis))
                  
#   rospy.Subscriber(f'/axis_{i+1}/stop', std_msgs.msg.Empty, 
#                   lambda msg, axis=i: stop_callback(axis))

# rospy.init_node('gui_node')
# rospy.spin()

if __name__ == "__main__":
    app = ClickAndHoldApp(root, D1AxisController(), position_labels)
    root.mainloop()
    
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

--------------------------------------------------------------16-11-2023----------------------------------------------------------------------------------
--------------------------------------------------------------Command Line args----------------------------------------------------------------------------------
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import moveit_commander
import sys

# Import the D1 class from the provided program
class D1:
    # ... (Your existing D1 class code)

def publish_joint_positions(joint_positions):
    # Your existing publish_joint_positions function
    pass

def check_robot_status():
    # Your existing check_robot_status function
    pass

def joint_state_callback(data):
    # Your existing joint_state_callback function
    pass

def handle_commands(d1, command):
    if command == 'home':
        d1.homing(homespeedsw=your_speed, homeacc=your_acceleration)
    elif command == 'mode1':
        # Set mode 1 specific logic
        print("Setting robot to mode 1. Add your logic here.")
    elif command == 'mode2':
        # Set mode 2 specific logic
        print("Setting robot to mode 2. Add your logic here.")
    elif command == 'status':
        check_robot_status()
    else:
        print("Invalid command. Available commands: 'home', 'mode1', 'mode2', 'status'")
        sys.exit(1)

def listener(d1):
    rospy.init_node('robot_controller', anonymous=True)

    # Example joint positions, replace with your actual data
    initial_joint_positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]

    # Check for command line arguments
    if len(sys.argv) == 2:
        command = sys.argv[1]
        handle_commands(d1, command)

    # Publish joint positions
    publish_joint_positions(initial_joint_positions)

    # Initialize MoveIt
    moveit_commander.roscpp_initialize(sys.argv)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Shut down MoveIt
        moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    # Replace with actual IP, port, and axis label
    d1 = D1(IP_Adress="your_ip_address", Port=your_port, Axis="your_axis_label")
    listener(d1)

