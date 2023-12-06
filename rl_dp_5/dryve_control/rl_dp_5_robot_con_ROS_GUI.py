# Created by Laugesen, Nichlas O.; Dam, Elias Thomassen.
# built from "Manual/Operating Manual dryve D1 EN V3.0.1.pdf"

import tkinter as tk
import dryve_D1 as dryve
from sensor_msgs.msg import JointState
import numpy as np
import threading
import rospy
speed=5
accel=100
homespeed=5
homeaccel=100

root = tk.Tk()
root.title("Click and Hold")

frame = tk.Frame(root)
frame.pack(padx=20, pady=20)

heading_label = tk.Label(frame, text="Axis Control Panel", font=("Helvetica", 16))
heading_label.grid(row=0, column=0, columnspan=6, pady=(0, 20))

position_labels = []
desired_position = []

# Create labels for displaying positions for each axis
for axis in range(5):
    position_label = tk.Label(frame, text=f"Axis {axis + 1} Position: 0.00")
    position_label.grid(row=axis + 1, column=0, columnspan=3)
    position_labels.append(position_label)


class ClickAndHoldApp:
    def __init__(self, root, position_labels, axis_controller):
    # def __init__(self, root, axis_controller, position_labels):
        self.root = root
        self.position_labels = position_labels
        self.axis_controller = axis_controller
        self.homing_in_progress = False
        
        rospy.loginfo("Axis COntroller:", self.axis_controller)

        rospy.init_node('ros_gui_node', anonymous=True)

        # Publisher to simulate the robot pose in MoveIt based on the current position of the real robot
        self.fake_controller_joint_states_pub = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=1)

        self.joint_state = JointState()
        rospy.Subscriber('/joint_states', JointState, self.callback_fn, queue_size=1)
        
        print("Created ROS GUI interface")

        for axis in range(5): 
            homing_button = tk.Button(frame, text=f"Axis {axis + 1} Homing", command=lambda axis=axis: self.start_individual_homing(axis))
            homing_button.grid(row=axis + 1, column=3, padx=10, pady=5)

            jog_plus_button = tk.Button(frame, text=f"Axis {axis + 1} Jog+")
            jog_plus_button.grid(row=axis + 1, column=4, padx=10, pady=5)
            jog_plus_button.bind("<ButtonPress-1>", lambda event, axis=axis: self.jog(event, axis, 1))
            jog_plus_button.bind("<ButtonRelease-1>", lambda event, axis=axis: self.stop_jogging(event, axis))

            jog_minus_button = tk.Button(frame, text=f"Axis {axis + 1} Jog-")
            jog_minus_button.grid(row=axis + 1, column=5, padx=10, pady=5)
            jog_minus_button.bind("<ButtonPress-1>", lambda event, axis=axis: self.jog(event, axis, -1))
            jog_minus_button.bind("<ButtonRelease-1>", lambda event, axis=axis: self.stop_jogging(event, axis))

        homing_all_button = tk.Button(frame, text="Homing All", command=self.start_homing)
        homing_all_button.grid(row=6, column=0, columnspan=6, pady=20)

        self.update_timer()

    def callback_fn(self, data):
        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_state.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
        self.joint_state.position = list(data.position)
        return self.fake_controller_joint_states_pub.publish(self.joint_state)

    def jog(self, event, axis, direction):
        cur_position = self.axis_controller.axes[axis].getPosition()
        print('Curent position =', self.axis_controller.axes[axis].getPosition())
        desired_position = cur_position + direction*1
        self.axis_controller.axes[axis].profile_pos_mode(desired_position, 5,50)
        print(f"Started jogging axis ", axis, "From current postion=", cur_position, " To desired position = ", desired_position)
        #self.sub_fn()

    def stop_jogging(self, event, axis):
        self.axis_controller.setTargetVelocity(axis, 0)
        print(f"Stopped holding Axis {axis + 1}")
        #self.sub_fn()

    def start_individual_homing(self, axis):
        print(f"Started homing Axis {axis + 1}")
        self.axis_controller.home(axis)
        #self.sub_fn()

    def start_homing(self):
        if not self.homing_in_progress:
            self.homing_in_progress = True
            self.axis_controller.home_all()
        self.homing_in_progress = False
            #self.sub_fn()

    def update_timer(self):
        for axis, label in zip(self.axis_controller.axes, self.position_labels):
            position = "{:.2f}".format(axis.getPosition())
            label.config(text=f"{axis.Axis} Position: {position}")
        self.root.after(5, self.update_timer)

class D1AxisController:
    def __init__(self):
        # Initialize your 5 D1 axes here
        Aaxis = dryve.D1("169.254.0.1", 502, 'Axis 1', -140, -140, 140)
        Baxis = dryve.D1("169.254.0.2", 502, 'Axis 2', -100, -100, 50)
        Caxis = dryve.D1("169.254.0.3", 502, 'Axis 3', -115, -115, 115 )
        Daxis = dryve.D1("169.254.0.4", 502, 'Axis 4', -100, -100, 100)
        Eaxis = dryve.D1("169.254.0.5", 502, 'Axis 5', -180, -179, 179)

        self.axes = [Aaxis, Baxis, Caxis, Daxis, Eaxis]

        print("Created dryve interfaces")

    def setTargetVelocity(self, axis, velocity):
        if 0 <= axis < len(self.axes):
            self.axes[axis].targetVelocity(velocity)

    def setTargetPosition(self, axis, desired_position):
        if 0 <= axis < len(self.axes):
            self.axes[axis].profile_pos_mode(desired_position, speed, accel)

    def get_current_position(self, axis):
        return self.axes[axis].getPosition()
    
    def home(self, axis):
        print(f"Started homing Axis {axis + 1}")
        self.axes[axis].homing(homespeed, homeaccel)
        
    def home_all(self):
        for axis in self.axes:
            print(f"Started homing at robot level {axis.Axis}")
            axis.homing(homespeed, homeaccel)

if __name__ == "__main__":
    print("Created GUI")
    app = ClickAndHoldApp(root, position_labels, D1AxisController())
    # app = ClickAndHoldApp(root, D1AxisController(), position_labels)
    root.mainloop()

