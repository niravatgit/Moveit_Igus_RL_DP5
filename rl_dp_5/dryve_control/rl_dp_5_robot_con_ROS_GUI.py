from sensor_msgs.msg import JointState
import tkinter as tk
import numpy as np
import dryve_D1 as dryve
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

# Create labels for displaying positions for each axis
for axis in range(5):
    position_label = tk.Label(frame, text=f"Axis {axis + 1} Position: 0.00")
    position_label.grid(row=axis + 1, column=0, columnspan=3)
    position_labels.append(position_label)

class D1AxisController:
    def __init__(self):
        # Initialize your 5 D1 axes here
        Aaxis = dryve.D1("169.254.0.1", 502, 'Axis 1', -140, -140, 140)
        Baxis = dryve.D1("169.254.0.2", 502, 'Axis 2', -100, -100, 50)
        Caxis = dryve.D1("169.254.0.3", 502, 'Axis 3', -115, -115, 115)
        Daxis = dryve.D1("169.254.0.4", 502, 'Axis 4', -100, -100, 100)
        Eaxis = dryve.D1("169.254.0.5", 502, 'Axis 5', -180, -179, 179)

        self.axis_controller = [Aaxis, Baxis, Caxis, Daxis, Eaxis]

    def set_target_position(self, axis, desired_absolute_position):
        if 0 <= axis < len(self.axis_controller):
            self.axis_controller[axis].profile_pos_mode(desired_absolute_position, speed, accel)

    def home(self, axis):
        print(f"Started homing Axis {axis + 1}")
        self.axis_controller[axis].homing(homespeed, homeaccel)

    def home_all(self):
        for axis in self.axis_controller:
            print(f"Started homing {axis.Axis}")
            axis.homing(homespeed, homeaccel)

    def get_current_position(self, axis):
        return self.axis_controller[axis].getPosition()

    def setTargetVelocity(self, axis, velocity):
        if 0 <= axis < len(self.axis_controller):
            self.axis_controller[axis].targetVelocity(velocity)

class ROS_GUI_Interface:
    def __init__(self, robot):
        self.robot = robot
        self.joint_state = JointState()

        # Initialize ROS node
        rospy.init_node('ros_gui_node', anonymous=True)

        # ROS Publisher for simulating robot pose in MoveIt
        self.fake_controller_joint_states_pub = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=1)

        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_state.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
    
        rospy.Subscriber('/joint_states', JointState, self.callback_fn, queue_size=1)

    def callback_fn(self, data):
        # Update joint state based on received data
        self.joint_state = data
        self.joint_state_position = list(data.position)

        self.fake_controller_joint_states_pub.publish(self.joint_state)
        return self.joint_state_position

    def start_homing_all(self):
        print("homing all")
        self.robot.home_all()
        for i in range(5):
            self.joint_state.position[i] = self.robot.get_current_position(i)
        self.fake_controller_joint_states_pub.publish(self.joint_state)

    def start_homing(self, axis):
        print(f"homing joint{axis+1}")
        self.robot.home(axis)
        for i in range(5):
            self.joint_state.position[i] = self.robot.get_current_position(i)
        self.fake_controller_joint_states_pub.publish(self.joint_state)

    def jog(self, event, axis, direction):
        #print(self.joint_state.position
        self.robot.set_target_position(axis, self.joint_state.position[axis] + direction * 1)
        print('self.joint_state.position:', list(self.joint_state.position))
        for i in range(5):
            self.joint_state.position[i] = self.robot.get_current_position(i)
        self.fake_controller_joint_states_pub.publish(self.joint_state)

    def stop_jogging(self, event, axis):
        print(f"Stopped holding Axis {axis + 1}")
        self.robot.setTargetVelocity(axis, 0)
        for i in range(5):
            self.joint_state.position[i] = self.robot.get_current_position(i)
        self.fake_controller_joint_states_pub.publish(self.joint_state)

class ClickAndHoldApp:
    def __init__(self, root, ros_gui_interface, position_labels):
        self.root = root
        self.position_labels = position_labels
        self.rgi = ros_gui_interface

        for axis in range(5):
            homing_button = tk.Button(frame, text=f"Axis {axis + 1} Homing", command=lambda axis=axis: self.rgi.start_homing(axis))
            homing_button.grid(row=axis + 1, column=3, padx=10, pady=5)

            jog_plus_button = tk.Button(frame, text=f"Axis {axis + 1} Jog+")
            jog_plus_button.grid(row=axis + 1, column=4, padx=10, pady=5)
            jog_plus_button.bind("<ButtonPress-1>", lambda event, axis=axis: self.rgi.jog(event, axis, 1))
            jog_plus_button.bind("<ButtonRelease-1>", lambda event, axis=axis: self.rgi.stop_jogging(event, axis))

            jog_minus_button = tk.Button(frame, text=f"Axis {axis + 1} Jog-")
            jog_minus_button.grid(row=axis + 1, column=5, padx=10, pady=5)
            jog_minus_button.bind("<ButtonPress-1>", lambda event, axis=axis: self.rgi.jog(event, axis, -1))
            jog_minus_button.bind("<ButtonRelease-1>", lambda event, axis=axis: self.rgi.stop_jogging(event, axis))

        homing_all_button = tk.Button(frame, text="Homing All", command=self.rgi.start_homing_all)
        homing_all_button.grid(row=6, column=0, columnspan=6, pady=20)

        self.update_timer()
        
    def update_timer(self):
        for axis, label in zip(self.rgi.robot.axis_controller, self.position_labels):
            position = "{:.2f}".format(axis.getPosition())
            label.config(text=f"{axis.Axis} Position: {position}")
        self.root.after(5, self.update_timer)


if __name__ == "__main__":

    robot = D1AxisController()
    ros_gui_interface = ROS_GUI_Interface(robot)
    app = ClickAndHoldApp(root, ros_gui_interface, position_labels)
    root.mainloop()
    
    
    
"""
self.joint_state.position: [0.0, 0.0, 0.0, 0.0, 0.0]
Exception in Tkinter callback
Traceback (most recent call last):
  File "/usr/lib/python3.8/tkinter/__init__.py", line 1892, in __call__
    return self.func(*args)
  File "rl_dp_5_robot_con_ROS_GUI.py", line 125, in <lambda>
    jog_plus_button.bind("<ButtonPress-1>", lambda event, axis=axis: self.rgi.jog(event, axis, 1))
  File "rl_dp_5_robot_con_ROS_GUI.py", line 103, in jog
    self.joint_state.position[i] = self.robot.get_current_position(i)
TypeError: 'tuple' object does not support item assignment
Stopped holding Axis 1
Exception in Tkinter callback
Traceback (most recent call last):
  File "/usr/lib/python3.8/tkinter/__init__.py", line 1892, in __call__
    return self.func(*args)
  File "rl_dp_5_robot_con_ROS_GUI.py", line 126, in <lambda>
    jog_plus_button.bind("<ButtonRelease-1>", lambda event, axis=axis: self.rgi.stop_jogging(event, axis))
  File "rl_dp_5_robot_con_ROS_GUI.py", line 110, in stop_jogging
    self.joint_state.position[i] = self.robot.get_current_position(i)
TypeError: 'tuple' object does not support item assignment
inspire_igus@inspireigus:~/catkin_ws/src/Moveit_Igus_RL_DP5/r
"""
