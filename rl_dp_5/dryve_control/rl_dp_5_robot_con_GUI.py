# Created by Laugesen, Nichlas O.; Dam, Elias Thomassen.
# built from "Manual/Operating Manual dryve D1 EN V3.0.1.pdf"
import socket
import time
import tkinter as tk
import struct
import dryve_D1 as dryve
speed=5
accel=100
homespeed=10
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

class ClickAndHoldApp:
    def __init__(self, root, axis_controller, position_labels):
        self.root = root
        self.position_labels = position_labels
        self.axis_controller = axis_controller

        for axis in range(5):
            anticlockwise_button = tk.Button(frame, text=f"Axis {axis + 1} Anti-Clockwise")
            anticlockwise_button.grid(row=axis + 1, column=3, padx=10, pady=5)
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
        Aaxis = dryve.D1("169.254.0.1", 502, 'Axis 1', -140, -140, 140)
        Baxis = dryve.D1("169.254.0.2", 502, 'Axis 2', -100, -100, 50)
        Caxis = dryve.D1("169.254.0.3", 502, 'Axis 3', -115, -115, 115 )
        Daxis = dryve.D1("169.254.0.4", 502, 'Axis 4', -100, -100, 100)
        Eaxis = dryve.D1("169.254.0.5", 502, 'Axis 5', -180, -179, 179)

        self.axes = [Aaxis, Baxis, Caxis, Daxis, Eaxis]

    def setTargetVelocity(self, axis, velocity):
        if 0 <= axis < len(self.axes):
            self.axes[axis].targetVelocity(velocity)

    def setTargetPosition(self, axis, desired_position):
        if 0 <= axis < len(self.axes):
            self.axes[axis].targetPosition(desired_position)

if __name__ == "__main__":
    app = ClickAndHoldApp(root, D1AxisController(), position_labels)
    root.mainloop()
