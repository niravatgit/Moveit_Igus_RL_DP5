# Created by Laugesen, Nichlas O.; Dam, Elias Thomassen.
# built from "Manual/Operating Manual dryve D1 EN V3.0.1.pdf"
import socket
import time
import tkinter as tk
read = 0
write = 1
profileAcceleration = 300
profileDeceleration = 300
HOST = "169.254.0.5"
PORT = 502

# Commands/arrays -------------------------------------------

status = [0, 0, 0, 0, 0, 13, 0, 43, 13, read, 0, 0, 96, 65, 0, 0, 0, 0, 2]
statusArray = bytearray(status)

shutdown = [0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 6, 0]
shutdownArray = bytearray(shutdown)

switchOn = [0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 7, 0]
switchOnArray = bytearray(switchOn)

enableOperation = [0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 0x60, 0x40, 0, 0, 0, 0, 2, 15, 0]
enableOperationArray = bytearray(enableOperation)

class D1:
    def __init__(self, IP_Adress, Port, Axis):
        self.IP_Adress=IP_Adress # Define the IP Adress
        self.Socket=Port # Define the port (default:502) to create socket
        self.Axis=Axis
        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except socket.error:
            print ('failed to create socket')         
        self.s.connect((IP_Adress, Port))


    def extractBytes(self,integer):
        return divmod(integer, 0x100)[::-1]

    def setShdn(self):
        self.sendCommand(shutdownArray)
        while (self.sendCommand(statusArray) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 33, 6]
            and self.sendCommand(statusArray) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 33, 22]
            and self.sendCommand(statusArray) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 33, 2]):
            print("wait for shutdown")

            # 1 Sekunde Verzoegerung
            # 1 second delay
            time.sleep(1)


    # Function for switching on
    def setSwon(self):
        self.sendCommand(switchOnArray)
        while (self.sendCommand(statusArray) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 35, 6]
            and self.sendCommand(statusArray) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 35, 22]
            and self.sendCommand(statusArray) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 35, 2]):
            print("wait for switch on")

            # 1 Sekunde Verzoegerung
            # 1 second delay
            time.sleep(1)


    # Function for enabling operation
    def setOpEn(self):
        self.sendCommand(enableOperationArray)
        while (self.sendCommand(statusArray) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 6]
            and self.sendCommand(statusArray) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22]
            and self.sendCommand(statusArray) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 2]):
            print("wait for op en")

            # 1 Sekunde Verzoegerung
            # 1 second delay
            time.sleep(1)


    def setMode(self,mode):
        # Set operation modes in object 6060h Modes of Operation
        self.sendCommand(bytearray([0, 0, 0, 0, 0, 14, 0, 43, 13, 1, 0, 0, 96, 96, 0, 0, 0, 0, 1, mode]))
        while (self.sendCommand(bytearray([0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 97, 0, 0, 0, 0, 1])) != [0, 0, 0, 0, 0, 14,
                                                                                                        0, 43, 13, 0, 0,
                                                                                                        0, 96, 97, 0, 0,
                                                                                                        0, 0, 1, mode]):
            # 1 second delay
            time.sleep(1)

    def startProcedure(self):
        self.reset = [0, 0, 0, 0, 0, 15, 0, 43, 13, write, 0, 0, 96, 64, 0, 0, 0, 0, 2, 0, 1]
        self.resetArray = bytearray(self.reset)
        self.sendCommand(self.resetArray)

        self.sendCommand(statusArray)

        self.setShdn()
        self.setSwon()
        self.setOpEn()
        self.setMode(1)
        # set velocity and acceleration of profile
        self.sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 0x60, 0x81, 0, 0, 0, 0, 2, 0x2c, 0x1]))
        # Profile acceleration set below
        self.sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 0x60, 0x83, 0, 0, 0, 0, 2, self.extractBytes(profileAcceleration)[0], self.extractBytes(profileAcceleration)[1]]))
        # Profile deacceleration set below
        self.sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 0x60, 0x84, 0, 0, 0, 0, 2, self.extractBytes(profileDeceleration)[0], self.extractBytes(profileDeceleration)[1]]))

    def sendCommand(self,data):
        # Create socket and send request
        self.s.send(data)
        self.res = self.s.recv(24)
        # #Print response telegram
        # print(list(res))
        return list(self.res)

    def getPosition(self):
        getPositionFromDryve = bytearray([0, 0, 0, 0, 0, 13, 0, 43, 13, read, 0, 0, 0x60, 0x64, 0, 0, 0, 0, 4])
        positionRaw = self.sendCommand(getPositionFromDryve)

        raw_position = 0
        for i in range(4):
            raw_position = raw_position + positionRaw[i + 19] * 256 ** i

        sign_bit = (raw_position & 0x80000000)

        if sign_bit:
            position = -((raw_position ^ 0xFFFFFFFF) + 1) / 100
        else:
            position = raw_position / 100

        return "{:.2f}".format(position)
    

    def targetPosition(self,angle, rw=1):
        self.setMode(1)

        # Convert angle to target position value
        self.target = int(angle * 100)  # Multiply angle by 100 to obtain the target position

        # Check if target datavalue is within range
        if self.target > 0xffff:
            print("Invalid target specified")
        else:
            if self.target > 255:
                self.target2Byt = self.extractBytes(self.target)
                self.targetPos = [0, 0, 0, 0, 0, 15, 0, 43, 13, rw, 0, 0, 0x60, 0x7a, 0, 0, 0, 0, 2, self.target2Byt[0],
                            self.target2Byt[1]]
            elif self.target <= 255:
                self.targetPos = [0, 0, 0, 0, 0, 14, 0, 43, 13, rw, 0, 0, 0x60, 0x7a, 0, 0, 0, 0, 1, self.target]
            self.targetPosArray = bytearray(self.targetPos)
            self.sendCommand(self.targetPosArray)

            # Execute command
            self.sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, rw, 0, 0, 0x60, 0x40, 0, 0, 0, 0, 2, 0x1f, 0x0]))

            # Check Statusword for target reached
            while (self.sendCommand(statusArray) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 0x60, 0x41, 0, 0, 0, 0, 2, 39, 22]):
                time.sleep(0.001)

            self.sendCommand(enableOperationArray)

    def targetVelocity(self, target):
        self.setMode(3)

        if target > 0xffff:
            print("Invalid target velocity specified")
        else:
            targetVel2Byt = (target).to_bytes(4, byteorder='little', signed=True)

            # set velocity of the profile
            self.sendCommand(bytearray([0, 0, 0, 0, 0, 17, 0, 43, 13, write, 0, 0, 0x60, 0xFF, 0, 0, 0, 0, 4]) + targetVel2Byt)

    def profileVelocity(self, target):
        def extractBytes(integer):
            return divmod(integer, 0x100)[::-1]
        
        if target > 0xffff or target == 0:
            print("Invalid target velocity specified")
        else:
            if target > 255:  # If the target is over 2 bytes large, split the data correctly into two separate bytes.
                targetVel2Byt = extractBytes(target)
                # set velocity and acceleration of the profile
                self.sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 0x60, 0x81, 0, 0, 0, 0, 2, targetVel2Byt[0], targetVel2Byt[1]]))
            elif target <= 255:
                # set velocity and acceleration of the profile
                self.sendCommand(bytearray(
                    [0, 0, 0, 0, 0, 14, 0, 43, 13, 1, 0, 0, 0x60, 0x81, 0, 0, 0, 0, 1, target]))

    def homing(self):
        self.setMode(6)

        setHomingMethodLSN = [0, 0, 0, 0, 0, 14, 0, 43, 13, write, 0, 0, 96, 152, 0, 0, 0, 0, 1, 17]
        setHomingMethodLSNArray = bytearray(setHomingMethodLSN)
        self.sendCommand(setHomingMethodLSNArray)

        # Set homing speeds 6099h
        self.sendCommand(bytearray([0, 0, 0, 0, 0, 14, 0, 43, 13, 1, 0, 0, 0x60, 0x99, 0, 0, 0, 0, 1, 200]))
        self.sendCommand(bytearray([0, 0, 0, 0, 0, 14, 0, 43, 13, 1, 0, 0, 0x60, 0x99, 1, 0, 0, 0, 1, 200]))
        self.sendCommand(bytearray([0, 0, 0, 0, 0, 14, 0, 43, 13, 1, 0, 0, 0x60, 0x99, 2, 0, 0, 0, 1, 200]))

        # Set acceleration 609Ah
        self.sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 0x60, 0x9a, 0, 0, 0, 0, 2, 0xe8, 0x3]))

        time.sleep(0.1)

        print("Begin Homing")
        # Start Homing 6040h
        self.sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 0x60, 0x40, 0, 0, 0, 0, 2, 0x1f, 0]))

        while (self.sendCommand(statusArray) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 0x60, 0x41, 0, 0, 0, 0, 2, 39, 22]):
            # 1 second delay
            time.sleep(0.1)

        print("Homing complete")

        self.sendCommand(enableOperationArray)



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

        homing_all_button = tk.Button(frame, text="Homing All", command=self.start_homing)
        homing_all_button.grid(row=6, column=0, columnspan=6, pady=20)

        self.update_timer()

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
        self.axis_controller.axes[axis].homing()


    def start_homing(self):
        for axis in self.axis_controller.axes:
            print(f"Started homing {axis.Axis}")
            axis.homing()

    def update_timer(self):
        for axis, label in zip(self.axis_controller.axes, self.position_labels):
            position = axis.getPosition()
            label.config(text=f"{axis.Axis} Position: {position}")
        self.root.after(50, self.update_timer)

class D1AxisController:
    def __init__(self):
        # Initialize your 5 D1 axes here
        Aaxis = D1("169.254.0.1", 502, 'Axis 1')
        Baxis = D1("169.254.0.2", 502, 'Axis 2')
        Caxis = D1("169.254.0.3", 502, 'Axis 3')
        Daxis = D1("169.254.0.4", 502, 'Axis 4')
        Eaxis = D1("169.254.0.5", 502, 'Axis 5')

        Aaxis.startProcedure()
        Baxis.startProcedure()
        Caxis.startProcedure()
        Daxis.startProcedure()
        Eaxis.startProcedure()

        self.axes = [Aaxis, Baxis, Caxis, Daxis, Eaxis]

    def setTargetVelocity(self, axis, velocity):
        if 0 <= axis < len(self.axes):
            self.axes[axis].targetVelocity(velocity)

if __name__ == "__main__":
    app = ClickAndHoldApp(root, D1AxisController(), position_labels)
    root.mainloop()
