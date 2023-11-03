# Created by Laugesen, Nichlas O.; Dam, Elias Thomassen.
# built from "Manual/Operating Manual dryve D1 EN V3.0.1.pdf"
import socket
import time
import tkinter as tk
import struct
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

enableOperation_relative = [0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 79, 0]
enableOperationArray_relative = bytearray(enableOperation_relative)

feedrate_array = bytearray([0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 146, 1, 0, 0, 0, 4]) # Send Telegram(TX) Read Obejct 6092h subindex 1 for the feed rate
SI_unit_array = bytearray([0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 168, 0, 0, 0, 0, 4]) # Send Telegram(TX) Read Object 60A8h for SI Unit Position
sendTargetPosition =  = bytearray([ 0, 0, 0, 0, 0, 17, 0, 43, 13, 1, 0, 0, 96, 122, 0, 0, 0, 0, 4, 0, 0, 0, 0 ]);
sendStartMovementRelArray = bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 95, 0]);

        #const unsigned char sendStartMovementRel[21] = { 0,0,0,0,0,15,0,43,13,1,0,0,96,64,0,0,0,0,2,95,0 };

class D1:
    def __init__(self, IP_Adress, Port, Axis):
        self.IP_Adress=IP_Adress # Define the IP Adress
        self.Socket=Port # Define the port (default:502) to create socket
        self.Axis=Axis
        self.cur_position = 0
        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except socket.error:
            print ('failed to create socket')         
        self.s.connect((IP_Adress, Port))

        if (int.from_bytes(self.sendCommand(SI_unit_array)[21:22],byteorder='big') == 1):
            # Equation to calculate the multiplication factor from the recieved byte 3 of object 60A8h
            if (int.from_bytes(self.sendCommand(SI_unit_array)[22:],byteorder='little') > 5):
                self.SI_unit_fact=(10 ** -3)/(10 ** ((int.from_bytes(self.sendCommand(SI_unit_array)[22:],byteorder='little'))-256))
            if (int.from_bytes(self.sendCommand(SI_unit_array)[22:],byteorder='little') < 5):
                self.SI_unit_fact=(10 ** -3)/(10 ** ((int.from_bytes(self.sendCommand(SI_unit_array)[22:],byteorder='little'))))
        # Read the SI Unit Position and calculation of the multiplication factor when rotary movement(byte 2 == 41h) is set; for further informations please see manual chapter "Detailed description Motion Control Object" Object 60A8h and Object 6092h
        if (int.from_bytes(self.sendCommand(SI_unit_array)[21:22],byteorder='big') == 65):
            # Equation to calculate the multiplication factor from the recieved byte 3 of object 60A8h
            if (int.from_bytes(self.sendCommand(SI_unit_array)[22:],byteorder='little') > 5):
                self.SI_unit_fact=(10 ** (-1 * ((int.from_bytes(self.sendCommand(SI_unit_array)[22:],byteorder='little'))-256)))
            if (int.from_bytes(self.sendCommand(SI_unit_array)[22:],byteorder='little') < 5):
                self.SI_unit_fact=(10 ** (-1 * ((int.from_bytes(self.sendCommand(SI_unit_array)[22:],byteorder='little')))))
        # 6092h_02h Feed constant Subindex 2 (Shaft revolutions)
        # Set shaft revolutions to 1; refer to manual (Byte 19 = 1)
        self.sendCommand(bytearray([0, 0, 0, 0, 0, 14, 0, 43, 13, 1, 0, 0, 96, 146, 2, 0, 0, 0, 1, 1]))
        # Read the feed rate from the D1 and convert it to integer [mm]
        self.feed_rate=(int.from_bytes(self.sendCommand(feedrate_array)[19:],byteorder='little'))/self.SI_unit_fact
	
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

        return (position)
    

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


    # Function to Convert Integer to 4 Byte; further information see manual chapter "Conversion Decimal into Double Word Decimal"
    def to_four_byte(self,x):
        # converts integer to 4 byte array
        sequence=list(struct.unpack("4b", struct.pack("i",x)))
        for i in range(0,4):
            if sequence[i]<0:
                sequence[i]=256+sequence[i]
        return sequence
    # Function "Profile Position Mode"; Move to an absolute position[mm] with given velocity[mm/s] and acceleration[mm/s²]
    def profile_pos_mode(self,position,velo,acc):
        self.velocity=int(velo * self.SI_unit_fact) # velocity multiplied by SI unit factor --> value that needs to be send by telegram
        self.acceleration=int(acc*self.SI_unit_fact) # acceleration multiplied by SI unit factor --> value that needs to be send by telegram
        self.position_value = int(position*self.SI_unit_fact) # positon in units
        # 6060h Modes of Operation
        # Set Profile Position Mode (see "def set_mode(mode):"; Byte 19 = 1)
        self.setMode(1)
        # 6081h Profile Velocity
        # Set velocity to the entered value
        self.sendCommand(bytearray([0, 0, 0, 0, 0, 17, 0, 43, 13, 1, 0, 0, 96, 129, 0, 0, 0, 0, 4, self.to_four_byte( self.velocity)[0], self.to_four_byte( self.velocity)[1], self.to_four_byte( self.velocity)[2], self.to_four_byte( self.velocity)[3]]))
        # 6083h Profile Acceleration
        # Set acceleration to to the entered value
        self.sendCommand(bytearray([0, 0, 0, 0, 0, 17, 0, 43, 13, 1, 0, 0, 96, 131, 0, 0, 0, 0, 4, self.to_four_byte( self.acceleration)[0], self.to_four_byte( self.acceleration)[1], self.to_four_byte( self.acceleration)[2], self.to_four_byte( self.acceleration)[3]]))
        # Send Telegram(TX) Write Target Position 607Ah "Write Value"
        self.sendCommand(bytearray([0, 0, 0, 0, 0, 17, 0, 43, 13, 1, 0, 0, 96, 122, 0, 0, 0, 0, 4, self.to_four_byte(self.position_value)[0], self.to_four_byte(self.position_value)[1], self.to_four_byte(self.position_value)[2], self.to_four_byte(self.position_value)[3]]))
        # Enable Operation to set bit 4 of the controlword to low again; see manual chapter "Controlword"
        self.sendCommand(enableOperationArray_relative)
        # Send Telegram(TX) Write Controlword 6040h Command: Start Movement; high flank of bit 4

        self.sendCommand(sendStartMovementRelArray)
        # A new movement only starts after completing a movement and the target is reached
        #while (self.sendCommand(statusArray) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22]):
         #   time.sleep(0.01) #delay in seconds delay in seconds between checking for "target reached"

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
	#self.axis_controller.axes[axis].getPosition() 
        self.axis_controller.axes[axis].profile_pos_mode(direction*10, 5,50)
        print(f"Started jogging axis ", axis, "From current postion=")#, self.cur_position, " To desired position = ", desired_postion)

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
            position = "{:.2f}".format(axis.getPosition())
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

    def setTargetPosition(self, axis, desired_position):
        if 0 <= axis < len(self.axes):
            self.axes[axis].targetPosition(desired_position)

if __name__ == "__main__":
    app = ClickAndHoldApp(root, D1AxisController(), position_labels)
    root.mainloop()
