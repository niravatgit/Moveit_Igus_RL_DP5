#****************************************************************************************************************************
# Python sample programm demonstrating the communication/control of two dryve D1 via Modbus TCP as a Gateway (Ethernet based)
# Version 1.2

# Programing language: Python 3.8
# Editor: Visual Studio Code 1.60.0

# Dual axis control for point to point movement and 2 axis gantry robot
# The supplied dryve D1 configuration must be loaded: "D1-2-RaspberryPI-ModbusTCP(GW)-14-V1.2-1.txt" and "D1-2-RaspberryPI-ModbusTCP(GW)-14-V1.2-2.txt"
# The inital parameters feed rate, available stroke, mode of homing and homing offset have to be set according to your system beforehand in the GUI
# The IP address of the dryve D1 must match with the IP address stated at line 237 or 238.
# From line 34 to 233: Definition of class and functions
# From line 235 to end: Start Main Program; Object declaration and calling of functions defined in the class to perform movements
# All movement parameter can be adopted. E.g. speed, acceleration or position.

# Please use the latest firmware available at www.igus.eu/D1!!!

# No support is provided for this sample program.
# No responsibility/liability will be assumed for the test program.
#***************************************************************************************************************************


# Import libraries 
import socket
import time
import struct

# Variables start value
start = 0
ref_done_X = 0
ref_done_Y = 0
var = 1

# class definition for D1 controler; definiton of functions; establishing of the ethernet connection; basis settings and initialization
# !!!How to declare object and call functions defined in the class to perform movements can be seen from line 202!!!
class D1:
    def __init__(self, IP_Adress, Port, Axis):
        self.debug=False # "True" = print command enabled, only for easier debugging, "False" = print command disabled 
        self.pause=True # "True" = enable delay, "False" = disable delay
        self.multi_move=False # "True" = simultaneous movements of the two controlers, "False" = movement has to be finished by one controler before starting a new one
        self.IP_Adress=IP_Adress # Define the IP Adress
        self.Socket=Port # Define the port (default:502) to create socket
        self.Axis=Axis # Label for the connected D1
                                        
        # Definition of important Telegrams
        self.status_array = bytearray([0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2]) # Send Telegram(TX) Read Statusword 6041h "Status Request"; refer to manual chapter "RX/TX Telegram Example"
        self.shutdown_array = bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 6, 0]) # Send Telegram(TX) Write Controlword 6040h Command: Shutdown; refer to manual chapter "RX/TX Telegram Example"
        self.switchOn_array = bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 7, 0]) # Send Telegram(TX) Write Controlword 6040h Command: Switch on; refer to manual chapter "RX/TX Telegram Example"
        self.enableOperation_array = bytearray([0, 0, 0, 0, 0, 15, 0, 43,13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 15, 0]) # Send Telegram(TX) Write Controlword 6040h Command: Enable Operation; refer to manual chapter "RX/TX Telegram Example"
        self.feedrate_array = bytearray([0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 146, 1, 0, 0, 0, 4]) # Send Telegram(TX) Read Obejct 6092h subindex 1 for the feed rate
        self.SI_unit_array = bytearray([0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 168, 0, 0, 0, 0, 4]) # Send Telegram(TX) Read Object 60A8h for SI Unit Position
        self.DInputs_array = bytearray([0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 253, 0, 0, 0, 0, 4]) # Send Telegram(TX) Read Object 60FDh for Status of digital Inputs
        self.reset_array = bytearray([0, 0, 0, 0, 0, 15, 0, 43,13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 0, 1]) # Send Telegram(TX) Write Controlword 6040h Command to reset the dryve status

        # Establish bus connection
        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except socket.error:
            print ('failed to create socket')         
        self.s.connect((IP_Adress, Port))
        if self.debug==True:
            print ('Socket created')

        # Initialize
        self.initialize()
        # Read the SI Unit Position calculation of the multiplication factor when linear movement(byte 2 == 01h) is set; for further informations please see manual chapter "Detailed description Motion Control Object" Object 60A8h and Object 6092h
        if (int.from_bytes(self.sendCommand(self.SI_unit_array)[21:22],byteorder='big') == 1):
            # Equation to calculate the multiplication factor from the recieved byte 3 of object 60A8h
            if (int.from_bytes(self.sendCommand(self.SI_unit_array)[22:],byteorder='little') > 5):
                self.SI_unit_fact=(10 ** -3)/(10 ** ((int.from_bytes(self.sendCommand(self.SI_unit_array)[22:],byteorder='little'))-256))
            if (int.from_bytes(self.sendCommand(self.SI_unit_array)[22:],byteorder='little') < 5):
                self.SI_unit_fact=(10 ** -3)/(10 ** ((int.from_bytes(self.sendCommand(self.SI_unit_array)[22:],byteorder='little'))))
        # Read the SI Unit Position and calculation of the multiplication factor when rotary movement(byte 2 == 41h) is set; for further informations please see manual chapter "Detailed description Motion Control Object" Object 60A8h and Object 6092h
        if (int.from_bytes(self.sendCommand(self.SI_unit_array)[21:22],byteorder='big') == 65):
            # Equation to calculate the multiplication factor from the recieved byte 3 of object 60A8h
            if (int.from_bytes(self.sendCommand(self.SI_unit_array)[22:],byteorder='little') > 5):
                self.SI_unit_fact=(10 ** (-1 * ((int.from_bytes(self.sendCommand(self.SI_unit_array)[22:],byteorder='little'))-256)))
            if (int.from_bytes(self.sendCommand(self.SI_unit_array)[22:],byteorder='little') < 5):
                self.SI_unit_fact=(10 ** (-1 * ((int.from_bytes(self.sendCommand(self.SI_unit_array)[22:],byteorder='little')))))
        # 6092h_02h Feed constant Subindex 2 (Shaft revolutions)
        # Set shaft revolutions to 1; refer to manual (Byte 19 = 1)
        self.sendCommand(bytearray([0, 0, 0, 0, 0, 14, 0, 43, 13, 1, 0, 0, 96, 146, 2, 0, 0, 0, 1, 1]))
        # Read the feed rate from the D1 and convert it to integer [mm]
        self.feed_rate=(int.from_bytes(self.sendCommand(self.feedrate_array)[19:],byteorder='little'))/self.SI_unit_fact
                    
    # Function to send and recieve response telegrams
    def sendCommand(self,data):
        # Create socket and send request
        self.s.send(data)
        res = self.s.recv(24)
        if self.debug==True:
            # Print response telegram
            print(list(res))
        return list(res)

    # Function to read the value of Digital Inputs
    def readDigitalInput(self):
        return self.sendCommand(self.DInputs_array)
        
    # Function to Convert Integer to 4 Byte; further information see manual chapter "Conversion Decimal into Double Word Decimal"
    def to_four_byte(self,x):
        # converts integer to 4 byte array
        sequence=list(struct.unpack("4b", struct.pack("i",x)))
        for i in range(0,4):
            if sequence[i]<0:
                sequence[i]=256+sequence[i]
        return sequence

    # Function "Shutdown"; can be used to shutdown the D1 controler; further information see manual chapter "State Machine Visualisation after Boot Up"
    def set_shutdn(self):      
        self.sendCommand(self.shutdown_array) # Send Telegram(TX) Write Controlword 6040h Command: Shutdown
        while (self.sendCommand(self.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 33, 6]     # Wait for Shutdown to be set
            and self.sendCommand(self.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 33, 22]
            and self.sendCommand(self.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 33, 2]):
            if self.debug==True:
                print("wait for shutdown")
            if self.pause==True:          
                time.sleep(0.01)  # delay in seconds between checking if mode is set

    # Function "Switch on"; further information see manual chapter "State Machine Visualisation after Boot Up"
    def set_swon(self): 
        self.sendCommand(self.switchOn_array) # Send Telegram(TX) Write Controlword 6040h Command: Switch On
        while (self.sendCommand(self.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 35, 6]   # Wait for Switch on to be set
            and self.sendCommand(self.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 35, 22]
            and self.sendCommand(self.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 35, 2]):
            if self.debug==True:
                print("wait for Switch on")
            if self.pause==True:          
                time.sleep(0.01)  # delay in seconds between checking if mode is set

    # Function "Enable Operation"; further information see manual chapter "State Machine Visualisation after Boot Up"
    def set_op_en(self): 
        self.sendCommand(self.enableOperation_array) # Send Telegram(TX) Write Controlword 6040h Command: Enable Operation
        while (self.sendCommand(self.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 6]   # Wait for Enable Operation to be set
            and self.sendCommand(self.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22]
            and self.sendCommand(self.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 2]):
            if self.debug==True:
                print("wait for op en")
            if self.pause==True:          
                time.sleep(0.01) # delay in seconds between checking if mode is set
    
    # Function to intinalize the "State Machine"; further information see manual chapter "State Machine Visualisation after Boot Up"
    def initialize(self):
        # Call of the function to run through the State Machine and make D1 ready for further usage(e.g Homing, Profile Postion Mode, etc.); see operating manual chapter "State Machine Visualisation after Boot Up"
        self.sendCommand(self.reset_array)
        self.set_shutdn()
        self.set_swon()
        self.set_op_en()

    # Function to change the mode of operation; change between Homing, Profile Position Mode, etc.; further information see manual chapter "Homing" and following
    def set_mode(self,mode):
        # Set operation modes in object 6060h Modes of Operation
        self.sendCommand(bytearray([0, 0, 0, 0, 0, 14, 0, 43, 13, 1, 0, 0, 96, 96, 0, 0, 0, 0, 1, mode]))
        # Wait for the mode to be set
        while (self.sendCommand(bytearray([0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 97, 0, 0, 0, 0, 1])) != [0, 0, 0, 0, 0, 14, 0, 43, 13, 0, 0, 0, 96, 97, 0, 0, 0, 0, 1, mode]):
            if self.debug==True:
                print("wait for mode")
            if self.pause==True:          
                time.sleep(0.01) # delay in seconds between checking if mode is set

    # Function to check for errors in the D1
    def check_error(self):
        if (self.sendCommand(self.status_array) == [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 8, 22]
            or self.sendCommand(self.status_array) == [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 8, 6]
            or self.sendCommand(self.status_array) == [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 8, 34]
            or self.sendCommand(self.status_array) == [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 8, 2]):
            self.error = 1
        else:
            self.error = 0
        return self.error

    # Function to wait for movement to be finished
    def wait_for_ready(self):
        # Checks the status if target was reached, a new setpoint is given, operation enabled and if there is no error
         while (self.sendCommand(self.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22]
            and self.sendCommand(self.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 8, 22]):
            # If the StopButton is pushed the loop breaks
            if self.readDigitalInput() == [0, 0, 0, 0, 0, 17, 0, 43, 13, 0, 0, 0, 96, 253, 0, 0, 0, 0, 4, 8, 0, 66, 0]:
                break
            time.sleep(0.01)
            print ("Wait for Ready")

    # Function "Homing"; executing a homing; mode of homing and offset has to be set beforehand in the GUI
    def homing(self, homespeedsw, homeacc):
        self.homespeedswitch = int(homespeedsw * self.SI_unit_fact) # velocity multiplied by SI unit factor --> value that needs to be send by telegram
        self.homeacceleration = int(homeacc * self.SI_unit_fact) # acceleration multiplied by SI unit factor --> value that needs to be send by telegram
        # Set Homing mode (see "def set_mode(mode):"; Byte 19 = 6)
        self.set_mode(6)
        # 6099h_01h Homing speeds Switch
        # Speed during search for switch is set to the entered value
        self.sendCommand(bytearray([0, 0, 0, 0, 0, 17, 0, 43, 13, 1, 0, 0, 96, 153, 1, 0, 0, 0, 4, self.to_four_byte( self.homespeedswitch)[0], self.to_four_byte( self.homespeedswitch)[1], self.to_four_byte( self.homespeedswitch)[2], self.to_four_byte( self.homespeedswitch)[3]]))
        # 609Ah Homing acceleration
        # Set Homing acceleration to the entered value
        self.sendCommand(bytearray([0, 0, 0, 0, 0, 17, 0, 43, 13, 1, 0, 0, 96, 154, 0, 0, 0, 0, 4, self.to_four_byte( self.homeacceleration)[0], self.to_four_byte( self.homeacceleration)[1], self.to_four_byte( self.homeacceleration)[2], self.to_four_byte( self.homeacceleration)[3]]))
        # Enable Operation to set bit 4 of the controlword to low again; see manual chapter "Controlword"
        self.set_op_en()
        # Send Telegram(TX) Write Controlword 6040h Command: Start Movement; high flank of bit 4
        self.sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 31, 0]))
        # Check Statusword for signal referenced 
        while (self.sendCommand(self.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22]
            and self.sendCommand(self.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 8, 6]
            and self.sendCommand(self.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 8, 34]
            and self.sendCommand(self.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 8, 2]):
            # If the "Stop" Button is pushed the loop breaks
            if self.readDigitalInput() == [0, 0, 0, 0, 0, 17, 0, 43, 13, 0, 0, 0, 96, 253, 0, 0, 0, 0, 4, 8, 0, 66, 0]:
                break
            time.sleep(0.01)
            print ("Homing")
        
    # Function "Profile Position Mode"; Move to an absolute position[mm] with given velocity[mm/s] and acceleration[mm/s²]
    def profile_pos_mode(self,position,velo,acc):
        self.velocity=int(velo * self.SI_unit_fact) # velocity multiplied by SI unit factor --> value that needs to be send by telegram
        self.acceleration=int(acc*self.SI_unit_fact) # acceleration multiplied by SI unit factor --> value that needs to be send by telegram
        self.position_value = int(position*self.SI_unit_fact) # positon in units
        # 6060h Modes of Operation
        # Set Profile Position Mode (see "def set_mode(mode):"; Byte 19 = 1)
        self.set_mode(1)
        # 6081h Profile Velocity
        # Set velocity to the entered value
        self.sendCommand(bytearray([0, 0, 0, 0, 0, 17, 0, 43, 13, 1, 0, 0, 96, 129, 0, 0, 0, 0, 4, self.to_four_byte( self.velocity)[0], self.to_four_byte( self.velocity)[1], self.to_four_byte( self.velocity)[2], self.to_four_byte( self.velocity)[3]]))
        # 6083h Profile Acceleration
        # Set acceleration to to the entered value
        self.sendCommand(bytearray([0, 0, 0, 0, 0, 17, 0, 43, 13, 1, 0, 0, 96, 131, 0, 0, 0, 0, 4, self.to_four_byte( self.acceleration)[0], self.to_four_byte( self.acceleration)[1], self.to_four_byte( self.acceleration)[2], self.to_four_byte( self.acceleration)[3]]))
        # 6084h Profile Deceleration
        # Set deceleration to the same value as the acceleration
        self.sendCommand(bytearray([0, 0, 0, 0, 0, 17, 0, 43, 13, 1, 0, 0, 96, 132, 0, 0, 0, 0, 4, self.to_four_byte( self.acceleration)[0], self.to_four_byte( self.acceleration)[1], self.to_four_byte( self.acceleration)[2], self.to_four_byte( self.acceleration)[3]]))
        # Send Telegram(TX) Write Target Position 607Ah "Write Value"
        self.sendCommand(bytearray([0, 0, 0, 0, 0, 17, 0, 43, 13, 1, 0, 0, 96, 122, 0, 0, 0, 0, 4, self.to_four_byte(self.position_value)[0], self.to_four_byte(self.position_value)[1], self.to_four_byte(self.position_value)[2], self.to_four_byte(self.position_value)[3]]))
        # Enable Operation to set bit 4 of the controlword to low again; see manual chapter "Controlword"
        self.set_op_en()
        # Send Telegram(TX) Write Controlword 6040h Command: Start Movement; high flank of bit 4
        self.sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 31, 0]))
        # A new movement only starts after completing a movement and the target is reached
        if self.multi_move==False:
            while (self.sendCommand(self.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22]
                and self.sendCommand(self.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 8, 22]):
                # If the "Stop" Button is pushed the loop breaks
                if self.readDigitalInput() == [0, 0, 0, 0, 0, 17, 0, 43, 13, 0, 0, 0, 96, 253, 0, 0, 0, 0, 4, 8, 0, 66, 0]:
                    break
                time.sleep(0.01)
                print ("Profile Position Mode")

### Start Main Program: Object declaration and calling of functions defined in the class to perform movements ###
# Setup the communication and initialization of the D1 controller    
Xaxis=D1("169.254.0.1", 502, 'Linear X-Axis')  # Define D1 as X axis (IP_adress , Port , 'label')
Yaxis=D1("169.254.0.2", 502, 'Linear Y-Axis')  # Define D1 as Y axis (IP_adress , Port , 'label')

# Check if an error occurred in the D1 controller
Xaxis.check_error()
Yaxis.check_error()

# If there is no error start main program
while Xaxis.error == 0 and Yaxis.error == 0:
    # Check if Digital Input 1 is High, if High change the variable start to 1
    if Xaxis.readDigitalInput() == [0, 0, 0, 0, 0, 17, 0, 43, 13, 0, 0, 0, 96, 253, 0, 0, 0, 0, 4, 8, 0, 65, 0] or Yaxis.readDigitalInput() == [0, 0, 0, 0, 0, 17, 0, 43, 13, 0, 0, 0, 96, 253, 0, 0, 0, 0, 4, 8, 0, 65, 0]:
        start = 1
   
    # When the Start Button was pressed start movements
    while start == 1:
        # Do the homing once before starting movements
        if ref_done_X == 0:
            Xaxis.homing(30, 600) # Homing (Switch search speed[mm/s], Acceleration for Homing[mm/s²])
            # Check whether someone has stopped during the homing or if an error has occurred
            if (Xaxis.readDigitalInput() == [0, 0, 0, 0, 0, 17, 0, 43, 13, 0, 0, 0, 96, 253, 0, 0, 0, 0, 4, 8, 0, 66, 0]
                or Xaxis.sendCommand(Xaxis.status_array) == [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 8, 6]
                or Xaxis.sendCommand(Xaxis.status_array) == [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 8, 34]
                or Xaxis.sendCommand(Xaxis.status_array) == [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 8, 2] ):
                break
            # Set state to reference done
            ref_done_X = 1
        # Do the homing once before starting movements
        if ref_done_Y == 0:
            Yaxis.homing(30, 600) # Homing (Switch search speed[mm/s], Acceleration for Homing[mm/s²])
            # Check whether someone has stopped during the homing or if an error has occurred
            if (Yaxis.readDigitalInput() == [0, 0, 0, 0, 0, 17, 0, 43, 13, 0, 0, 0, 96, 253, 0, 0, 0, 0, 4, 8, 0, 66, 0]
                or Yaxis.sendCommand(Yaxis.status_array) == [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 8, 6]
                or Yaxis.sendCommand(Yaxis.status_array) == [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 8, 34]
                or Yaxis.sendCommand(Yaxis.status_array) == [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 8, 2] ):
                break
            # Set state to reference done
            ref_done_Y = 1
        time.sleep(0.1) # Wait for 100 ms before starting new movement
        # While reference is done do movements
        while ref_done_X == 1 and ref_done_Y == 1:
            Xaxis.multi_move=True   # Enable multi axis move for X axis
            Yaxis.multi_move=True   # Enable multi axis move for Y axis
            if var == 1:
                Xaxis.profile_pos_mode(120, 30, 2000) # Profile Position Mode (Position[mm], Velocity[mm/s], Acceleration[mm/s²]) Moves the X axis to absolute position 300 mm 
                Yaxis.profile_pos_mode(60, 15, 2000) # Profile Position Mode (Position[mm], Velocity[mm/s], Acceleration[mm/s²]) Moves the Y axis to absolute position 300 mm 
                # Checks the status if target was reached, a new setpoint is given, operation enabled and if there is no error
                while (Xaxis.sendCommand(Xaxis.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22]
                    and Yaxis.sendCommand(Yaxis.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22]):
                    # If the "Stop" Button is pushed the loop breaks
                    if Xaxis.readDigitalInput() == [0, 0, 0, 0, 0, 17, 0, 43, 13, 0, 0, 0, 96, 253, 0, 0, 0, 0, 4, 8, 0, 66, 0] or Yaxis.readDigitalInput() == [0, 0, 0, 0, 0, 17, 0, 43, 13, 0, 0, 0, 96, 253, 0, 0, 0, 0, 4, 8, 0, 66, 0]:
                        var = 1
                        break
                    time.sleep(0.01)
                    print ("Wait for Ready") # Wait for the Xaxis to complete the movement and reach the target; in the function a delay of 100 ms is set to wait after the movement has finished
                if Xaxis.readDigitalInput() == [0, 0, 0, 0, 0, 17, 0, 43, 13, 0, 0, 0, 96, 253, 0, 0, 0, 0, 4, 8, 0, 66, 0] or Yaxis.readDigitalInput() == [0, 0, 0, 0, 0, 17, 0, 43, 13, 0, 0, 0, 96, 253, 0, 0, 0, 0, 4, 8, 0, 66, 0]:
                    break
                if Xaxis.sendCommand(Xaxis.status_array) == [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22] and Yaxis.sendCommand(Yaxis.status_array) == [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22]:
                    var = 2
                    time.sleep(1) # Wait for 100 ms before starting new movement
            if var == 2:
                Xaxis.profile_pos_mode(0, 30, 2000) # Profile Position Mode (Position[mm], Velocity[mm/s], Acceleration[mm/s²]) Moves the X axis to absolute position 0 mm 
                Yaxis.profile_pos_mode(0, 15, 2000) # Profile Position Mode (Position[mm], Velocity[mm/s], Acceleration[mm/s²]) Moves the Y axis to absolute position 0 mm 
                # Checks the status if target was reached, a new setpoint is given, operation enabled and if there is no error
                while (Xaxis.sendCommand(Xaxis.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22]
                    and Yaxis.sendCommand(Yaxis.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22]):
                    # If the "Stop" Button is pushed the loop breaks
                    if Xaxis.readDigitalInput() == [0, 0, 0, 0, 0, 17, 0, 43, 13, 0, 0, 0, 96, 253, 0, 0, 0, 0, 4, 8, 0, 66, 0] or Yaxis.readDigitalInput() == [0, 0, 0, 0, 0, 17, 0, 43, 13, 0, 0, 0, 96, 253, 0, 0, 0, 0, 4, 8, 0, 66, 0]:
                        var = 2
                        break
                    time.sleep(0.01)
                    print ("Wait for Ready") # Wait for the Xaxis to complete the movement and reach the target; in the function a delay of 100 ms is set to wait after the movement has finished
                if Xaxis.readDigitalInput() == [0, 0, 0, 0, 0, 17, 0, 43, 13, 0, 0, 0, 96, 253, 0, 0, 0, 0, 4, 8, 0, 66, 0] or Yaxis.readDigitalInput() == [0, 0, 0, 0, 0, 17, 0, 43, 13, 0, 0, 0, 96, 253, 0, 0, 0, 0, 4, 8, 0, 66, 0]:
                    break
                if Xaxis.sendCommand(Xaxis.status_array) == [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22] and Yaxis.sendCommand(Yaxis.status_array) == [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22]:
                    var = 1
                    time.sleep(1) # Wait for 100 ms before starting new movement
        # Check whether someone has stopped during the movement or if an error has occurred
        if (Xaxis.readDigitalInput() == [0, 0, 0, 0, 0, 17, 0, 43, 13, 0, 0, 0, 96, 253, 0, 0, 0, 0, 4, 8, 0, 66, 0]
            or Yaxis.readDigitalInput() == [0, 0, 0, 0, 0, 17, 0, 43, 13, 0, 0, 0, 96, 253, 0, 0, 0, 0, 4, 8, 0, 66, 0]):
            break
    
    # If "Stop" is pressed, set bit 8 of the control word high --> Stop the Movement        
    if Xaxis.readDigitalInput() == [0, 0, 0, 0, 0, 17, 0, 43, 13, 0, 0, 0, 96, 253, 0, 0, 0, 0, 4, 8, 0, 66, 0] or Yaxis.readDigitalInput() == [0, 0, 0, 0, 0, 17, 0, 43, 13, 0, 0, 0, 96, 253, 0, 0, 0, 0, 4, 8, 0, 66, 0]:
        start = 0
        Xaxis.sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43,13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 15, 1]))
        Yaxis.sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43,13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 15, 1]))
        time.sleep(0.1)
        # If an error has occurred, the loop is interrupted    
    if (Xaxis.check_error() == 1 or Yaxis.check_error() == 1):
        break
    print ("Wait for Start")    

print("Error on D1")       
