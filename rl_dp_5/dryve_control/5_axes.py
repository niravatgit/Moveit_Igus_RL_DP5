#****************************************************************************************************************************
# Python sample programm demonstrating the communication/control of two dryve D1 via Modbus TCP as a Gateway (Ethernet based)
# Version 1.2

# Programing language: Python 3.8
# Editor: Visual Studio Code 1.60.0

# Dual axis control for point ot point movement and 2 axis gantry robot
# The supplied dryve D1 configuration must be loaded: "20220428_D1-2-PC-ModbusTCP(GW)-12-V1.2-1.txt" and "20220428_D1-2-PC-ModbusTCP(GW)-12-V1.2-2.txt"
# The inital parameters feed rate, available stroke, mode of homing and homing offset have to be set according to your system beforehand in the GUI
# The IP address of the dryve D1 must match with the IP address stated at line 204 or 205.
# From line 29 to 200: Definition of class and functions
# From line 202 to end: Start Main Program; Object declaration and calling of functions defined in the class to perform movements
# All movement parameter can be adopted. E.g. speed, acceleration or position.

# Please use the latest firmware available at www.igus.eu/D1!!!

# No support is provided for this sample program.
# No responsibility/liability will be assumed for the test program.
#***************************************************************************************************************************


# Import libraries 
import socket
import time
import struct

# class definition for D1 controler; definiton of functions; establishing of the ethernet connection; basis settings and initialization
# !!!How to declare object and call functions defined in the class to perform movements can be seen from line 202!!!
class D1:
    def __init__(self, IP_Adress, Port, Axis):
        self.debug=True # "True" = print command enabled, only for easier debugging, "False" = print command disabled 
        self.pause=True # "True" = enable delay, "False" = disable delay
        self.multi_move=False # "True" = simultaneous movements of the two controlers, "False" = movement has to be finished by one controler before starting a new one
        self.IP_Adress=IP_Adress # Define the IP Adress
        self.Socket=Port # Define the port (default:502) to create socket
        self.Axis=Axis # Label for the connected D1
                                        
        # Definition of important Telegrams
        self.current_position_array = bytearray([0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 100, 0, 0, 0, 0, 2]) # Send Telegram(TX) Read Statusword 6064h "Current position"; refer to manual chapter "RX/TX Telegram Example"
        self.status_array = bytearray([0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2]) # Send Telegram(TX) Read Statusword 6041h "Status Request"; refer to manual chapter "RX/TX Telegram Example"
        self.shutdown_array = bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 6, 0]) # Send Telegram(TX) Write Controlword 6040h Command: Shutdown; refer to manual chapter "RX/TX Telegram Example"
        self.switchOn_array = bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 7, 0]) # Send Telegram(TX) Write Controlword 6040h Command: Switch on; refer to manual chapter "RX/TX Telegram Example"
        self.enableOperation_array = bytearray([0, 0, 0, 0, 0, 15, 0, 43,13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 15, 0]) # Send Telegram(TX) Write Controlword 6040h Command: Enable Operation; refer to manual chapter "RX/TX Telegram Example"
        self.feedrate_array = bytearray([0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 146, 1, 0, 0, 0, 4]) # Send Telegram(TX) Read Obejct 6092h subindex 1 for the feed rate
        self.SI_unit_array = bytearray([0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 168, 0, 0, 0, 0, 4]) # Send Telegram(TX) Read Object 60A8h for SI Unit Position
        self.reset_array = bytearray([0, 0, 0, 0, 0, 15, 0, 43,13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 0, 1]) # Send Telegram(TX) Write Controlword 6040h Command to reset the dryve status
        self.sendStartMovementRelArray = bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 95, 0]); # for realtive movement

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

    # Function to Convert Integer to 4 Byte; further information see manual chapter "Conversion Decimal into Double Word Decimal"
    def to_four_byte(self,x):
        # converts integer to 4 byte array
        sequence=list(struct.unpack("4b", struct.pack("i",x)))
        for i in range(0,4):
            if sequence[i]<0:
                sequence[i]=256+sequence[i]
        return sequence

    #function to get current position
    def get_current_position(self):
        print(self.sendCommand(self.current_position_array))
    # Function "Shutdown"; can be used to shutdown the D1 controler; further information see manual chapter "State Machine Visualisation after Boot Up"
    def set_shutdn(self):      
        self.sendCommand(self.shutdown_array) # Send Telegram(TX) Write Controlword 6040h Command: Shutdown
        while (self.sendCommand(self.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 33, 6]     # Wait for Shutdown to be set
            and self.sendCommand(self.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 33, 22]
            and self.sendCommand(self.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 33, 2]):
            if self.debug==True:
                print("wait for shutdown")
            if self.pause==True:          
                time.sleep(0.01)  #delay in seconds between checking if mode is set

    # Function "Switch on"; further information see manual chapter "State Machine Visualisation after Boot Up"
    def set_swon(self): 
        self.sendCommand(self.switchOn_array) # Send Telegram(TX) Write Controlword 6040h Command: Switch On
        while (self.sendCommand(self.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 35, 6]   # Wait for Switch on to be set
            and self.sendCommand(self.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 35, 22]
            and self.sendCommand(self.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 35, 2]):
            if self.debug==True:
                print("wait for Switch on")
            if self.pause==True:          
                time.sleep(0.01)  #delay in seconds between checking if mode is set

    # Function "Enable Operation"; further information see manual chapter "State Machine Visualisation after Boot Up"
    def set_op_en(self): 
        self.sendCommand(self.enableOperation_array) # Send Telegram(TX) Write Controlword 6040h Command: Enable Operation
        while (self.sendCommand(self.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 6]   # Wait for Enable Operation to be set
            and self.sendCommand(self.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22]
            and self.sendCommand(self.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 2]):
            if self.debug==True:
                print("wait for op en")
            if self.pause==True:          
                time.sleep(0.01) #delay in seconds between checking if mode is set
    
    # Function to intinalize the "State Machine"; further information see manual chapter "State Machine Visualisation after Boot Up"
    def initialize(self):
        # Call of the function to run through the State Machine and make D1 ready for further usage(e.g Homing, Profile Postion Mode, etc.); see operating manual chapter "State Machine Visualisation after Boot Up"
        self.sendCommand(self.reset_array)
        self.set_shutdn()
        self.set_swon()
        self.set_op_en()

    # Function to change the mode of operation; change between Homing, Profile Position Mode, etc.; further information see manual chapter "Homing" and following
    def set_mode(self,mode):
        #Set operation modes in object 6060h Modes of Operation
        self.sendCommand(bytearray([0, 0, 0, 0, 0, 14, 0, 43, 13, 1, 0, 0, 96, 96, 0, 0, 0, 0, 1, mode]))
        # Wait for the mode to be set
        while (self.sendCommand(bytearray([0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 97, 0, 0, 0, 0, 1])) != [0, 0, 0, 0, 0, 14, 0, 43, 13, 0, 0, 0, 96, 97, 0, 0, 0, 0, 1, mode]):
            if self.debug==True:
                print("wait for mode")
            if self.pause==True:          
                time.sleep(0.01) #delay in seconds between checking if mode is set

    # Function to wait for movement to be finished
    def wait_for_ready(self):
        # Checks the status if target was reached and a new setpoint is given, also if operation enabled
         while (self.sendCommand(self.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22]):
                if self.debug==True:
                    print("wait for next command")
                if self.pause==True:          
                    time.sleep(0.01) #delay in seconds between checking if target reached

    # Function "Homing"; executing a homing; mode of homing and offset has to be set beforehand in the GUI
    def homing(self, homespeedsw, homeacc):
        self.homespeedswitch = int(homespeedsw * self.SI_unit_fact) # velocity multiplied by SI unit factor --> value that needs to be send by telegram
        self.homeacceleration = int(homeacc * self.SI_unit_fact) # acceleration multiplied by SI unit factor --> value that needs to be send by telegram
        #Set Homing mode (see "def set_mode(mode):"; Byte 19 = 6)
        self.set_mode(6)
        # 6099h_01h Homing speeds Switch
        # Speed during search for switch is set to the entered value
        self.sendCommand(bytearray([0, 0, 0, 0, 0, 17, 0, 43, 13, 1, 0, 0, 96, 153, 1, 0, 0, 0, 4, self.to_four_byte( self.homespeedswitch)[0], self.to_four_byte( self.homespeedswitch)[1], self.to_four_byte( self.homespeedswitch)[2], self.to_four_byte( self.homespeedswitch)[3]]))
        # 609Ah Homing acceleration
        # Set Homing acceleration to the entered value
        self.sendCommand(bytearray([0, 0, 0, 0, 0, 17, 0, 43, 13, 1, 0, 0, 96, 154, 0, 0, 0, 0, 4, self.to_four_byte( self.homeacceleration)[0], self.to_four_byte( self.homeacceleration)[1], self.to_four_byte( self.homeacceleration)[2], self.to_four_byte( self.homeacceleration)[3]]))
        # Enable Operation to set bit 4 of the controlword to low again; see manual chapter "Controlword"
        self.sendCommand(self.enableOperation_array)
        # Send Telegram(TX) Write Controlword 6040h Command: Start Movement; high flank of bit 4
        self.sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 31, 0]))
        # Check Statusword for signal referenced 
        while (self.sendCommand(self.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22]):
                if self.debug==True:
                    print("wait for Homing to end")
                if self.pause==True:          
                    time.sleep(0.01) #delay in seconds between checking for reference status
        
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
        # Send Telegram(TX) Write Target Position 607Ah "Write Value"
        self.sendCommand(bytearray([0, 0, 0, 0, 0, 17, 0, 43, 13, 1, 0, 0, 96, 122, 0, 0, 0, 0, 4, self.to_four_byte(self.position_value)[0], self.to_four_byte(self.position_value)[1], self.to_four_byte(self.position_value)[2], self.to_four_byte(self.position_value)[3]]))
        # Enable Operation to set bit 4 of the controlword to low again; see manual chapter "Controlword"
        self.sendCommand(self.enableOperation_array)
        # Send Telegram(TX) Write Controlword 6040h Command: Start Movement; high flank of bit 4
        self.sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 31, 0]))
        # A new movement only starts after completing a movement and the target is reached
        if self.multi_move==False:
            while (self.sendCommand(self.status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22]):
                if self.debug==True:
                    print("wait for next command")
                if self.pause==True:          
                    time.sleep(0.01) #delay in seconds delay in seconds between checking for "target reached"

### Start Main Program: Object declaration and calling of functions defined in the class to perform movements ###
# Setup the communication and initialization of the D1 controller    http://169.254.230.10/
Xaxis=D1("169.254.0.1", 502, 'Linear X-Axis')  # Define D1 as X axis (IP_adress , Port , 'label')
#Yaxis=D1("169.254.0.2", 502, 'Linear Y-Axis')  # Define D1 as Y axis (IP_adress , Port , 'label')
#Zaxis=D1("169.254.0.3", 502, 'Linear Z-Axis')  # Define D1 as X axis (IP_adress , Port , 'label')
#Aaxis=D1("169.254.0.4", 502, 'Linear A-Axis')  # Define D1 as Y axis (IP_adress , Port , 'label')
#Baxis=D1("169.254.0.5", 502, 'Linear B-Axis')  # Define D1 as X axis (IP_adress , Port , 'label')
speed=5
accel=100
homespeed=10
homeaccel=100
#while True:
 #   get_current_position()
# Homing of X and Yaxis; only one time to allow absolute movement
if True:
    Xaxis.homing(homespeed, homeaccel) # Homing (Switch search speed[mm/s], Acceleration for Homing[mm/s²])
    Xaxis.profile_pos_mode(50, homespeed, homeaccel)
    time.sleep(0.1) # Wait for 100 ms before starting new movement
    
    #Yaxis.homing(homespeed, homeaccel) # Homing (Switch search speed[mm/s], Acceleration for Homing[mm/s²])
    #Yaxis.profile_pos_mode(90, homespeed, homeaccel)
    #time.sleep(0.1) # Wait for 100 ms before starting new movement

    #Zaxis.homing(homespeed, homeaccel) # Homing (Switch search speed[mm/s], Acceleration for Homing[mm/s²])
    #Zaxis.profile_pos_mode(115, homespeed, homeaccel)
    #time.sleep(0.1) # Wait for 100 ms before starting new movement

    #Aaxis.homing(homespeed, homeaccel) # Homing (Switch search speed[mm/s], Acceleration for Homing[mm/s²])
    #Aaxis.profile_pos_mode(100, homespeed, homeaccel)
    #time.sleep(0.1) # Wait for 100 ms before starting new movement

    #Baxis.homing(homespeed, homeaccel) # Homing (Switch search speed[mm/s], Acceleration for Homing[mm/s²])
    #Baxis.profile_pos_mode(25, speed, 100)
    #time.sleep(0.1) # Wait for 100 ms before starting new movement
    
# Change from while to if to stop the loop; True is set by default so that the commands are executed
while False:
    #Zaxis.profile_pos_mode(10,speed,accel)
    Xaxis.multi_move=True
    Yaxis.multi_move=True
    Zaxis.multi_move=True
    Aaxis.multi_move=True
    Baxis.multi_move=True
    Xaxis.profile_pos_mode(140,speed,accel)
    Yaxis.profile_pos_mode(90,speed,accel)
    Zaxis.profile_pos_mode(25,speed,accel)
    Aaxis.profile_pos_mode(10,speed,accel)
    Baxis.profile_pos_mode(112,speed,accel)
    Xaxis.wait_for_ready()
    Yaxis.wait_for_ready()
    Zaxis.wait_for_ready()
    Aaxis.wait_for_ready()
    Baxis.wait_for_ready()
    time.sleep(0)
232.0, 34.0, 14.0, 145.0, 225.0
69.0, 62.0, 102.0, 199.0, 279.0
140.0, 81.0, 10.0, 32.0, 112.0
140.0, 81.0, 10.0, 32.0, 112.0
