#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import sys
from dryve_D1 import D1

printJointStates = False

def command_handler(command):
    if len(sys.argv)!= []:
        num = len(sys.argv)-1
        print("There are 3 arguments available. They are:")
        print("1. Homing - 'home' \n2. Profile Mode - 'mode'\n3. Status - 'stat'")
    else:
        print('Pass some kind of command to robot')

    #Assuming robot's spped and acceleration components
    if command == 'home':
        print("Homing the robot")
        #D1.homing(self, homespeedsw=1, homeacc=0.5)

    elif command == 'mode':
        print("Mode Selection")
        #D1.profile_pos_mode(D1.getPosition, velo=1, acc=0.5)

    elif command == 'status':
        #Assuming this is the robot initial position from dryve)D1.py
        if D1.get_current_position != bytearray([0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 100, 0, 0, 0, 0, 2]):
            print("Robot is initialized")
        else:
            pass

    else:
        print("Invalid command. Available commands: 'home', 'mode', 'status'")
        sys.exit(1)

if __name__ == '__main__':

    try:
        #d1 = D1(IP_Adress="", Port="", Axis="")
        if len(sys.argv) == 2:
            command = sys.argv[1]
            command_handler(command)
            
    except rospy.ROSInterruptException:
        pass




# #!/usr/bin/env python3
# import rospy
# from sensor_msgs.msg import JointState

# rospy.init_node('joint_state_subscriber')

# last_msg = None

# def test_msg(msg):
# 	global last_msg
# 	last_msg = msg

# def print_msg(msg):
#   if(msg is not None):
#     print('The Joint States of the robot is:', msg)

# sub = rospy.Subscriber('/joint_states', JointState, test_msg, queue_size=1)

# while not rospy.is_shutdown():
# 	print_msg(last_msg)
     