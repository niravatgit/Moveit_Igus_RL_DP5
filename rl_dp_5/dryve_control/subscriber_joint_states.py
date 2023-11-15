#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from moveit_commander import move_group as mg
import moveit_msgs

printJointStates = False

def joint_state_callback(data):
    global printJointStates
    printJointStates = data
    #rospy.Header.frame_id = "Joint_States"
    time = rospy.get_rostime()
    if printJointStates:
        print("Position of joints at timestamp " + str(time) + " :", list(data.position))
        printJointStates = False

def listener():
    rospy.init_node('listener', anonymous=True)
    print("Printing Joint states of all joints")
    joint_state_positions = rospy.Subscriber("/joint_states", JointState, joint_state_callback, queue_size=1)
    rospy.spin()

    return joint_state_positions

if __name__ == '__main__':
    while not rospy.is_shutdown():
        listener()
        rospy.spin()


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
     