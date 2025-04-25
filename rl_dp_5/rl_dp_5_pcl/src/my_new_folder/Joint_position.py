#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
import numpy as np

def publish_joint_positions():
    rospy.init_node('joint_position_publisher', anonymous=True)
    pub = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=10)
    
    rate = rospy.Rate(1)  # 1 Hz

    # Joint names and positions (in radians)
    joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
    
    # Degrees to radians
    joint_positions_deg = [0, 0, 0, 0, 0.88]
    joint_positions_rad = [np.deg2rad(angle) for angle in joint_positions_deg]

    while not rospy.is_shutdown():
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = joint_names
        joint_state_msg.position = joint_positions_rad

        pub.publish(joint_state_msg)
        rospy.loginfo(f"Published joint positions: {joint_positions_rad}")
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_joint_positions()
    except rospy.ROSInterruptException:
        pass

