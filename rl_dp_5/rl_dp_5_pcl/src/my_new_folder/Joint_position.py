#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
import time

def run_automated_trajectory():
    rospy.init_node('automated_trajectory_executor')
    
    # Publisher to send joint positions
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz (adjust speed as needed)

    # Your 8 pre-calculated joint positions (replace with your actual values)
    joint_positions = [
        [0, 0, 0, 0, 0.88],  # Position 1 (radians)
        [0, 0, 0, 0, 0.89],  # Position 2
        [0, 0, 0, 0, 0.85],  # Position 3
        [0, 0, 0, 0, 0],  # Position 4
        [0, 0, 0, 0, 0.82],  # Position 5
        [0, 0, 0, 0, 0.85],  # Position 6
        [0, 0, 0, 0, 0.89],  # Position 7
        [0, 0, 0, 0, 0]   # Position 8
    ]

    rospy.loginfo("Starting automated trajectory...")
    
    for i, positions in enumerate(joint_positions):
        if rospy.is_shutdown():
            break

        rospy.loginfo(f"Moving to Position {i+1}: {positions}")

        # Publish joint positions
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
        js.position = positions
        pub.publish(js)

        # Wait before next position (adjust delay as needed)
        time.sleep(2.0)  
        rate.sleep()

    rospy.loginfo("Trajectory complete!")

if __name__ == '__main__':
    try:
        run_automated_trajectory()
    except rospy.ROSInterruptException:
        pass
