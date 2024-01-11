#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

try:
    from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
except ImportError:
    from moveit_commander import roscpp_initialize  # If `moveit_commander` is not found, try to import `roscpp_initialize`
    roscpp_initialize([])  # Initialize rospy manually

import moveit_msgs.msg
import geometry_msgs.msg

def pose_callback(msg):
    # Assuming msg_list is a list of Pose messages
    joint_positions = []
    joint_orientations = []


    print(f"Shape of joint_positions: {np.asarray(msg.position)}")

if __name__ == '__main__':
    rospy.init_node("mover_robot_sim", anonymous=True)

    fake_controller_joint_states_pub = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=1)
    rospy.Subscriber('/cloud_pose', Pose, pose_callback)

    rospy.spin()
