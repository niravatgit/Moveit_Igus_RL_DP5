# Created by Laugesen, Nichlas O.; Dam, Elias Thomassen.
# built from "Manual/Operating Manual dryve D1 EN V3.0.1.pdf"

#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import dryve_D1 as dryve
import numpy as np


speed=5
accel=100
homespeed=5
homeaccel=100


#following are the fucntions that we want to expose through ROS
#workspace will be /rl_dp_5
#1. publisher: /status for a joint such as {mode of operation, current position, is_initialized }, this will require calling multiple functiosn from dryve_D1.py
#2. service: /setMode : integer as an input passed on to function set_mode from dryve_D1.py -> check the arguments
#3. service: /home : this will call homing from dryve_D1.py -> check the arguments
#4. subsriber: /cmd/set_joint_position : this will set desired joint position by calling profile_pos_mode -> check arguments
#5. 
#
#
#
#
#
#
#start ROS Node code here


class Rl_DP_5:
    def __init__(self):
        # Initialize your 5 D1 axes here
        Aaxis = dryve.D1("169.254.0.1", 502, 'Axis 1', -140, -140, 140)
        Baxis = dryve.D1("169.254.0.2", 502, 'Axis 2', -100, -100, 50)
        Caxis = dryve.D1("169.254.0.3", 502, 'Axis 3', -115, -115, 115)
        Daxis = dryve.D1("169.254.0.4", 502, 'Axis 4', -100, -100, 100)
        Eaxis = dryve.D1("169.254.0.5", 502, 'Axis 5', -180, -179, 179)

        self.axis_controller = [Aaxis, Baxis, Caxis, Daxis, Eaxis]
        print('Created dryve interfaces')

    def setTargetPosition(self, axis, desired_absolute_position):
        if 0 <= axis < len(self):
            self.axis_controller[axis].profile_pos_mode(desired_absolute_position, 5, 50)

    def home(self, axis):
        print(f"Started homing Axis {axis + 1}")
        self.axis_controller[axis].homing(homespeed, homeaccel)

    def homeAll(self):
        for axis in self.axis_controller:
            print(f"Started homing {axis.Axis}")
            axis.homing(homespeed, homeaccel)

    def get_current_position(self, axis):
        return self.axis_controller[axis].getPosition()

class MoveItInterface:
    def __init__(self, robot):
#here we should have two things
#1. Subsriber to joint_states from moveit to getjoint space trajectory to plannned pose
#2. Published to fake_joint_controller to simulate the robot pose in Moveit based on current postiison of the real robot

        self.robot = robot
        print('Created dryve interfaces')
        rospy.init_node('joint_states_subscriber', anonymous=True)

        # Publisher to simulate the robot pose in MoveIt based on the current position of the real robot
        self.fake_controller_joint_states_pub = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=10)
        print('Publishing values to fake controller joint states:', self.fake_controller_joint_states_pub)
        self.position_history = []
        

        # Subscriber to get joint states during trajectory planning from MoveIt
        rospy.Subscriber('/move_group/joint_states', JointState, self.joint_states_callback)
        print('subscribing to the move_group joint states')



    def publish_current_positions(self):
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
        joint_state.position = [np.deg2rad(self.robot.get_current_position(i)) for i in range(5)]
        
        print('Printing joint state positional values:', joint_state.position)

        # Publish joint state to simulate the robot pose in MoveIt
        self.fake_controller_joint_states_pub.publish(joint_state)


    def joint_states_callback(self, data):
        trajectory_points = []
        joint_state = JointState()
        joint_state.header = data.header
        joint_state.name = data.name
        joint_state.position = data.position

        # Publish joint state
        self.joint_states_pub.publish(joint_state)

        # Check for repeated values
        if self.check_repeated_values(joint_state.position, 20):
            rospy.loginfo("Trajectory planned after 20 repeated positions.")
            trajectory_points.append(joint_state.position)
            rospy.signal_shutdown("Trajectory planned.")
            
        print(trajectory_points)

    def check_repeated_values(self, current_values, threshold):
        self.position_history.append(current_values)
        if len(self.position_history) >= threshold:
            recent_positions = self.position_history[-threshold:]
            return all(positions == current_values for positions in recent_positions)
        return False
if __name__ == "__main__":
    print('Initialized an object for the robot')
    robot = Rl_DP_5()
    print('Initialized an object for Moveit interface')
    move_it_interface = MoveItInterface(robot)

    try:
        while not rospy.is_shutdown():
            print('publishing the positional data from the robot')
            move_it_interface.publish_current_positions()
            rospy.sleep(1)
    except rospy.ROSInterruptException:
        pass

