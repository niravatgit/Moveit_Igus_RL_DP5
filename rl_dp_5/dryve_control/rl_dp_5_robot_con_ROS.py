#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
import dryve_D1 as dryve
import numpy as np
import threading
import actionlib
from rldp5_msgs.msg import  rldp5_robotAction, _rldp5_robotGoal, rldp5_robotFeedback, rldp5_robotResult

speed = 5
accel = 100
homespeed = 5
homeaccel = 100

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

    def set_target_position(self, axis, desired_absolute_position):
        if 0 <= axis < len(self.axis_controller):
            self.axis_controller[axis].profile_pos_mode(desired_absolute_position, speed, accel)

    def home(self, axis):
        print(f"Started homing Axis {axis + 1}")
        self.axis_controller[axis].homing(homespeed, homeaccel)

    def home_all(self):
        for axis in self.axis_controller:
            print(f"Started homing {axis.Axis}")
            axis.homing(homespeed, homeaccel)

    def get_current_position(self, axis):
        return self.axis_controller[axis].getPosition()

#-----------------------------------------------------------------------------------------------------------------------------------
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
	#create all poublishedrs, subsribers and action commands of ROS baesd interface
        #Action commands: 1: home <iunt>, home_all, setmode <int>, set_swon, set_open, set_shtdown
	#publishers: status <can iclude a lot of interegers we will discuss later>
	#subsribers: 
#-----------------------------------------------------------------------------------------------------------------------------------

class RL_DP_5_ROS:

    _feedback = rldp5_robotFeedback()
    _result = rldp5_robotResult()

    def __init__(self, robot, name):
        self.robot = robot

        self._action_name = name
        rospy.loginfo("Action server starting...")
        self._as = actionlib.SimpleActionServer(self._action_name, rldp5_robotAction, execute_cb=self.execute_cb, auto_start = False)

       # Start the action server.
        self._as.start()
        rospy.loginfo("Action server started...")

    def execute_cb(self, goal):
        rospy.loginfo("execute_cb starting...")
        self.goal = goal
        success = True
        rospy.loginfo("execute_cb starting...")
        positions = []
        
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False
            
        if self.goal.command == 'home_all':
            self.robot.home_all()
            rospy.loginfo("got goal...")

            for i in range(5):
                positions.append(self.robot.get_current_position(i))
                
            self._feedback.status = positions 
            rospy.loginfo("publishing feedback for axis:")
            self._as.publish_feedback(self._feedback)
            rospy.loginfo("published feedback for axis: ")   
            
        if success:
            self._result.success = self._feedback.status
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
            rospy.loginfo("published goal...")


      
class MoveItInterface:

    def __init__(self, robot):
        self.robot = robot
        self.execution_result = None
        rospy.init_node('joint_states_subscriber', anonymous=True)
        
        # Publisher to simulate the robot pose in MoveIt based on the current position of the real robot
        self.fake_controller_joint_states_pub = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=1)
        print('Publishing values to fake controller joint states:', self.fake_controller_joint_states_pub)
        
        self.joint_state = JointState()
        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_state.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
        self.joint_state.position = list([np.deg2rad(self.robot.get_current_position(i)) for i in range(5)])
        self.fake_controller_joint_states_pub.publish(self.joint_state)

    def listener(self):
        rospy.Subscriber('/joint_states', JointState, self.callback_fn, queue_size=1)

    def callback_fn(self, data):
        self.joint_state_position = list(data.position)
        print(self.joint_state_position)

        self.thread_lock = threading.Lock() 
        with self.thread_lock:
            for i in range(5):
                self.t = threading.Thread(target=self.robot.set_target_position, args=(i, np.rad2deg(self.joint_state_position[i])), daemon=True)
                self.t.start()

if __name__ == "__main__":
    print('Initialized an object for the robot')
    robot = Rl_DP_5()
    print('Initialized an object for Moveit interface')
    #move_it_interface = MoveItInterface(robot)

    rospy.init_node('ros_action_home_all')
    print('Initialized an object for ROS Interface further implementing ROS Actions')
    rldp5_ros_interface = RL_DP_5_ROS(robot, rospy.get_name())

    try:
        while not rospy.is_shutdown():
            #move_it_interface.listener()
            rospy.sleep(0.01)

    except rospy.ROSInterruptException:
        pass
