#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
import dryve_D1 as dryve
import numpy as np
import threading
import actionlib
import sys
from rldp5_msgs.msg import home_allAction, home_allFeedback, home_allResult
from rldp5_msgs.msg import homeAction, homeFeedback, homeResult
from rldp5_msgs.msg import set_des_posAction, set_des_posFeedback, set_des_posResult

speed = 5
accel = 100
homespeed = 5
homeaccel = 100

class Rl_DP_5:
    
    def _init_(self):
        # Initialize your 5 D1 axes here
        Aaxis = dryve.D1("169.254.0.1", 502, 'Axis 1', -140, -140, 140)
        Baxis = dryve.D1("169.254.0.2", 502, 'Axis 2', -100, -100, 50)
        Caxis = dryve.D1("169.254.0.3", 502, 'Axis 3', -115, -115, 115)
        Daxis = dryve.D1("169.254.0.4", 502, 'Axis 4', -100, -100, 100)
        Eaxis = dryve.D1("169.254.0.5", 502, 'Axis 5', -180, -179, 179)

        self.axis_controller = [Aaxis, Baxis, Caxis, Daxis, Eaxis]
        print('Created dryve interfaces')
        self.isHomed = False

    def set_target_position(self, axis, desired_absolute_position):
        if self.isHomed == True:
            if 0 <= axis < len(self.axis_controller):
                if self.axis_controller[axis].min_pos < desired_absolute_position < self.axis_controller[axis].max_pos:
                    self.axis_controller[axis].profile_pos_mode(desired_absolute_position, speed, accel)
                else:
                    print('Axis limit error')
            else:
                print('Axis ID larger than permitted')    
        else:
            print('Robot is NOT HOMED')

    def home(self, axis):
        print(f"Started homing Axis {axis + 1}")
        self.axis_controller[axis].homing(homespeed, homeaccel)

    def home_all(self):
        for axis in self.axis_controller:
            print(f"Started homing {axis.Axis}")
            axis.homing(homespeed, homeaccel)

    def get_current_position(self, axis):
        return self.axis_controller[axis].getPosition()
    
    def setMode(self, axis, mode):
        return self.axis_controller[axis].set_mode(mode)
    
    def setSwon(self, axis):
        return self.axis_controller[axis].set_swon()
    
    def setOpen(self, axis):
        return self.axis_controller[axis].set_op_en()
    
    def setShutDn(self, axis):
        return self.axis_controller[axis].set_shutdn()

#-----------------------------------------------------------------------------------------------------------------------------------
# following are the functions that we want to expose through ROS
# workspace will be /rl_dp_5
# 1. publisher: /status for a joint such as {mode of operation, current position, is_initialized }, 
#    this will require calling multiple functions from dryve_D1.py
# 2. service: /setMode : integer as an input passed on to function set_mode from dryve_D1.py -> check the arguments
# 3. service: /home : this will call homing from dryve_D1.py -> check the arguments
# 4. subscriber: /cmd/set_joint_position : this will set desired joint position by calling profile_pos_mode -> check arguments
# 5. 
#
# start ROS Node code here
# create all publishers, subscribers and action commands of ROS based interface
# Action commands: 1: home <int>, home_all, setmode <int>, set_swon, set_open, set_shutdn [done]
# publishers: status <can include a lot of integers we will discuss later>
# subscribers: 
#-----------------------------------------------------------------------------------------------------------------------------------

class RL_DP_5_ROS:

    def _init_(self, robot):
        self.robot = robot
        self._action_name = rospy.get_name()
        rospy.loginfo("Multi Action server starting...")
        
        # self._as = actionlib.SimpleActionServer('RLDP5_Robot_Action', rldp5_robotAction, execute_cb=self.execute_cb, auto_start=False)
        # Home All Action Server
        self._as_home_all = actionlib.SimpleActionServer('home_all_action', home_allAction, execute_cb=self.home_all_execute_cb, auto_start=False)

        # Home Joint Action Server
        self._as_home_joint = actionlib.SimpleActionServer('home_joint_action', homeAction, execute_cb=self.home_execute_cb, auto_start=False)

        # Joint States Action Server
        self._as_joint_pos = actionlib.SimpleActionServer('joint_positions_action', set_des_posAction, execute_cb=self.joint_state_execute_cb, auto_start=False)

        # Start the action server.
        self._as_home_all.start()
        self._as_home_joint.start()
        self._as_joint_pos.start()

        rospy.loginfo("Multiple Action servers started...")

    def home_all_execute_cb(self, goal):
        robot.isHomed = False
        self.feedback_home_all = home_allFeedback()
        self.result_home_all = home_allResult()

        rospy.loginfo("Home all execute_cb starting...")

        self.goal = goal
        success = True
        rospy.loginfo("execute_cb starting...")
        print(self.result_home_all)

        if self._as_home_all.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as_home_all.set_preempted()
            success = False

        self.robot.home_all()
        self.send_feedback(self._as_home_all, self.feedback_home_all, self.result_home_all)
        self.check_result(self._as_home_all, self.result_home_all, success)
        robot.isHomed = True

    def home_execute_cb(self, goal):
        self.feedback_home = homeFeedback()
        self.result_home = homeResult()

        rospy.loginfo("Home Joint action execute_cb starting...")
        self.goal = goal.joint_index
        print('self.goal:', self.goal)
        success = True
        rospy.loginfo("Home Joint action execute_cb starting...")

        if self._as_home_joint.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as_home_all.set_preempted()
            success = False

        if 0 < self.goal <= 5:
            self.robot.home(self.goal - 1)
        else:
            rospy.loginfo("Provide proper joint index")

        self.send_feedback(self._as_home_joint, self.feedback_home, self.result_home)
        self.check_result(self._as_home_joint, self.result_home, success)

    def joint_state_execute_cb(self, goal):
        self.feedback_joint_pos = set_des_posFeedback()
        self.result_joint_pos = set_des_posResult()

        rospy.loginfo("Joint States action execute_cb starting...")
        self.goal = goal.joint_states
        success = True
        rospy.loginfo("Joint States action execute_cb starting...")

        if self._as_joint_pos.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as_home_all.set_preempted()
            success = False

        # Should check with client
        for i in range(5):
            self.robot.set_target_position(i, self.goal[i])

        self.send_feedback(self._as_joint_pos, self.feedback_joint_pos, self.result_joint_pos)
        self.check_result(self._as_joint_pos, self.result_joint_pos, success)

    def send_feedback(self, actionServer, fb, res):       
        self.positions = []
        self.fb = fb
        self.res = res
        self.actionServer = actionServer

        self.pos = [1, 2, 3, 4, 5]
        for i in range(5):
            self.positions.append(self.robot.get_current_position(i))

        print("Positions: ", self.positions)
        self.fb.status = self.positions 
        print("Feed Back: ", self.fb.status)
        self.actionServer.publish_feedback(self.fb)

    def check_result(self, actionServer, res, suc):
        self.res = res
        self.actionServer = actionServer
        self.suc = suc

        if self.suc:
            self.res.result_message = "%s goal succeeded" % self._action_name
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self.actionServer.set_succeeded(self.res)
            rospy.loginfo("Published %s goal" % self._action_name)
        else:
            rospy.loginfo("%s: Aborted - Goal is not in an active state" % self._action_name)
     
class MoveItInterface:

    def _init_(self, robot):
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
                self.t.join()
    
if __name__ == "__main__":
    try:
        # Initialize the robot
        print('Initializing an object for the robot...')
        robot = Rl_DP_5()

        # Uncomment the following lines if MoveItInterface is to be used instead of Action commands
        # print('Initializing an object for MoveIt interface...')
        move_it_interface = MoveItInterface(robot)
        move_it_interface.listener()
        # Initialize the ROS interface for implementing ROS Actions
        #rospy.init_node('RLDP5_Robot_Action', anonymous=True)
        print('Initializing an object for ROS Interface further implementing ROS Actions...')
        rldp5_ros_interface = RL_DP_5_ROS(robot)

        # Main loop
        while not rospy.is_shutdown():
            # Uncomment the following line if MoveItInterface is to be used instead of Action commands
            #move_it_interface.listener()

            # Sleep to control the loop frequency
            rospy.sleep(0.01)

    except rospy.ROSInterruptException:
        pass
