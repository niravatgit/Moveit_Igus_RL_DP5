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
    """
    Class representing the RLDP5 ROS Action Server.

    This class handles the execution of ROS actions for the RLDP5 robot.

    Attributes:
    - _feedback (rldp5_robotFeedback): Feedback object for the action.
    - _result (rldp5_robotResult): Result object for the action.
    - robot: Instance of the RLDP5 robot.
    - _action_name (str): Name of the ROS action server.
    - _as (SimpleActionServer): SimpleActionServer for handling ROS actions.
    - goal (rldp5_robotGoal): Goal received from the action client.
    """

    _feedback = rldp5_robotFeedback()
    _result = rldp5_robotResult()

    def __init__(self, robot):
        """
        Initializes the RLDP5ROS instance.

        Args:
        - robot: Instance of the RLDP5 robot.
        - name (str): Name of the ROS action server.
        """
        self.robot = robot
        self._action_name = rospy.get_name()
        rospy.loginfo("Action server starting...")
        
        # self._as = actionlib.SimpleActionServer(self._action_name, rldp5_robotAction, execute_cb=self.execute_cb, auto_start=False)
        self._as = actionlib.SimpleActionServer('RLDP5_Robot_Action', rldp5_robotAction, execute_cb=self.execute_cb, auto_start=False)
        # Start the action server.
        self._as.start()
        rospy.loginfo("Action server started...")

    def execute_cb(self, goal):
        """
        Callback function for executing the ROS action.

        Args:
        - goal (rldp5_robotGoal): Goal received from the action client.
        """
        rospy.loginfo("execute_cb starting...")
        self.goal = goal
        success = True
        rospy.loginfo("execute_cb starting...")

        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False
            
        if self.goal.command == 'home_all':
            self.robot.home_all()
            self.send_feedback()
                
        elif self.goal.command.startswith('joint_') and self.goal.command[6:].isdigit():
            joint_number = int(self.goal.command[6:])           
            self.robot.home(joint_number-1)
            self.send_feedback()
                
        # elif self.goal.command == 'set_shutdn':
        #     dryve.set_shutdn()
        #     self.send_feedback()

        # elif self.goal.command == 'set_swon':
        #     dryve.set_swon()
        #     self.send_feedback()

        # elif self.goal.command == 'set_op_en':
        #     dryve.set_op_en()
        #     self.send_feedback()
                
        else:
            # Handle invalid commands here if needed            
            print("Provide valid goal command from Client side")
            success = False
            
        if success:
           self._result.success = self._feedback.status
           rospy.loginfo('%s: Succeeded' % self._action_name)
           self._as.set_succeeded(self._result.success)
           rospy.loginfo("published goal...")
           
        else:
           rospy.loginfo("%s: Aborted - Goal is not in an active state" %self._action_name)
            
    def send_feedback(self):
        self.positions = []
        for i in range(5):
            self.positions.append(self.robot.get_current_position(i))
            
        self._feedback.status = self.positions 
        rospy.loginfo("publishing feedback for axis:")
        self._as.publish_feedback(self._feedback)            
        return self._feedback       
      
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
    try:
        # Initialize the robot
        print('Initializing an object for the robot...')
        robot = Rl_DP_5()

        # Uncomment the following lines if MoveItInterface is to be used instead of Action commands
        # print('Initializing an object for MoveIt interface...')
        # move_it_interface = MoveItInterface(robot)

        # Initialize the ROS interface for implementing ROS Actions
        rospy.init_node('RLDP5_Robot_Action', anonymous=True)
        print('Initializing an object for ROS Interface further implementing ROS Actions...')
        rldp5_ros_interface = RL_DP_5_ROS(robot)

        # Main loop
        while not rospy.is_shutdown():
            # Uncomment the following line if MoveItInterface is to be used instead of Action commands
            # move_it_interface.listener()

            # Sleep to control the loop frequency
            rospy.sleep(0.01)

    except rospy.ROSInterruptException:
        pass
        
'''
[ERROR] [1701239398.795959]: Exception in your execute callback: 'list' object has no attribute 'success'
Traceback (most recent call last):
  File "/opt/ros/noetic/lib/python3/dist-packages/actionlib/simple_action_server.py", line 289, in executeLoop
    self.execute_callback(goal)
  File "rl_dp_5_robot_con_ROS.py", line 147, in execute_cb
    self._as.set_succeeded(self._result.success)
  File "/opt/ros/noetic/lib/python3/dist-packages/actionlib/simple_action_server.py", line 162, in set_succeeded
    self.current_goal.set_succeeded(result, text)
  File "/opt/ros/noetic/lib/python3/dist-packages/actionlib/server_goal_handle.py", line 195, in set_succeeded
    self.action_server.publish_result(self.status_tracker.status, result)
  File "/opt/ros/noetic/lib/python3/dist-packages/actionlib/action_server.py", line 182, in publish_result
    self.result_pub.publish(ar)
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py", line 882, in publish
    self.impl.publish(data)
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py", line 1066, in publish
    serialize_message(b, self.seq, message)
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/msg.py", line 152, in serialize_message
    msg.serialize(b)
  File "/home/inspire_igus/catkin_ws/devel/lib/python3/dist-packages/rldp5_msgs/msg/_rldp5_robotActionResult.py", line 153, in serialize
    buff.write(_get_struct_5d().pack(*self.result.success))
AttributeError: 'list' object has no attribute 'success'

[ERROR] [1701239398.797781]: To transition to an aborted state, the goal must be in a preempting or active state, it is currently in state: 3

'''
