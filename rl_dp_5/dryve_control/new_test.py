#!/usr/bin/env python
import rospy
from std_msgs.msg import String 

import sys

# ROS node
rospy.init_node('command_node')

pub_mode = rospy.Publisher('/robot/mode', String, queue_size=10)
pub_cmd = rospy.Publisher('/robot/cmd', String, queue_size=10)  

def print_usage():
    print("Greetings! Here is how to use this program:")
    print("  python robot_control.py command arguments")
    print("The available commands are:")
    print("  home - Requests homing of the robot") 
    print("  mode OPERATION_MODE - Respectfully asks to set operation mode")
    print("  cmd JOINT_POSITION - Politely commands joint position")

if __name__ == "__main__":

    if len(sys.argv) < 2:
        print_usage()
        print("Please provide a valid command.")
        sys.exit(1)

    command = sys.argv[1]

    if command == "home":
        print("Requesting the robot to home itself.")
        pub_cmd.publish("home")

    elif command == "mode":
        if len(sys.argv) < 3:
            print("My apologies, the operation mode was not specified.")
            print_usage()
            sys.exit(1)

        mode = sys.argv[2]
        print("Asking to set mode to", mode)
        pub_mode.publish(mode)

    elif command == "cmd":
        if len(sys.argv) < 3:
            print("Sorry, the joint position was not specified.")
            print_usage()
            sys.exit(1)
            
        position = float(sys.argv[2])
        print("Politely commanding joint to", position)
        pub_cmd.publish(str(position))

    else:
        print("I do not understand this command. Please use one of the valid commands.")
        print_usage()
        sys.exit(1)


------------------------------------------------------------------------------- 15-11-2023 ----------------------------------------------------------------------------------------------------
Subscriber code:

#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

rospy.init_node('joint_state_subscriber')

last_msg = None

def test_msg(msg):
	global last_msg
	last_msg = msg

def print_msg(msg):
  if(msg is not None):
    print('The Joint States of the robot is:', msg)

sub = rospy.Subscriber('/joint_states', JointState, test_msg, queue_size=1)

while not rospy.is_shutdown():
	print_msg(last_msg)











---------------------------------------------
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header
import dryve_D1 as dryve
import numpy as np

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
            self.axis_controller[axis].profile_pos_mode(desired_absolute_position, 5, 50)

    def home(self, axis):
        print(f"Started homing Axis {axis + 1}")
        self.axis_controller[axis].homing(homespeed, homeaccel)

    def home_all(self):
        for axis in self.axis_controller:
            print(f"Started homing {axis.Axis}")
            axis.homing(homespeed, homeaccel)

    def get_current_position(self, axis):
        return self.axis_controller[axis].getPosition()

class MoveItInterface:
    def __init__(self, robot):
        self.robot = robot
        rospy.init_node('moveit_interface_node', anonymous=True)

        # Subscriber to get joint states during trajectory planning from MoveIt
        rospy.Subscriber('/move_group/joint_states', JointState, self.moveit_joint_states_callback)

        # Publisher to simulate the robot pose in MoveIt based on the current position of the real robot
        self.fake_controller_joint_states_pub = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=10)

        self.position_history = []

    def publish_current_positions(self):
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
        joint_state.position = [np.deg2rad(self.robot.get_current_position(i)) for i in range(5)]

        # Publish joint state to simulate the robot pose in MoveIt
        self.fake_controller_joint_states_pub.publish(joint_state)

    def moveit_joint_states_callback(self, data):
        # Callback function for joint states from MoveIt
        joint_trajectory = JointTrajectory()
        joint_trajectory.header = Header(stamp=rospy.Time.now())
        joint_trajectory.joint_names = data.name
        joint_trajectory.points.append(data)

        # Process joint trajectory as needed
        # You can use joint_trajectory.points for planned joint positions

    def joint_states_callback(self, data):
        joint_state = JointState()
        joint_state.header = data.header
        joint_state.name = data.name
        joint_state.position = data.position

        # Publish joint state
        self.fake_controller_joint_states_pub.publish(joint_state)

        # Check for repeated values
        if self.check_repeated_values(joint_state.position, 20):
            rospy.loginfo("Trajectory planned after 20 repeated positions.")
            rospy.signal_shutdown("Trajectory planned.")

    def check_repeated_values(self, current_values, threshold):
        self.position_history.append(current_values)
        if len(self.position_history) >= threshold:
            recent_positions = self.position_history[-threshold:]
            return all(positions == current_values for positions in recent_positions)
        return False

if __name__ == "__main__":
    robot = Rl_DP_5()
    moveit_interface = MoveItInterface(robot)

    try:
        while not rospy.is_shutdown():
            moveit_interface.publish_current_positions()
            rospy.sleep(1)
    except rospy.ROSInterruptException:
        pass

------------------------------------------23-11-2023: Multithreading and unclean code-----------------------------------------------
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import ExecuteTrajectoryActionResult
import dryve_D1 as dryve
import numpy as np
import threading

speed = 5
accel = 100
homespeed = 5
homeaccel = 100


#

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

class RL_DP_5_ROS:
    def __init__(self):
        print('Hello ROS')
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

class MoveItInterface:
#here we should have two things
#1. Subsriber to joint_states from moveit to getjoint space trajectory to plannned pose
#2. Published to fake_joint_controller to simulate the robot pose in Moveit based on current postiison of the real robot

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
        # Subscriber to get joint states during trajectory planning from MoveIt
        #print('Subscribing to the move_group joint states')
        rospy.Subscriber('/joint_states', JointState, self.callback_fn, queue_size=1)
        

        # Subscriber to monitor the execution result of planned trajectories
        #rospy.Subscriber('/execute_trajectory/result', ExecuteTrajectoryActionResult, self.execution_result_callback)

    def callback_fn(self, data):
        self.joint_state_position = list(data.position)
        print(self.joint_state_position)

        with self.thread_lock:
            for i in range(5):
                thread = threading.Thread(target=self.robot.set_target_position, args=(i, np.rad2deg(self.joint_state_position[i])))
                thread.start()

        for i in range(5):
            thread = threading.Thread(target=self.robot.set_target_position, args=(i, np.rad2deg(self.joint_state_position[i])))
            threads.append(thread)

            for thread in threads:
                thread.start()

            for thread in thread:
                thread.join()
        # for i in range(5):
        #     print("setting robot axis_", i,"as :", self.joint_state_position[i])
        #     self.robot.set_target_position(i, np.rad2deg(self.joint_state_position[i])) 
        	
        #self.position_history.append(joint_state.position)
        #print("Trajectory Points:", joint_state.position)
        # self.send_position_to_robot(self.joint_state.position)

#    def publish_current_positions(self):
 #       print('Publishing the positional data from the robot')

#        print("got the position", self.joint_state)
        
#        print("pub the position")
        
#        self.listener()


    def send_position_to_robot(self, current_joint_position):
        for axis, position in enumerate(current_joint_position):
            self.robot.set_target_position(axis, position)
            rospy.sleep(1)
            if self.check_repeated_values(current_joint_position, 5):
                rospy.loginfo("Robot is stationary.")
                rospy.signal_shutdown("IGUS is immobile.")
            continue
        rospy.sleep(1)	
        

    def check_repeated_values(self, current_values, threshold):
        self.position_history.append(current_values)
        #for i in position_history:
            #self.send_position_to_robot(i)
        if len(self.position_history) >= threshold:
            recent_positions = self.position_history[-threshold:]
            return all(positions == current_values for positions in recent_positions)
        return False

    def execution_result_callback(self, data):
        self.execution_result = data

    # def is_trajectory_started(self):
    #     return self.execution_result is not None and self.execution_result.status.status == 1  # Check if status is ACTIVE

    # def is_trajectory_finished(self):
    #     return self.execution_result is not None and self.execution_result.status.status == 3  # Check if status is SUCCEEDED

if __name__ == "__main__":
    print('Initialized an object for the robot')
    robot = Rl_DP_5()
    print('Initialized an object for Moveit interface')
    move_it_interface = MoveItInterface(robot)

    try:
        while not rospy.is_shutdown():

            move_it_interface.listener()

            # if move_it_interface.is_trajectory_started():
            #     print("Trajectory is started!")

            # if move_it_interface.is_trajectory_finished():
            #     print("Trajectory is finished!")

            rospy.sleep(0.01)
    except rospy.ROSInterruptException:
        pass

-------------------------------------------------- 24-11-2023 ROS Actions ---------------------------------------
import actionlib
import rldp5_action.action 

class RL_DP_5_ROS:

    def __init__(self, robot):
      
        self.robot = robot
            
        # Define the action server
        self.action_server = actionlib.SimpleActionServer('rldp5_action', 
            rldp5_action.action.rldp5_robotAction, 
            execute_cb=self.action_execute_callback, 
            auto_start=False)
        self.action_server.start()
        
        # Define the action client
        self.action_client = actionlib.SimpleActionClient('rldp5_action', 
            rldp5_action.action.rldp5_robotAction)
        self.action_client.wait_for_server() 
        
        
    def action_execute_callback(self, goal):
        # Execute callbacks when goals are received
        
        if goal.command == "home_all":
            self.robot.home_all()
            self.action_server.set_succeeded()
            
        elif goal.command == "move_joint":
            # Move joint
            self.action_server.set_succeeded()
            
        # Other execute functions
        
        
    def send_action_goal(self, command):
        
        # Create goal
        goal = rldp5_action.action.rldp5_robotGoal()
        goal.command = command 
        
        # Send goal
        self.action_client.send_goal(goal)
        
        # Wait for result
        self.action_client.wait_for_result()
        
        return self.action_client.get_result()
    
    -------------------------------------------------------------------