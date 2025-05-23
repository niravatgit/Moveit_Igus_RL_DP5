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

    def __init__(self):
        Aaxis = dryve.D1("169.254.0.1", 502, 'Axis 1', -140, -140, 140)
        Baxis = dryve.D1("169.254.0.2", 502, 'Axis 2', -100, -100, 50)
        Caxis = dryve.D1("169.254.0.3", 502, 'Axis 3', -115, -115, 115)
        Daxis = dryve.D1("169.254.0.4", 502, 'Axis 4', -100, -100, 100)
        Eaxis = dryve.D1("169.254.0.5", 502, 'Axis 5', -180, -179, 179)

        self.axis_controller = [Aaxis, Baxis, Caxis, Daxis, Eaxis]
        print('Created dryve interfaces')
        self.isHomed = False
        self.lock = threading.Lock()
        
        for axis in self.axis_controller:
            axis.target_position = 0
            axis.travel_distance = 0

    def set_target_position(self, axis, desired_absolute_position):
        if not self.isHomed:
            print('Robot is NOT HOMED')
            return

        if not (0 <= axis < len(self.axis_controller)):
            print('Axis ID larger than permitted')
            return

        if not (self.axis_controller[axis].min_pos < desired_absolute_position < self.axis_controller[axis].max_pos):
            print('Axis limit error')
            return
            
        current_pos = self.axis_controller[axis].getPosition()
        
        travel_distance = abs(desired_absolute_position - current_pos)
        
        self.axis_controller[axis].target_position = desired_absolute_position
        self.axis_controller[axis].travel_distance = travel_distance


    def home(self, axis):
        print(f"Started homing Axis {axis + 1}")
        self.axis_controller[axis].homing(homespeed, homeaccel)

    def home_all(self):
        threads = []
        for axis in range(len(self.axis_controller)):
            t = threading.Thread(
                target=self.axis_controller[axis].homing,
                args=(homespeed, homeaccel)
            )
            threads.append(t)
            t.start()

        for t in threads:
            t.join()

        self.isHomed = True

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

# -----------------------------------------------------------------------------------------------------------------------------------

class RL_DP_5_ROS:

    def __init__(self, robot):
        self.robot = robot
        self._action_name = rospy.get_name()
        rospy.loginfo("Multi Action server starting...")

        self._as_home_all = actionlib.SimpleActionServer('home_all_action', home_allAction, execute_cb=self.home_all_execute_cb, auto_start=False)
        self._as_home_joint = actionlib.SimpleActionServer('home_joint_action', homeAction, execute_cb=self.home_execute_cb, auto_start=False)
        self._as_joint_pos = actionlib.SimpleActionServer('joint_positions_action', set_des_posAction, execute_cb=self.joint_state_execute_cb, auto_start=False)

        self._as_home_all.start()
        self._as_home_joint.start()
        self._as_joint_pos.start()

        rospy.loginfo("Multiple Action servers started...")

    def home_all_execute_cb(self, goal):
        self.robot.isHomed = False
        self.feedback_home_all = home_allFeedback()
        self.result_home_all = home_allResult()

        rospy.loginfo("Home all execute_cb starting...")

        success = True

        if self._as_home_all.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as_home_all.set_preempted()
            success = False

        self.robot.home_all()
        self.send_feedback(self._as_home_all, self.feedback_home_all, self.result_home_all)
        self.check_result(self._as_home_all, self.result_home_all, success)
        self.robot.isHomed = True

    def home_execute_cb(self, goal):
        self.feedback_home = homeFeedback()
        self.result_home = homeResult()

        rospy.loginfo("Home Joint action execute_cb starting...")
        joint_index = goal.joint_index
        success = True

        if self._as_home_joint.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as_home_joint.set_preempted()
            success = False

        if 0 < joint_index <= 5:
            self.robot.home(joint_index - 1)
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

        if self._as_joint_pos.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as_joint_pos.set_preempted()
            success = False
            
        # Calculate current positions and movement distances
        current_positions = [self.robot.get_current_position(i) for i in range(5)]
        distances = [abs(self.goal[i] - current_positions[i]) for i in range(5)]
    
        # Find the maximum distance (longest movement)
        max_distance = max(distances)
        if max_distance == 0:
            max_distance = 1  # Avoid division by zero if no movement needed
    
        # Calculate movement time (based on max distance and base speed)
        movement_time = max_distance / speed
    
        # Calculate required speeds for each joint
        joint_speeds = []
        for dist in distances:
            if dist == 0:
                joint_speeds.append(0)  # No movement needed
            else:
                joint_speeds.append(dist / movement_time)
    
        # Move all joints simultaneously with calculated speeds
        threads = []
        for i in range(5):
            if joint_speeds[i] > 0:  # Only move if needed
                t = threading.Thread(
                    target=self.robot.axis_controller[i].profile_pos_mode,
                    args=(self.goal[i], joint_speeds[i], accel),
                    daemon=True
                )
                threads.append(t)

    
        # Start all movements
        for t in threads:
            t.start()
    
        # Wait for completion
        for t in threads:
            t.join()
        
        self.send_feedback(self._as_joint_pos, self.feedback_joint_pos, self.result_joint_pos)
        self.check_result(self._as_joint_pos, self.result_joint_pos, success)

    def send_feedback(self, actionServer, fb, res):
        positions = []
        for i in range(5):
            positions.append(self.robot.get_current_position(i))
        fb.status = positions
        actionServer.publish_feedback(fb)

    def check_result(self, actionServer, res, suc):
        if suc:
            res.result_message = "%s goal succeeded" % self._action_name
            rospy.loginfo('%s: Succeeded' % self._action_name)
            actionServer.set_succeeded(res)
        else:
            rospy.loginfo("%s: Aborted - Goal is not in an active state" % self._action_name)

class MoveItInterface:

    def __init__(self, robot):
        self.robot = robot
        self.execution_result = None
        rospy.init_node('joint_states_subscriber', anonymous=True)

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
    
        # Calculate current positions and movement distances
        current_positions = [self.robot.get_current_position(i) for i in range(5)]
        target_positions = [np.rad2deg(self.joint_state_position[i]) for i in range(5)]
        distances = [abs(target_positions[i] - current_positions[i]) for i in range(5)]
    
        # Find the maximum distance
        max_distance = max(distances)
        if max_distance == 0:
           max_distance = 1  # Avoid division by zero
    
        # Calculate movement time and individual speeds
        movement_time = max_distance / speed
        joint_speeds = [dist/movement_time if dist > 0 else 0 for dist in distances]
    
        # Move all joints
        threads = []
        for i in range(5):
            if joint_speeds[i] > 0:  # Only move if needed
                t = threading.Thread(
                    target=self.robot.axis_controller[i].profile_pos_mode,
                    args=(target_positions[i], joint_speeds[i], accel),
                    daemon=True
                )
                threads.append(t)
    
        # Start all movements
        for t in threads:
            t.start()
    
        # Wait for completion
        for t in threads:
            t.join()

if __name__ == "__main__":
    try:
        print('Initializing an object for the robot...')
        robot = Rl_DP_5()

        # Uncomment the following lines if MoveItInterface is to be used instead of Action commands
        # print('Initializing an object for MoveIt interface...')
        move_it_interface = MoveItInterface(robot)
        move_it_interface.listener()

        # Initialize the ROS interface for implementing ROS Actions
        # rospy.init_node('RLDP5_Robot_Action', anonymous=True)
        print('Initializing an object for ROS Interface further implementing ROS Actions...')
        rldp5_ros_interface = RL_DP_5_ROS(robot)

        while not rospy.is_shutdown():
            rospy.sleep(0.01)

    except rospy.ROSInterruptException:
        pass
