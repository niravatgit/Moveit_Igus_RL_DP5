#! /usr/bin/env python

# This code has been adapted from the ROS Wiki actionlib tutorials to the context
# of this course.
# (http://wiki.ros.org/hrwros_msgs/Tutorials)

import rospy

import actionlib

from rldp5_msgs import rldp5_robotAction, rldp5_robotGoal, rldp5_robotFeedback, rldp5_robotResult

class rldp5ActionClass(object):
    # create messages that are used to publish feedback/result
    _feedback = rldp5_robotFeedback()
    _result = rldp5_robotResult()

    def __init__(self, robot):
        # This will be the name of the action server which clients can use to connect to.
        self.robot = robot

        # Create a simple action server of the newly defined action type and
        # specify the execute callback where the goal will be processed.
        self._as = actionlib.SimpleActionServer(self._action_name, rldp5_robotAction, execute_cb=self.execute_cb, auto_start = False)

        # Start the action server.
        self._as.start()
        rospy.loginfo("Action server started...")

    def execute_cb(self, goal):

        if goal == 'home_all':
            print("homing all")

        # start executing the action
        for counter_idx in range(0, goal.num_counts):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            # publish the feedback
            self._feedback.counts_elapsed = counter_idx
            self._as.publish_feedback(self._feedback)
            # Wait for counter_delay s before incrementing the counter.
            delay_rate.sleep()

        if success:
            self._result.result_message = "Successfully completed counting."
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    # Initialize a ROS node for this action server.
    rospy.init_node('rldp5_robot_action')

    # Create an instance of the action server here.
    server = rldp5ActionClass(rospy.get_name())
    rospy.spin()