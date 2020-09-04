#!/usr/bin/env python
"""
Watche gripper feedback topics to esimate if an object is currently being
grasped
"""

import roslib
import rospy
import math
import rosnode

import datetime as dt

from copy import copy

from std_msgs.msg import Bool, Int16
from sensor_msgs.msg import JointState


class PopcornGripperEstimator():


    def __init__(self):
        """
        Constructor
        """

        # Construct node
        rospy.init_node("grasp_estimator", anonymous=True)

        # Check if we're in simulation or not
        self.sim = rospy.get_param("~sim", False)
        if (self.sim):
            rospy.loginfo("Is in simulation")
            rospy.wait_for_message('/sim_initialized', Bool)

        # Get an optional cylinder diamter parameter - grasping estimates are
        # based on assuming we grasp an object of this 'diameter'
        # Radius varies from 0 (~9cm object) to 1 (~4cm object)
        self.grasp_posture = rospy.get_param("~grasp_posture", 0.58)

        # Subscribe to gripper topics
        input_topic = "/movo/{}_gripper/joint_states"
        '''self.left_gripper_sub = rospy.Subscriber(
            input_topic.format("left"),
            JointState,
            self.left_gripper_callback
        )'''
        self.right_gripper_sub = rospy.Subscriber(
            input_topic.format("right"),
            JointState,
            self.right_gripper_callback
        )

        # Prepare output topics
        output_topic = "/popcorn/{}_gripper/is_gripping"
        self.left_gripper_pub = rospy.Publisher(
            output_topic.format("left"),
            Int16,
            queue_size=1,
            latch=True
        )
        self.right_gripper_pub = rospy.Publisher(
            output_topic.format("right"),
            Int16,
            queue_size=1,
            latch=True
        )

        # State variables
        self.left_is_gripping = False
        self.right_is_gripping = False
        rospy.set_param('~left_is_grasping',self.left_is_gripping)
        rospy.set_param('~right_is_grasping',self.right_is_gripping)
        self.close_threshold = rospy.get_param("~close_threshold")

        # Publish output topics until we exit
        self.hz = 100
        self.rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            self.left_gripper_pub.publish(Int16(int(self.left_is_gripping)))
            self.right_gripper_pub.publish(Int16(int(self.right_is_gripping)))
            self.rate.sleep()


    def process(self, pos, vel, eff, prev_val=False):
        """
        Attempts to classify grasping/not grasping based on the given
        variables
        """

        # Threshold - positions less than this this indicate the hand is fully open
        open_threshold = (self.grasp_posture + 0.0) / 2.0

        successful_grasp = False
        if pos < open_threshold:
            # Hand is fully open
            successful_grasp = False
        elif pos < self.close_threshold:
            # Didn't quite reach fully closed position - assume we've grasped successfully
            successful_grasp = True
        else:
            # Hand is fully closed - no grasp
            successful_grasp = False

        return successful_grasp

    def mean(self, number_list):
        """
        Helper function - computes mean of a list of flaots
        """
        return float(sum(number_list)) / max(len(number_list), 1)

    def left_gripper_callback(self, msg):
        """
        Callback when feedback is revieved for the left gripper
        """
        self.left_is_gripping = self.process(
            self.mean(msg.position[0:2]),
            self.mean(msg.velocity[0:2]),
            self.mean(msg.effort[0:2]),
            self.left_is_gripping
        )
        rospy.set_param('~left_is_grasping',self.left_is_gripping)


    def right_gripper_callback(self, msg):
        """
        Callback when feedback is revieved for the right gripper
        """
        self.right_is_gripping = self.process(
            self.mean(msg.position[0:1]),
            self.mean(msg.velocity[0:1]),
            self.mean(msg.effort[0:1]),
            self.right_is_gripping
        )
        rospy.set_param('~right_is_grasping',self.right_is_gripping)


# Main function
if __name__ == '__main__':

    pge = PopcornGripperEstimator()

