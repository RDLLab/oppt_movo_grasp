#!/usr/bin/env python
import math
import numpy as np
import yaml
from threading import Lock
import rospy
import rosbag
import rospkg
import actionlib
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState


from movo_action_clients.jaco_action_client import JacoActionClient
from movo_action_clients.torso_action_client import TorsoActionClient
from movo_action_clients.gripper_action_client import GripperActionClient
from movo_action_clients.head_action_client import HeadActionClient
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from moveit_msgs.msg import MoveItErrorCodes
from trac_ik_python.trac_ik import IK


from std_srvs.srv import *


class MovoControllerInterface(object):
	def __init__(self, Dof='7dof', sim = False):
		self.lock = Lock()
		self.dof = Dof
		self.tfBuffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tfBuffer)
		self.moveGroup = MoveGroupInterface("right_side", "base_link")
		self.moveGroup.setPlannerId("RRTConnectkConfigDefault")


		# Dictionary with Movo body groups as keys and their corresponding action controller as values
		rospy.sleep(0.2)
		self.MovoBodyControllers = {
			#Right Arm
			"RightArm" : JacoActionClient('right', self.dof),

			#Left Arm
			"LeftArm" : JacoActionClient('left', self.dof),

			#Torso
			"Torso" : TorsoActionClient(),

			#Head
			"Head" : HeadActionClient(),

			#Right Gripper
			"RightGripper" : GripperActionClient('right'),

			#Left Gripper
			"LefGripper" : GripperActionClient('left')
		}



		# Open yaml file with movo configuration
		rospack = rospkg.RosPack()
		info_file_path = rospack.get_path('popcorn_manager') + '/config/movo_controller_interface.yaml'
		info_file = open(info_file_path, 'r')
		self.movo_info = yaml.load(info_file)
		self.MovoJointNames = self.movo_info["Joint Names"]
		self.MovoJointValues = self.movo_info["Initial Joint Values"]
		self.jpos_limits = self.movo_info["Joint Limits"]


		# Subscriber to joint states
		self.jointStates = rospy.Subscriber('/joint_states', JointState, self.updateJointValuesCB)
		# Wait to ensure joint values are updated from callback
		rospy.sleep(0.2)


	def getJointAngles(self, body_group):
		#Check if body group is within movo names
		if(not body_group in self.MovoJointNames):
			rospy.fatal("body group not in Movo names")
			return

		# Get values using lock
		try:
			self.lock.acquire()
			JointValues = self.MovoJointValues[body_group]
		finally:
			self.lock.release()

		return JointValues


	def getLinkPose(self, baseLink, targetLink):
		# Query for pose information of targetLink relative to baseLink
		transInfo = None
		rospy.loginfo("Waiting for TF....")
		# Loop until pose is returned
		while(not rospy.is_shutdown()):
			try:
				transInfo = self.tfBuffer.lookup_transform(baseLink, targetLink, rospy.Time(0))
				break
			except Exception as e:
				print(e)
				rospy.sleep(0.1)
				continue

		return transInfo




	def getBodyNames(self):
		return self.MovoJointNames.keys()


	def getBodyJointNames(self, body_group):
		#Check if body group is within movo names
		if(not body_group in self.MovoJointNames):
			rospy.fatal("body group not in Movo names")

		return self.MovoJointNames[body_group]


	def updateJointValuesCB(self, data):
		""" Callback function to update the joint angle positions"""
		try:
			# Get lock
			data_joints = data.name
			self.lock.acquire()


			# Update values
			for body_group in self.MovoJointNames:
				for body_joint_index in range(len(self.MovoJointNames[body_group])):
					data_index = data_joints.index(self.MovoJointNames[body_group][body_joint_index])
					self.MovoJointValues[body_group][body_joint_index] = data.position[data_index]
		

		finally:
			self.lock.release()


	def getBodyController(self, body_group):
		#Check if body group is within movo names
		if(not body_group in self.MovoBodyControllers):
			rospy.fatal("body group not in Movo controllers")
			return

		return self.MovoBodyControllers[body_group]




	"""----------------------------------------- Methods to move the robot ------------------------"""
	def moveUpperBody(self, right_arm_target, torso_target, time=5.0):
		# Get action clients needed
		right_arm_controller = self.getBodyController("RightArm")
		torso_controller = self.getBodyController("Torso")
		# Execute trajectory for arm
		self.execTrajectory(self.getJointAngles("RightArm"), right_arm_target, "RightArm", 10, timeout=time)
		# Execute trajectory for the torso
		self.execTrajectory(self.getJointAngles("Torso"), torso_target, "Torso", 10, timeout=time)
		# Wait for execution to finish
		right_arm_controller.wait(time)
		torso_controller.wait(time)
		right_arm_controller.clear()
		torso_controller.clear()
		rospy.sleep(1.0)



	def tableHeightToLinearJoint(self, table_height = 0.74):
		""" Function to map the environment table height to a suitable
		linear joint value, so the robot can perform the demo """
		
		""" Check for heights allowed """
		default_joint_val = 0.11
		default_height = 0.74

		# Height of robot roughly increases 1cm by every 0.01 increase in linear joint
		diff_height = table_height - default_height

		return default_joint_val + diff_height





	def bringToScanningPose(self, table_height=0.74, time=5.0):
		""" Function to set the robot into a pre scanning pose """
		rospy.loginfo("Moving to Scanning pose")

		# Set target joint values
		scanning_pose_arm = [0, -1.57, 0, 0, 0.0, 0, -1.57]
		scanning_pose_torso = [self.tableHeightToLinearJoint(table_height)]

		# Move upper body to target values
		self.moveUpperBody(scanning_pose_arm, scanning_pose_torso)

		# Open gripper
		self.getBodyController("RightGripper").command(0.165)
		rospy.loginfo("Moved to scanning pose Completed")
		rospy.sleep(2.0)



	def bringToPreGraspPose(self, table_height=0.74, time=5.0):
		""" Function to set the robot into a pre-grasp pose """
		rospy.loginfo("Bringing to pre grasp pose")

		# Set the target values
		pre_grasp_pose_arm = [0.0, -1.80, 0, -1.0, 0.0, 1.0, -1.57]
		pre_grasp_pose_torso = [self.tableHeightToLinearJoint(table_height)]

		# Move upper body to target values
		self.moveUpperBody(pre_grasp_pose_arm, pre_grasp_pose_torso)



	def execTrajectory(self, start_point, end_point, body_group, num_traj_points=5, timeout=5.0):
		""" Function to move the specified body_group from the current state to the target vals.
		The indices for end points must match the order of the joint names of the body group according to
		self.MovoJointNames """

		#Check if body group exists
		if(not body_group in self.MovoJointNames):
			rospy.logfatal("Body group does not exists. Please refer to MovoControllerInterface.py")
			return

		# Get the body group action client, values, and joint names
		body_controller = self.getBodyController(body_group)
		body_joint_vals = self.getJointAngles(body_group)
		body_joint_names = self.getBodyJointNames(body_group)

		# Check for joint limits and cap them if necessary
		for joint_index, joint_name in enumerate(body_joint_names):
			# Check that joint name is in limits database
			if(not joint_name in self.jpos_limits):
				break
			else:
				min_limit, max_limit = self.jpos_limits[joint_name]
				# Check for max and min
				if(end_point[joint_index] > max_limit):
					end_point[joint_index] = max_limit
				elif(end_point[joint_index] < min_limit):
					end_point[joint_index] = min_limit


		# Partition start to timeout in num_points slices
		times_from_start = np.linspace(0.0, timeout, num_traj_points)
		trajectory_points = []
		for i in xrange(len(start_point)):
			tp = np.linspace(start_point[i], end_point[i], num_traj_points)
			trajectory_points.append(tp)
		trajectory_points = np.array(trajectory_points)
		for i in xrange(0, len(trajectory_points.T)):
			trajectory_point = trajectory_points.T[i]
			body_controller.add_point(trajectory_point, times_from_start[i])

		# Start and wait until done
		body_controller.start()
		body_controller.wait(50)
		exe_result = body_controller.result()
		# rospy.loginfo("Finished execution of %s" % (body_group))

		# Clear controller
		body_controller.clear()
