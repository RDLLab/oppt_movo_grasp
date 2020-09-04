#!/usr/bin/env python
import os
import importlib
import rospy
import rosbag
import rospkg
import actionlib
import tf2_ros
import pandas as pd
import yaml
import numpy as np
import matplotlib.pyplot as plt
from ConfigParser import ConfigParser
from mpl_toolkits.mplot3d import Axes3D
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState

from MovoControllerInterface import MovoControllerInterface
from MovoPlannerInterface import MovoPlannerInterface
from MovoVisionInterface import MovoVisionInterface
from popcorn_oppt_ros_interface_msgs.msg import OPPTPlanAction, OPPTPlanGoal
from popcorn_oppt_ros_interface_msgs.msg import OPPTInitBeliefAction, OPPTInitBeliefGoal
from popcorn_oppt_ros_interface_msgs.msg import OPPTObservation
from std_srvs.srv import *


# This file is the top level script that communicates with Robot Planner Script and the Robot Controller Script
# It is responsible for making the function calls to the afore mentioned scripts in order to get the robot into 
# the correct initial position,setup the vision module and OPPT client, recording and sending observations to OPPT 
# and executing the actions received. The actual implementation of each of the aforementioned process is 
# distributed amongst the files MovoControllerInterface.py, MovoPlannerInterface.py, MovoVisionInterface.py.
# This script provides a high level control of these actions so that users may understand the broad steps of the entire implementation

def main():
    print("STARTED interface packing version")
    rospy.init_node("oppt_ros_interface_tester")
    rospy.loginfo('oppt_ros_interface_tester: up and running')
    filename = '/config.ini'
    config = ConfigParser()
    config.read(os.path.join(os.path.dirname(__file__), 'config.ini'))
    visionFile = config.get('Vision','filename')
    visionClass = config.get('Vision','class')
    MovoVisionInterface = getattr(__import__(visionFile, fromlist=[visionClass]), visionClass)

    # Retrieve some environment info from file
    vision_pkg_path = rospkg.RosPack().get_path('popcorn_vision')
    tabletop_file_path = vision_pkg_path + "/config/popcorn_tabletop_objects.yaml"
    tabletop_file = open(tabletop_file_path, "r")
    tabletop_info = yaml.load(tabletop_file)
    table_dims = tabletop_info["popcorn_tabletop_object_supports"]["table"]["dimension"]
    table_dims_height = table_dims["box_z"]
    print("HEIGHT IS %f" %(table_dims_height))

    
    # Initialize the movo control and planning module. The control module is capable of handling two versions of the robot-
    # 6DOF and 7DOF Jaco Arms
    MovoClient = MovoControllerInterface('7dof', True)
    next_iter_param = '/oppt_next_iter'
    rospy.set_param(next_iter_param, False);
    print("Initialized robot control interface")

    MovoVisionClient = MovoVisionInterface()
    MovoClient.bringToScanningPose(table_height = table_dims_height)

    MovoPlanner = MovoPlannerInterface(False)
    print("Initialized planner interface")


    # Initialize the vision client with the model name of the target object as specified in your sdf model
    initial_objs, initial_obstacles = MovoVisionClient.getVision()
    print(initial_objs)

    # Initialize the bring to initial config
    MovoClient.bringToPreGraspPose(table_height = table_dims_height)
    InitBeliefStatus = MovoPlanner.setupInitBeliefClient()


    print("InitBeliefStatus")
    if (InitBeliefStatus is not True):
    	return

    print("Connecting to initial belief")

    InitBeliefFinishedStatus = MovoPlanner.sendInitBelief(MovoClient, initial_objs, initial_obstacles)


    print("Init Belief Finished Status : ", InitBeliefFinishedStatus)

    if (InitBeliefFinishedStatus is not True):
    	return

    PlanServerStatus = MovoPlanner.setupOPPTPlanClient()
    print("Connecting to planner")

    if (PlanServerStatus is not True):
    	return


    # Loop until ros shuts down
    rospy.set_param(next_iter_param, False)
    while(not rospy.is_shutdown()):
        # Send the current observation of the environment
    	MovoPlanner.sendCurrentObservation(MovoClient)
        # Process and execute the next best action
        MovoPlanner.processNextAction(MovoClient)
        MovoPlanner.executeAction(MovoClient)


if __name__ == "__main__":
    main()
    rospy.spin()