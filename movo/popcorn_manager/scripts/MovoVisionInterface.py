import tf2_ros
import rospy
import actionlib
import matplotlib
matplotlib.use('Agg')
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Pose
from popcorn_oppt_ros_interface_msgs.msg import OPPTPlanAction, OPPTPlanGoal
from popcorn_oppt_ros_interface_msgs.msg import OPPTInitBeliefAction, OPPTInitBeliefGoal
from popcorn_oppt_ros_interface_msgs.msg import OPPTObservation
from popcorn_vision.msg import TabletopObjectLocalizationAction, TabletopObjectLocalizationGoal
from std_srvs.srv import *
from Vision import Vision
"""The vision module handles camera initialization and is responsible for communicating the the object location and orientation 
   to the top level python script. If you need to inplement your own version of the vision module, then please refer to vision.txt in
   the documentation folder to understand the acceptanle format in for the popcorn_manager module """


class MovoVisionInterface(Vision):

    def __init__(self):
        self.initial_objs = []
        self.initial_obstacles = []
        action_name = "/tabletop_object_localizer/localize_object"
        self.__client = actionlib.SimpleActionClient(action_name, TabletopObjectLocalizationAction)
        self.__goal = TabletopObjectLocalizationGoal()    
        self.using_tracking = False
        get_mode_name = "/tabletop_object_localizer/get_mode"
        rospy.loginfo("Waiting for " + get_mode_name)
        rospy.wait_for_service(get_mode_name)
        self.__get_mode_client = rospy.ServiceProxy(get_mode_name, Trigger)

    def getVision(self):
        return  self.getSceneObjectsObstacles(target_obj_name="tube")

    """The following function simply separates the recognizable object list into support and target objects"""

    def objStrSplit(self, obj_str):
        obj, raw_support = obj_str.split('@')
        if '!' in raw_support:
            _ = raw_support.split('!')
            support = _[0]
            obstacles = _[1:]
        else:
            support = raw_support
            obstacles = []

        obj_and_support_names = []
        obj_and_support_names.append( (obj,support) )
        for o in obstacles:
            obj_and_support_names.append( (o,support) )

        return obj_and_support_names # include obstacles (as objects)


    """The following function makes the high level function calls to the low level vision 
        module to identify the target object, support and obstacles. It also returns their locations wrapped as 
        a geometry msgs"""
    def getSceneObjectsObstacles(self, target_obj_name, timeout=180.0):
        target = target_obj_name + "@table"
        obj_and_support_names = self.objStrSplit(target)
        obj_idx = 0; support_idx = 1
        object_names = [i[obj_idx] for i in obj_and_support_names]
        support_names = [i[support_idx] for i in obj_and_support_names]
        self.__goal.object_names = object_names
        self.__goal.support_names = support_names

        rospy.logdebug('sending goal...')
        self.__client.send_goal(self.__goal)

        rospy.logdebug('waiting for result...')
        finished = self.__client.wait_for_result( rospy.Duration(timeout) )
        rospy.logdebug('finished= '+str(finished))

        result = self.__client.get_result()
        objs = result.objects
        supports = result.supports
        obstacles = objs[len(supports):]
        objs = objs[:len(supports)]
        obj_and_support_list = []
        for i, obj in enumerate(objs):
            obj_and_support_list.append((obj, supports[i]))

        obj, support = obj_and_support_list[0]
        return [obj,support], obstacles


    # The following  function returns the tracking mode of the vision client
    def getMode(self):
        mode = self.get_mode()
        self.using_tracking = mode == 'kinect_tracking' or mode == 'deep_kinect_tracking' or mode == 'deep'
        return self.using_tracking

    def get_mode(self):
        rospy.logdebug('Getting mode')
        try:
            resp = self.__get_mode_client()
            mode = resp.message
        except:
            mode = None
        rospy.logdebug('Result: ' + str(mode))
        return mode    