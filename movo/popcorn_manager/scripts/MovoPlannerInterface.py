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



"""" @TODO: CHECK FOR LINES WITH COMMENTS OF THE FORM : ##"""
class MovoPlannerInterface:
    def __init__(self, sim=False):
        """ Constructor of the interface """
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.gripper_almost_fully_closed_posture = None
        self.gripper_grasp_posture = None
        
        if sim:
            self.gripper_fully_closed_posture = 1.8
            self.gripper_fully_open_posture = 0.0
        else:
            self.gripper_fully_closed_posture = -0.02
            self.gripper_almost_fully_closed_posture = 0.010
            self.gripper_fully_open_posture = 0.165     

        
        self.close_gripper_cmd = +1
        self.open_gripper_cmd  = -1
        self.noop_gripper_cmd  =  0
        self.is_grasping = False
        self.last_gripper_action = self.noop_gripper_cmd
        self.obj_loc = None
        self.rightEE_belief_x = []
        self.rightEE_belief_y = []
        self.rightEE_belief_z = []
        self.obj_belief_x = []
        self.obj_belief_y = []
        self.obj_belief_z = []
        self.obs = None



    """ --------------------------------------------------- INITIAL BELIEF METHODS -------------------------------------------- """
    def setupInitBeliefClient(self):
        """ Function to synchronise the robot controller with the initial belief interface on the Planner side 
        @return: The function returns a boolean indicating the status of the initial belief client connection"""
        rospy.loginfo('waiting for oppt_init_belief_srv_param...')
        oppt_init_belief_srv_param = '/oppt_init_belief_server_initialized'

        # wait for the server parameter to be ready to indicate that the planner side is ready to interact
        while not rospy.is_shutdown():
            if rospy.has_param(oppt_init_belief_srv_param):
                if rospy.get_param(oppt_init_belief_srv_param):
                    break
            rospy.sleep(0.1)

        # Set up the init belief client with the proper action name. Temporarily using the belief action name directly.
        init_belief_action_name = rospy.get_param('~oppt_ros_interface/init_belief_action_name') #'/oppt_ros_interface/init_belief'
        self.init_belief_client = actionlib.SimpleActionClient(init_belief_action_name, OPPTInitBeliefAction)

        # Attempt to connect
        rospy.loginfo('Waiting for actionServers = '+ init_belief_action_name)
        init_belief_server_connected =  self.init_belief_client.wait_for_server(timeout=rospy.Duration(10.0)) 


        # Check if the initial belief connection was successful
        if (not init_belief_server_connected):
            rospy.logfatal("init_belief_server: NOT connected!!!")
            return False
        

        return True



     ## NEED TO MAKE SURE INFORMATION ABOUT THE PERCEPTION IS CORRECTLY PASSED INTO THIS
    def sendInitBelief(self, MovoController, initial_objs, initial_obstacles):
        """ Function to send information about the initial belief of the robot to the the OPPT_Planner_Server"""
        init_belief_goal = OPPTInitBeliefGoal()
        init_belief_goal.initial_state.header.stamp = rospy.Time.now()
        init_belief_goal.initial_state.header.frame_id = 'base_link'
        init_belief_goal.initial_state.objects = initial_objs
    
        for obstacle in initial_obstacles:
            init_belief_goal.initial_state.objects.append(obstacle)

        init_belief_goal.initial_state.object_grasped = False
        init_belief_goal.initial_state.gripper_closed = False

        """Add the right arm and torso joint values to the initial belief information stream"""
        init_belief_goal.initial_state.joint_names = MovoController.getBodyJointNames("RightArm") + MovoController.getBodyJointNames("Torso")
        RightArmJointValues = MovoController.getJointAngles("RightArm")
        TorsoJointValues = MovoController.getJointAngles("Torso")
        init_belief_goal.initial_state.joint_position.positions = RightArmJointValues + TorsoJointValues

        rospy.loginfo('sending init_belief_goal...')
        self.init_belief_client.send_goal(init_belief_goal)

        rospy.loginfo('waiting for init_belief_result...')
        init_belief_finished = self.init_belief_client.wait_for_result( rospy.Duration(3.0) )
        init_belief_result = self.init_belief_client.get_result()
        rospy.loginfo('init_belief_finished= ' + str(init_belief_finished))

        if not init_belief_finished:
            rospy.logfatal('init_belief_finished: false!!!')
            return False


        return True





    """ ----------------------------------------------- Planner interface methods ---------------------------------------------"""
    def setupOPPTPlanClient(self):
        """ Function to synchronise the OPPT_Planner_Client (Popcorn) with the OPPT_Planner_Server on the Planner side 
        @return: The function returns a boolean indication the status of the initial belief client connection"""
        rospy.loginfo('waiting for oppt_plan_srv_param...')
        oppt_plan_srv_param = '/oppt_plan_server_initialized'

        # Wait until the server has indicated its start
        while not rospy.is_shutdown():
            if rospy.has_param(oppt_plan_srv_param):
                if rospy.get_param(oppt_plan_srv_param):
                    break
            rospy.sleep(0.1)

        plan_action_name =  rospy.get_param('~oppt_ros_interface/plan_action_name')
        self.plan_client = actionlib.SimpleActionClient(plan_action_name, OPPTPlanAction)

        rospy.loginfo('Waiting for actionServe r= ' + plan_action_name)
        plan_server_connected = self.plan_client.wait_for_server(timeout=rospy.Duration(1.0))
        if plan_server_connected:
            rospy.loginfo("plan_server: connected.")
            return True
        else:
            rospy.logfatal("plan_server: NOT connected!!!")
            return False



    def sendCurrentObservation(self, MovoController):
        """ Pack the current state of the robot and its perception into a observation message and send it to the OPPT_Planner_Server
            @param: Grasper: The Grasper object related to the perception module of the popcorn pipeline
                    MovoController: The interface object to control the movement and executions of the robot 
            @return: None """

        parent_frame_id = 'base_link'
        rate = rospy.Rate(0.1)
        # rospy.loginfo('Waiting for TF....')
        # rospy.loginfo("Getting observation")
        while (not rospy.is_shutdown()):
            try:
                ee_trans = self.tfBuffer.lookup_transform(parent_frame_id, 'right_ee_link', rospy.Time(0))
                break
            except Exception as e: #(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print(e)
                rate.sleep()
                continue


        # Pack the observation structure
        obs = OPPTObservation()

        # Header info
        obs.header.stamp = rospy.Time.now()
        obs.header.frame_id = parent_frame_id
        obs.gripper_pose.position.x = ee_trans.transform.translation.x
        obs.gripper_pose.position.y = ee_trans.transform.translation.y
        obs.gripper_pose.position.z = ee_trans.transform.translation.z
        obs.gripper_pose.orientation = ee_trans.transform.rotation

        # Object's pose info
        p = Pose()
        p.position.x, p.position.y, p.position.z = [float('nan'), float('nan'), float('nan')]
        obs.object_pose = p

        # Other state information
        obs.gripper_closed = (True if self.last_gripper_action==self.close_gripper_cmd else False) # assume that `close-gripper` action always closes the gripper
        obs.object_grasped = int(self.is_grasping)
        obs.joint_names = MovoController.getBodyJointNames("RightArm") + MovoController.getBodyJointNames("Torso")

        # Fill in the joint values for this message
        RightArmJointValues = MovoController.getJointAngles("RightArm")
        # print("RIGHT ARM JOINT VALUES" , RightArmJointValues)
        TorsoJointValues = MovoController.getJointAngles("Torso")
        obs.joint_position.positions = RightArmJointValues + TorsoJointValues

        # Send the observation
        plan_goal = OPPTPlanGoal()
        plan_goal.observation = obs

        self.plan_client.send_goal(plan_goal)
        self.obs = obs
        





    """----------------------------------------------------- OTHER METHODS ---------------------------------"""
    def processNextAction(self, MovoController):
        """ Waits for the next action from the OPPT_Planner_Server and executes according to the joint limits of the robot
            @param: Grasper: The Grasper object related to the perception module of the popcorn pipeline
                    MovoController: The interface object to control the movement and executions of the robot """
        # rospy.loginfo("time before WAIT FOR RESULT")
        plan_finished = self.plan_client.wait_for_result()
        # rospy.loginfo("time after WAIT FOR RESULT")
        plan_result = self.plan_client.get_result()
        # rospy.loginfo("time after getting RESULT")
        self.plan_result = plan_result

        try:
            assert plan_result is not None
            assert self.plan_client.get_state() == GoalStatus.SUCCEEDED
            assert plan_finished == True
            # rospy.loginfo("OPPT Plan found")
        except Exception as e:
            print(e)
            rospy.logwarn('No OPPT plan, exitting')
            print(plan_result)
            print(self.plan_client.get_state())
            print(plan_finished)
            rospy.sleep(2.0)
            ## MOVO ROBOT TO PRE GRASP POSE
            MovoController.bringToPreGraspPose()
            # break

        self.update_next_observation = plan_result.action.update_observation



    def executeAction(self, MovoController):
        # Controllers to move torso and right arm
        right_gripper_controller = MovoController.getBodyController("RightGripper")
        right_arm_controller = MovoController.getBodyController("RightArm")
        torso_controller = MovoController.getBodyController("Torso")

        # Execute close gripper command
        if (self.plan_result.action.gripper_command == self.close_gripper_cmd) and (self.last_gripper_action != self.close_gripper_cmd):
            
            rospy.loginfo("CLOSING GRIPPER")
            self.last_gripper_action = self.close_gripper_cmd
            self.gripper_grasp_posture = self.gripper_fully_closed_posture
            #Linearize closing motion of gripper
            num_traj_points = 5
            gripper_trajectory = np.linspace(self.gripper_fully_open_posture, self.gripper_grasp_posture, num_traj_points)
            for i in xrange(num_traj_points):
                right_gripper_controller.command(gripper_trajectory[i])
                rospy.sleep(0.25) # TODO unhardcode sleep duration

            # wait to have good estimate of whether grasping is successful from grasp_estimator node
            # Based on http://www.kinovarobotics.com/wp-content/uploads/2017/10/Kinova-Specs-Grippers-Web-170512.pdf.pdf
            # Opening or closing travel time = 1.2 sec
            gripper_travel_time = 1.2
            rospy.sleep(gripper_travel_time + 1.5)
            grasp_estimator_param = '/grasp_estimator/right_is_grasping'
            try:
                self.is_grasping = rospy.get_param(grasp_estimator_param)
            except KeyError:
                rospy.logfatal("grasp_estimator_param: not exist!!!")
                self.grasper.pause_tracking()
                return

        # Execute open gripper command
        elif (self.plan_result.action.gripper_command == self.open_gripper_cmd) and (self.last_gripper_action != self.open_gripper_cmd):
            self.last_gripper_action = self.open_gripper_cmd
            rospy.loginfo("OPENING GRIPPER")
            self.gripper_grasp_posture = self.gripper_fully_open_posture
            gripper_trajectory = np.linspace(self.gripper_fully_closed_posture, self.gripper_grasp_posture, 5)
            for i in xrange(5):
                right_gripper_controller.command(gripper_trajectory[i])
                rospy.sleep(0.2) # TODO unhardcode sleep duration   

        # Execute right arm movement command
        else:
            # rospy.loginfo("MOVING RIGHT ARM")
            right_arm_start_vals = MovoController.getJointAngles("RightArm")
            right_arm_joint_names = MovoController.getBodyJointNames("RightArm")

            # Increment joint angles
            right_arm_target_vals = right_arm_start_vals[:]
            for index, joint_name in enumerate(self.plan_result.action.joint_names):
                joint_index = right_arm_joint_names.index(joint_name)
                right_arm_target_vals[joint_index] += self.plan_result.action.point_increments[0].positions[index]

            # Execute to move to new target vals
            MovoController.execTrajectory(right_arm_start_vals, right_arm_target_vals, "RightArm", num_traj_points=5, timeout=0.75)