#!/usr/bin/env python
import sys
import rospy

from gazebo_msgs.msg import ModelStates
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from moveit_msgs.msg import MoveItErrorCodes
from movo_action_clients.gripper_action_client import GripperActionClient

def main():
    # init
    rospy.init_node('robot_initializer')

    sim = rospy.get_param("~sim",False)
    if (sim):
        # Wait until gazebo is up and running
        rospy.wait_for_message('/gazebo/model_states',ModelStates)
        rospy.sleep(1)

        gripper_closed = 0.96
        gripper_open = 0.0
    else:
        gripper_closed = 0.0
        gripper_open = 0.165

    move_group = MoveGroupInterface("upper_body","base_link")
    move_group.setPlannerId("RRTConnectkConfigDefault")
    lgripper = GripperActionClient('left')
    rgripper = GripperActionClient('right')

    upper_body_joints = ["right_shoulder_lift_joint",
                        "right_shoulder_pan_joint",
                        "right_arm_half_joint",
                        "right_elbow_joint",
                        "right_wrist_spherical_1_joint",
                        "right_wrist_spherical_2_joint",
                        "right_wrist_3_joint",
                        "left_shoulder_lift_joint",
                        "left_shoulder_pan_joint",
                        "left_arm_half_joint",
                        "left_elbow_joint",
                        "left_wrist_spherical_1_joint",
                        "left_wrist_spherical_2_joint",
                        "left_wrist_3_joint",
                        "linear_joint",
                        "pan_joint",
                        "tilt_joint"]

    planning_scene = PlanningSceneInterface("base_link")
    planning_scene.clear()

    # move
    rospy.loginfo("Lifting up arms...")
    '''both_arm_lifted_up_pose = [ 2.28, 2.17,-2.56,-0.09, 0.15,1.082, \
                               -2.28,-2.17, 2.56, 0.09,-0.15,2.06, \
                                0.42, 0.0,  0.0]'''
    both_arm_lifted_up_pose = [-1.57, 0, 0, 0, 0.0, 0, -1.57,\
                               1.6, 1.5, -0.4, 2.7, 0.0, -0.5, 2.7, \
                                0.38, 0.0, 0.0]
    # right_arm_lifted_up_pose = [-2.4, -2.23, 0.0, -0.5, 0.0, -0.8, \
    #                            -2.28,-2.17, 2.56, 0.09,-0.15,2.06, \
    #                             0.15, 0.0, 0.0]
    # tuck_pose = [-2.8,   -1.48,-1.48, 0, 0, 1.571, \
    #               2.8,    1.48, 1.48, 0, 0,-1.571, \
    #               0.0371, 0.0,  0.0]

    success = False
    target_pose = both_arm_lifted_up_pose
    while not rospy.is_shutdown() and not success:
        result = move_group.moveToJointPosition(upper_body_joints, target_pose, tolerance=0.05)
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            success = True
        else:
            rospy.logwarn('result.error_code.val= '+str(result.error_code.val))

    
    lgripper.command(gripper_closed)
    rgripper.command(gripper_closed)

    rospy.sleep(1.0)
    rospy.set_param('/robot_initialized', True)

if __name__ == '__main__':
    main()
