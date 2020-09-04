#!/usr/bin/env python
import rospy
from movo_action_clients.gripper_action_client import GripperActionClient

def main():
	rospy.init_node("GripperActionClient")
	rgripper = GripperActionClient('right')
	rgripper.command(0.165)

if __name__ == '__main__':
    main()
