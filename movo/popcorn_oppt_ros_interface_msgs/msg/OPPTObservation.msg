# Standard ROS msg header
Header header

# Pose of gripper
geometry_msgs/Pose gripper_pose

# If gripper is 'closed' or not (e.g. hard-coded angles)
# True = is finished closing
bool gripper_closed

# Pose of object
geometry_msgs/Pose object_pose

# If object is 'grasped' or not (e.g. from tactile sensors)
# True = object is grasped
bool object_grasped

# Robot pose
string[] joint_names
trajectory_msgs/JointTrajectoryPoint joint_position
