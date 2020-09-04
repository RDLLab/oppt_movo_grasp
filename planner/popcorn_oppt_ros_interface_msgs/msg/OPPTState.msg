# Standard ROS msg header
Header header

# Robot pose
string[] joint_names
trajectory_msgs/JointTrajectoryPoint joint_position

# If gripper is 'closed' or not (e.g. hard-coded angles)
# True = is finished closing
bool gripper_closed

# Coliision objects
moveit_msgs/CollisionObject[] objects

# If object is 'grasped' or not (e.g. from tactile sensors)
# True = object is grasped
bool object_grasped
