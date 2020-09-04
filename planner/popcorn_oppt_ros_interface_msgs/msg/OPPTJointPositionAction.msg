Header header
string[] joint_names
trajectory_msgs/JointTrajectoryPoint[] point_increments
int8 gripper_command # +1: close, -1: open, 0: do nothing
int8 left_arm_command # +1: set to wall, -1 bring home, 0 do nothing
bool update_observation
trajectory_msgs/JointTrajectoryPoint[] beliefParticles
geometry_msgs/Point[] knucklePose
geometry_msgs/Point[] rightEEPose

