#ifndef POPCORN_VISION_TABLETOP_OBJECT_LOCALIZATION_H
#define POPCORN_VISION_TABLETOP_OBJECT_LOCALIZATION_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/PointHeadAction.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
#include <grasp_msgs/Object.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <boost/smart_ptr.hpp>

#include <popcorn_vision/TabletopObjectLocalizationAction.h>
#include <popcorn_vision/GetCandyHeightAction.h>
#include <popcorn_vision/GetMarkers.h>
#include <popcorn_vision/GetPose.h>
#include <popcorn_vision/pcl_tracker.h>

#include <vector>

namespace popcorn_vision {

typedef pcl::PointXYZRGB PointT;
typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> PointHeadClient;
typedef actionlib::SimpleActionServer<popcorn_vision::TabletopObjectLocalizationAction> ServerT;
typedef actionlib::SimpleActionServer<popcorn_vision::GetCandyHeightAction> CandyHeightServer;
typedef std::vector<moveit_msgs::CollisionObject> ColObjs;
typedef boost::shared_ptr<ColObjs> ColObjsPtr;

class TabletopObjectLocalization {
 public:
  TabletopObjectLocalization(ros::NodeHandle _nh);

 private:
  void kinectCallback(const pcl::PointCloud<PointT>::ConstPtr& _cloud);
  void localizationCallback(const popcorn_vision::TabletopObjectLocalizationGoalConstPtr &goal);

  void getCandyHeightCallback(const popcorn_vision::GetCandyHeightGoalConstPtr &goal);
  bool getCandyBoxWithFiducial(geometry_msgs::PoseStamped& box_pose);
  bool getCandyBoxWithoutFiducial(geometry_msgs::PoseStamped& box_pose);

  bool getModeCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool pauseOrResumeTracking(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool getTrackedPoseCallback(popcorn_vision::GetPose::Request &req, popcorn_vision::GetPose::Response &res);
  popcorn_vision::TabletopObjectLocalizationResult localizeWithOptitrack(const popcorn_vision::TabletopObjectLocalizationGoalConstPtr &goal);
  popcorn_vision::TabletopObjectLocalizationResult localizeWithKinect(const popcorn_vision::TabletopObjectLocalizationGoalConstPtr &goal,
      GraspObjectsPtr raw_objects=GraspObjectsPtr(), GraspObjectsPtr raw_supports=GraspObjectsPtr());
  popcorn_vision::TabletopObjectLocalizationResult localizeWithKinectTracking(const popcorn_vision::TabletopObjectLocalizationGoalConstPtr &goal);
  void kinectDebugLoop();
  void kinectOptitrackDebugLoop();
  void kinectTrackingDebugLoop();
  void segmentKinect(std::vector<grasp_msgs::Object>& objects,
      std::vector<grasp_msgs::Object>& planes,
      pcl::PointCloud<PointT>::Ptr vis_cloud=pcl::PointCloud<PointT>::Ptr());
  void trackingCallback(const ros::TimerEvent&);
  void updateTracker();
  void lookAt(const geometry_msgs::Point& target, std::string frame, double duration=1.0);
  // void segment(const pcl::PointCloud<PointT>::ConstPtr& cloud, vector<moveit_msgs::CollisionObject>* objects);

  // Enable or disable the fiducial tag tracking
  void setAlvarTracking(bool enable);

  void waitForNewCloud();

  double distance(const geometry_msgs::Point& p0, const geometry_msgs::Point& p1);

  popcorn_vision::TabletopObjectLocalizationResult getLocalizationResult(
      const popcorn_vision::TabletopObjectLocalizationGoalConstPtr &goal,
      geometry_msgs::Pose object_pose, geometry_msgs::Pose support_pose,
      shape_msgs::SolidPrimitive support_box,
      ColObjsPtr obstacles = ColObjsPtr());

  void loadObjects();

  ros::NodeHandle nh_;
  bool sim_;
  std::string base_frame_;
  std::string eef_frame_;
  bool use_candy_fiducial_;
  std::string candy_alvar_frame_;
  std::string alvar_srv_name_;

  tf2_ros::TransformBroadcaster tf_br_;


  ros::Publisher tracked_pose_pub_;
  ros::Publisher debug_pose_pub_;
  ros::Publisher debug_pose_pub2_;
  ros::Publisher cloud_pub_;
  ros::Subscriber cloud_sub_;
  pcl::PointCloud<PointT>::ConstPtr cloud_;

  boost::shared_ptr<PointHeadClient> point_head_client_;
  control_msgs::PointHeadGoal point_head_goal_;

  ros::ServiceClient enable_deep_client_;
  ros::ServiceClient get_deep_client_;

  boost::shared_ptr<ServerT> server_;
  std::string action_name_;
  std::string mode_;
  bool kinect_debug_mode_;
  bool kinect_tracking_debug_mode_;
  bool kinect_optitrack_debug_mode_;
  std::vector<moveit_msgs::CollisionObject> localized_objects_;

  boost::shared_ptr<CandyHeightServer> candy_height_server_;

  ros::ServiceServer pause_or_resume_tracking_server_;
  ros::ServiceServer get_mode_server_;
  ros::ServiceServer get_tracked_pose_server_;

  double workspace_min_x_;
  double workspace_max_x_;
  double workspace_min_y_;
  double workspace_max_y_;
  double workspace_min_z_;
  double workspace_max_z_;

  std::ofstream debug_file_;

  bool is_tracking_;
  bool track_success_;
  PclTracker pcl_tracker_;
  std::vector<std::string> object_names_;
  std::vector<pcl::PointCloud<PointT>::Ptr> object_clouds_;
  geometry_msgs::Pose old_support_pose_;
  shape_msgs::SolidPrimitive old_support_box_;
  geometry_msgs::Pose last_object_pose_;
  int current_obj_;
  ros::Time last_tracking_time_;
  ros::Timer tracking_timer_;

  geometry_msgs::Point last_look_target_;
};

} // namespace popcorn_vision

#endif
