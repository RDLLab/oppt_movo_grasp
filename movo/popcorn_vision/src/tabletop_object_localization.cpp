#include <assert.h>
#include <fstream>
#include <cmath>
#include <ctime>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_datatypes.h>

#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <popcorn_vision/tabletop_object_localization.h>
#include <popcorn_vision/pcl_tools.h>

using namespace std;

tf2_ros::Buffer g_tf_buffer;

namespace popcorn_vision {

TabletopObjectLocalization::TabletopObjectLocalization(ros::NodeHandle _nh)
  : nh_(_nh), is_tracking_(false), track_success_(false), current_obj_(-1) {
  nh_.getParam("sim", sim_);
  nh_.getParam("mode", mode_);
  nh_.param("base_frame", base_frame_, std::string("base_link"));
  nh_.param("eef_frame", eef_frame_, std::string("right_ee_link"));
  // Set to false for now
  // nh_.param("use_candy_fiducial", use_candy_fiducial_, false);
  // nh_.param("candy_alvar_frame", candy_alvar_frame_, std::string("ar_marker_0"));
  nh_.param("kinect_debug_mode", kinect_debug_mode_, false);
  nh_.param("kinect_tracking_debug_mode", kinect_tracking_debug_mode_, false);
  nh_.param("kinect_optitrack_debug_mode", kinect_optitrack_debug_mode_, false);
  nh_.param("workspace_min_x", workspace_min_x_, 0.1);
  nh_.param("workspace_max_x", workspace_max_x_, 2.0);
  nh_.param("workspace_min_y", workspace_min_y_, -0.35);
  nh_.param("workspace_max_y", workspace_max_y_, 0.35);
  nh_.param("workspace_min_z", workspace_min_z_, 0.65);
  nh_.param("workspace_max_z", workspace_max_z_, 1.3);

  //alvar_srv_name_ = "/ar_track_alvar/set_parameters";

  string cloud_topic = "/movo_camera/point_cloud/points";
  nh_.getParam("cloud_topic", cloud_topic);
  // if (sim_) assert(cloud_topic=="/kinect2/sd/points");
  ROS_INFO_STREAM("cloud_topic= " << cloud_topic);


  cout << "I AM INSIDE THE VISION" << std::endl;

  tracked_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/tracked_object_pose", 1);
  debug_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/tabletop_debug_pose", 1);
  debug_pose_pub2_ = nh_.advertise<geometry_msgs::PoseStamped>("/tabletop_debug_pose2", 1);
  cloud_pub_ = nh_.advertise< pcl::PointCloud<PointT> >("/tabletop_debug_cloud", 1);
  cloud_sub_ = nh_.subscribe< pcl::PointCloud<PointT> >
    (cloud_topic, 1, &TabletopObjectLocalization::kinectCallback, this);


  // Tracker init
  loadObjects();
  pcl_tracker_.initRos(nh_);

  ROS_INFO("Waiting for head controller.");
  point_head_client_.reset(new PointHeadClient("movo/head_controller/point_head_action", true));
  point_head_client_->waitForServer();
  point_head_goal_.max_velocity = 1.0;
  point_head_goal_.pointing_axis.x = 1.0;
  point_head_goal_.pointing_frame = "/movo_camera_link";
  ROS_INFO("Head controller ready");

  // Deep node
  if (mode_ == "deep" || mode_ == "deep_kinect_tracking")
  {
    ROS_INFO("Waiting for deep node.");
    enable_deep_client_ = nh_.serviceClient<std_srvs::SetBool>("/enable_deep_node");
    get_deep_client_ = nh_.serviceClient<popcorn_vision::GetMarkers>("/get_deep_detections");
    enable_deep_client_.waitForExistence();
    std_srvs::SetBool srv;
    srv.request.data = false;
    enable_deep_client_.call(srv);
    ROS_INFO("Done");
  }

  if (kinect_debug_mode_)
  {
    ROS_INFO_STREAM("Entering kinect debug loop.");
    kinectDebugLoop();
    return;
  }
  else if (kinect_tracking_debug_mode_)
  {
    ROS_INFO_STREAM("Entering kinect tracking debug loop.");
    kinect_debug_mode_ = true;
    kinectTrackingDebugLoop();
    return;
  }

  if (kinect_optitrack_debug_mode_)
  {
    ROS_INFO_STREAM("Entering kinect optitrack debug loop.");
    kinectOptitrackDebugLoop();
    return;
  }

  /** NOT USED AT THE MOMENT **/
  // ROS_INFO("Waiting for alvar tracker");
  // ros::service::waitForService(alvar_srv_name_, -1);
  // setAlvarTracking(false);
  // ROS_INFO("Alvar tracker ready. Disabling until needed.");

  action_name_ = "localize_object";
  server_.reset(new ServerT(nh_, action_name_,
    boost::bind(&TabletopObjectLocalization::localizationCallback, this, _1), false));
  server_->start();
  pause_or_resume_tracking_server_ = nh_.advertiseService("pause_or_resume_tracking",
      &TabletopObjectLocalization::pauseOrResumeTracking, this);

  get_mode_server_ = nh_.advertiseService("get_mode",
      &TabletopObjectLocalization::getModeCallback, this);
  get_tracked_pose_server_ = nh_.advertiseService("get_tracked_pose",
      &TabletopObjectLocalization::getTrackedPoseCallback, this);

  localized_objects_.clear();

  // candy_height_server_.reset(new CandyHeightServer(nh_, "get_candy_height",
  //     boost::bind(&TabletopObjectLocalization::getCandyHeightCallback, this, _1), false));
  // candy_height_server_->start();

  ROS_INFO_STREAM("tabletop_object_localizer: initialized, sim= " << sim_ << ", mode= " << mode_);
}

void TabletopObjectLocalization::kinectCallback(const pcl::PointCloud<PointT>::ConstPtr& cloud) {
  cloud_ = cloud;
}

void TabletopObjectLocalization::localizationCallback(const popcorn_vision::TabletopObjectLocalizationGoalConstPtr &goal) {
  popcorn_vision::TabletopObjectLocalizationResult result;

  if (mode_=="optitrack")
    result = localizeWithOptitrack(goal);
  else if (mode_=="kinect")
    result = localizeWithKinect(goal);
  else if (mode_ == "kinect_tracking" || mode_ == "deep_kinect_tracking" || mode_ == "deep")
    result = localizeWithKinectTracking(goal);
  else {
    assert(false && "[FATAL] TabletopObjectLocalization mode_: Unknown!");
    return;
  }

  server_->setSucceeded(result, "Succeeded.");
}



bool TabletopObjectLocalization::pauseOrResumeTracking(std_srvs::SetBool::Request &req,
    std_srvs::SetBool::Response &res)
{
  // If mode is not tracking respond false
  if (mode_ != "kinect_tracking" && mode_ != "deep_kinect_tracking" && mode_ != "deep")
  {
    res.success = false;
    return true;
  }

  // If resume is requested, look at object again in case robot moved
  //if (!is_tracking_ && req.data)
  //{
  //  geometry_msgs::Pose pose = pcl_tracker_.getPose();
  //  lookAt(pose.position, base_frame_);
  //  ros::Duration(1.0).sleep();
  //}
  is_tracking_ = req.data;
  res.success = true;
  return true;
}

bool TabletopObjectLocalization::getModeCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = true;
  res.message = mode_;
  return true;
}

bool TabletopObjectLocalization::getTrackedPoseCallback(popcorn_vision::GetPose::Request &req, popcorn_vision::GetPose::Response &res)
{
  // Deep modes do not run continuously but are updated only when requested
  if (mode_ == "deep" || mode_ == "deep_kinect_tracking")
  {
    updateTracker();
    if (!track_success_) {
      res.success = false;
      return true;
    }
  }
  res.success = true;
  res.pose.header.frame_id = base_frame_;
  res.pose.header.stamp = ros::Time::now();
  res.pose.pose = last_object_pose_;
  return true;
}

popcorn_vision::TabletopObjectLocalizationResult TabletopObjectLocalization::localizeWithOptitrack(const popcorn_vision::TabletopObjectLocalizationGoalConstPtr &goal) {
  ROS_INFO_STREAM("localizeWithOptitrack: nObjects= " << goal->object_names.size());
  assert( goal->object_names.size()==goal->support_names.size() );

  const string optitrack_frame_suffix = "_gt";
  const size_t n_iter_max = 5;

  popcorn_vision::TabletopObjectLocalizationResult result;
  result.objects.resize(goal->object_names.size());
  result.supports.resize(goal->support_names.size());

  for (size_t i=0; i<goal->object_names.size(); i++) {
    string object_name = goal->object_names[i];
    string support_name = goal->support_names[i];
    string object_frame =  object_name + optitrack_frame_suffix;
    string support_frame =  support_name + optitrack_frame_suffix;

    geometry_msgs::TransformStamped object_tf, support_tf;
    bool found = false;
    for (size_t j=0; j<n_iter_max and nh_.ok(); j++) {
      try{
        ROS_INFO_STREAM_ONCE("lookupTransform for "<< object_frame << " n_iter_max= " << n_iter_max);
        object_tf = g_tf_buffer.lookupTransform(base_frame_, object_frame,
                                                ros::Time(0), ros::Duration(3.0));
        support_tf = g_tf_buffer.lookupTransform(base_frame_, support_frame,
                                                 ros::Time(0), ros::Duration(3.0));
        found = true;
        break;
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN_ONCE("%s",ex.what());
        ros::Duration(0.1).sleep();
        continue;
      }
    }

    if (found) {
      // The object
      ROS_DEBUG_STREAM("object_tf= " << object_tf.transform.translation.x << ", "
        << object_tf.transform.translation.y << ", " << object_tf.transform.translation.z);

      moveit_msgs::CollisionObject object;

      object.header.frame_id = object_tf.header.frame_id;
      object.id = object_name;
      object.type.key = object_name;
      object.type.db = "popcorn_tabletop_objects";
      string object_param_prefix = string(object.type.db+"/"+object.type.key);

      object.primitives.resize(1);
      int object_primitive_type;
      nh_.getParam(string(object_param_prefix+"/primitive_type"), object_primitive_type);
      object.primitives[0].type = object_primitive_type;

      object.primitives[0].dimensions.resize(2);
      nh_.getParam(string(object_param_prefix+"/dimension/height"),
                   object.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT]);
      nh_.getParam(string(object_param_prefix+"/dimension/radius"),
                   object.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS]);

      object.primitive_poses.resize(1);
      object.primitive_poses[0].position.x = object_tf.transform.translation.x;
      object.primitive_poses[0].position.y = object_tf.transform.translation.y;
      object.primitive_poses[0].position.z = object_tf.transform.translation.z;
      object.primitive_poses[0].orientation = support_tf.transform.rotation; //object_tf.transform.rotation;
      object.operation = moveit_msgs::CollisionObject::ADD;

      result.objects[i] = object;

      // The support
      moveit_msgs::CollisionObject support;

      support.header.frame_id = support_tf.header.frame_id;
      support.id = support_name;
      support.type.key = support_name;
      support.type.db = "popcorn_tabletop_object_supports";
      string support_param_prefix = string(support.type.db+"/"+support.type.key);

      support.primitives.resize(1);
      int support_primitive_type;
      nh_.getParam(string(support_param_prefix+"/primitive_type"), support_primitive_type);
      support.primitives[0].type = support_primitive_type;

      support.primitives[0].dimensions.resize(3);
      nh_.getParam(string(support_param_prefix+"/dimension/box_x"),
                   support.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X]);
      nh_.getParam(string(support_param_prefix+"/dimension/box_y"),
                   support.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y]);
      nh_.getParam(string(support_param_prefix+"/dimension/box_z"),
                   support.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z]);

      support.primitive_poses.resize(1);
      support.primitive_poses[0].position.x = support_tf.transform.translation.x;
      support.primitive_poses[0].position.y = support_tf.transform.translation.y;
      support.primitive_poses[0].position.z = support_tf.transform.translation.z;
      support.primitive_poses[0].orientation = support_tf.transform.rotation;
      support.operation = moveit_msgs::CollisionObject::ADD;

      result.supports[i] = support;
    }
    else {
      ROS_WARN_STREAM("Object= " << object_name << ": NOT FOUND!");
    }
  }

  ROS_INFO_STREAM("localizeWithOptitrack: end");
  return result;
}

popcorn_vision::TabletopObjectLocalizationResult TabletopObjectLocalization::localizeWithKinect(const popcorn_vision::TabletopObjectLocalizationGoalConstPtr &goal,
    GraspObjectsPtr raw_objects, GraspObjectsPtr raw_supports)
{

  ROS_INFO("LOCALIZE WITH KINECT");


  // Cloud for visualization/debugging purposes
  pcl::PointCloud<PointT>::Ptr vis_cloud(new pcl::PointCloud<PointT>);

  // Scanning targets to point kinect at
  std::vector<geometry_msgs::Point> targets;
  geometry_msgs::Point p;
  p.x = 1.0;
  p.y = 0.0;
  p.z = 0.75;
  targets.push_back(p);
  p.x = 0.75;
  targets.push_back(p);
  p.x = 0.8;
  p.y = 0.15;
  targets.push_back(p);

  // Run scans
  std::vector<grasp_msgs::Object> object_detections;

  if (!raw_supports)
    raw_supports.reset(new std::vector<grasp_msgs::Object>());
  std::vector<grasp_msgs::Object>& support_detections = *raw_supports;

  // Threshold in meters for assuming two detections are of the same object
  double thresh = 0.07;

  for (std::size_t i = 0; i < targets.size(); i++)
  {
    // Point head at scanning target
    cout << "base frame: " << base_frame_ << endl;
    cout << "target: " << targets[i] << endl;
    lookAt(targets[i], base_frame_);
    ros::Duration(0.5).sleep();

    // Run kinect segmentation
    std::vector<grasp_msgs::Object> temp_supports;
    segmentKinect(object_detections, temp_supports, vis_cloud);

    //vis_cloud->header.frame_id = base_frame_;
    //pcl_conversions::toPCL(ros::Time::now(), vis_cloud->header.stamp);
    //cloud_pub_.publish(vis_cloud);

    if (temp_supports.size() > 0)
      support_detections.push_back(temp_supports[0]);
  }

  if (object_detections.size() == 0)
  {
    ROS_ERROR("Could not find any objects");
    return popcorn_vision::TabletopObjectLocalizationResult();
  }
  if (support_detections.size() == 0)
  {
    ROS_ERROR("Could not find any supports");
    return popcorn_vision::TabletopObjectLocalizationResult();
  }

  // Combine object detections
  if (!raw_objects)
    raw_objects.reset(new std::vector<grasp_msgs::Object>());
  for (std::size_t i = 0; i < object_detections.size(); i++)
  {
    const geometry_msgs::Pose& p1 = object_detections[i].primitive_poses[0];

    // Combine with existing detection if close enough
    bool added = false;
    for (std::size_t j = 0; j < raw_objects->size(); j++)
    {
      const geometry_msgs::Pose& p2 = (*raw_objects)[j].primitive_poses[0];
      if (distance(p1.position, p2.position) < thresh)
      {
        // Add pose to vector of poses, will be averaged later
        (*raw_objects)[j].primitive_poses.push_back(p1);

        // Take larger point cloud
        if (object_detections[i].point_cluster.data.size() > (*raw_objects)[j].point_cluster.data.size())
          (*raw_objects)[j].point_cluster = object_detections[i].point_cluster;

        added = true;
        break;
      }
    }

    // Could not add to existing detection, so add new object
    if (!added)
      raw_objects->push_back(object_detections[i]);
  }

  // Calculate average object poses
  for (std::size_t i = 0; i < raw_objects->size(); i++)
  {
    geometry_msgs::Pose pose;
    std::size_t num_poses = (*raw_objects)[i].primitive_poses.size();
    for (std::size_t j = 0; j < num_poses; j++)
    {
      pose.position.x += (*raw_objects)[i].primitive_poses[j].position.x;
      pose.position.y += (*raw_objects)[i].primitive_poses[j].position.y;
      pose.position.z += (*raw_objects)[i].primitive_poses[j].position.z;
    }

    pose.position.x /= num_poses;
    pose.position.y /= num_poses;
    pose.position.z /= num_poses;

    // TODO average orientation too
    pose.orientation = (*raw_objects)[i].primitive_poses[0].orientation;

    (*raw_objects)[i].primitive_poses.clear();
    (*raw_objects)[i].primitive_poses.push_back(pose);
  }

  // For now take the object closest to first look target
  geometry_msgs::Pose object_pose;
  double min_dist = 9999;
  for (std::size_t i = 0; i < raw_objects->size(); i++)
  {
    double dist = distance(targets[0], (*raw_objects)[i].primitive_poses[0].position);
    if (dist < min_dist)
    {
      min_dist = dist;
      object_pose = (*raw_objects)[i].primitive_poses[0];
    }
  }

  // Combined support detections
  geometry_msgs::Pose support_pose;
  shape_msgs::SolidPrimitive support_box;
  double x_min = 1000.0, y_min = 1000.0, z_min = 1000.0;
  double x_max = -1000.0, y_max = -1000.0, z_max = -1000.0;
  for (std::size_t i = 0; i < support_detections.size(); i++)
  {
    double x = support_detections[i].primitive_poses[0].position.x;
    double y = support_detections[i].primitive_poses[0].position.y;
    double z = support_detections[i].primitive_poses[0].position.z;
    double length = support_detections[i].primitives[0].dimensions[0];
    double width = support_detections[i].primitives[0].dimensions[1];
    double height = support_detections[i].primitives[0].dimensions[2];
    x_min = fmin(x_min, x - 0.5 * length);
    y_min = fmin(y_min, y - 0.5 * width);
    z_min = fmin(z_min, z - 0.5 * height);
    x_max = fmax(x_max, x + 0.5 * length);
    y_max = fmax(y_max, y + 0.5 * width);
    z_max = fmax(z_max, z + 0.5 * height);
  }
  support_pose.position.x = (x_min + x_max)/2;
  support_pose.position.y = (y_min + y_max)/2;
  support_pose.position.z = (z_min + z_max)/2;
  double safety_margin = 0.02;
  support_box.dimensions.push_back(x_max - x_min + safety_margin);
  support_box.dimensions.push_back(y_max - y_min + safety_margin);
  support_box.dimensions.push_back(z_max - z_min + safety_margin);

  // Publish visualization cloud for debugging
  vis_cloud->header.frame_id = base_frame_;
  pcl_conversions::toPCL(ros::Time::now(), vis_cloud->header.stamp);
  cloud_pub_.publish(vis_cloud);

  ROS_INFO_STREAM("localizeWithKinect: end");
  return getLocalizationResult(goal, object_pose, support_pose, support_box);
}

popcorn_vision::TabletopObjectLocalizationResult TabletopObjectLocalization::localizeWithKinectTracking(
    const popcorn_vision::TabletopObjectLocalizationGoalConstPtr &goal)
{

  ROS_INFO("LOCALIZE WITH KINECT TRACKING");

  // Initialize tracking
  // Get initial poses with old kinect method
  is_tracking_ = false;
  popcorn_vision::TabletopObjectLocalizationResult old_result;
  GraspObjectsPtr raw_objects(new std::vector<grasp_msgs::Object>());
  GraspObjectsPtr raw_supports(new std::vector<grasp_msgs::Object>());
  while (ros::ok() && (raw_objects->size() == 0 || old_result.supports.size() == 0))
  {
    raw_objects->clear();
    old_result = localizeWithKinect(goal, raw_objects, raw_supports);
    ROS_INFO_STREAM("Found " << raw_objects->size() << " objects and " << raw_supports->size() << " supports");
    ros::Duration(1.0).sleep();
  }
  old_support_pose_ = old_result.supports[0].primitive_poses[0];
  old_support_box_ = old_result.supports[0].primitives[0];

  // Determine which object was requested
  for (std::size_t i = 0; i < object_names_.size(); i++)
  {
    if (object_names_[i] == goal->object_names[0])
    {
      current_obj_ = i;
      pcl_tracker_.setObject(object_clouds_[current_obj_]);
      ROS_INFO_STREAM("Requested object is " << object_names_[i]);
      break;
    }
  }
  if (current_obj_ == -1)
  {
    ROS_ERROR_STREAM("Unknown object " << goal->object_names[0]);
    return popcorn_vision::TabletopObjectLocalizationResult();
  }

  // Convert point clouds into internal format
  std::vector<pcl::PointCloud<PointT>::Ptr> myclouds;
  for (std::size_t i = 0; i < raw_objects->size(); i++)
  {
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg((*raw_objects)[i].point_cluster, *cloud);
    myclouds.push_back(cloud);
  }

  // Determine which detected object is most similar to requested object
  double min_fit_ratio = 999;
  std::size_t best_object = -1;
  for (std::size_t i = 0; i < raw_objects->size(); i++)
  {
    pcl::PointCloud<PointT>::Ptr cloud = myclouds[i];
    pcl_tracker_.setInitialPose((*raw_objects)[i].primitive_poses[0]);
    pcl_tracker_.updateWithoutPreprocess(cloud, raw_supports);
    double fr = pcl_tracker_.getFitRatio();
    ROS_INFO_STREAM("Fit ratio: " << fr);

    // Fix for people putting hands on table: compare bounding box sizes
    double bbox_fit = pcl_tracker_.getBboxFit(cloud);
    ROS_INFO_STREAM("Bbox fit: " << bbox_fit);
    fr *= bbox_fit;

    ROS_INFO_STREAM("Adjusted fit ratio: " << fr);
    if (fr < min_fit_ratio)
    {
      min_fit_ratio = fr;
      best_object = i;
    }
  }


  // Look at object
  geometry_msgs::Pose initial_pose = (*raw_objects)[best_object].primitive_poses[0];
  //lookAt(initial_pose.position, base_frame_);
  geometry_msgs::Point p;
  p.x = 1.0;
  p.y = 0.0;
  p.z = 0.55;
  lookAt(p, base_frame_);
  ros::Duration(0.5).sleep();
  waitForNewCloud();

  if (mode_ != "deep")
  {
    pcl_tracker_.setInitialPose(initial_pose);
    last_object_pose_ = initial_pose;
    updateTracker();

    // Update again for better accuracy
    waitForNewCloud();
    updateTracker();
  }

  // Objects that are not the target object are considered obstacles
  ColObjsPtr obstacles(new ColObjs());
  for (std::size_t i = 0; i < raw_objects->size(); i++)
  {
    const geometry_msgs::Pose& p2 = (*raw_objects)[i].primitive_poses[0];
    if (distance(last_object_pose_.position, p2.position) > 0.07)
    {
      geometry_msgs::Pose obs_pose;
      shape_msgs::SolidPrimitive obs_shape;
      PclTools::boundingBox(myclouds[i], obs_pose, obs_shape);
      moveit_msgs::CollisionObject object;
      std::ostringstream s;
      s << "box" << obstacles->size();
      object.id = s.str();
      object.header.frame_id = base_frame_;
      object.operation = moveit_msgs::CollisionObject::ADD;
      object.primitives.push_back(obs_shape);
      object.primitive_poses.push_back(obs_pose);
      obstacles->push_back(object);
    }
  }


  if (mode_ == "kinect_tracking")
    tracking_timer_ = nh_.createTimer(ros::Duration(0.4), &TabletopObjectLocalization::trackingCallback, this);
  is_tracking_ = true;
  return getLocalizationResult(goal, last_object_pose_, old_support_pose_, old_support_box_, obstacles);
}

void TabletopObjectLocalization::kinectDebugLoop()
{
  while (ros::ok())
  {
    pcl::PointCloud<PointT>::Ptr vis_cloud(new pcl::PointCloud<PointT>);
    std::vector<grasp_msgs::Object> objects;
    std::vector<grasp_msgs::Object> supports;
    segmentKinect(objects, supports, vis_cloud);

    ROS_INFO_STREAM("Got " << objects.size() << " objects.");
    for (std::size_t i = 0; i < objects.size(); i++)
    {
      ROS_INFO_STREAM("Object " << i);
      ROS_INFO_STREAM("x: " << objects[i].primitive_poses[0].position.x);
      ROS_INFO_STREAM("y: " << objects[i].primitive_poses[0].position.y);
      ROS_INFO_STREAM("z: " << objects[i].primitive_poses[0].position.z);
    }

    vis_cloud->header.frame_id = base_frame_;
    pcl_conversions::toPCL(ros::Time::now(), vis_cloud->header.stamp);
    cloud_pub_.publish(vis_cloud);
    ros::Duration(1).sleep();
    ros::spinOnce();
  }
}

void TabletopObjectLocalization::kinectOptitrackDebugLoop()
{
  // Open debug file
  std::string package_path = ros::package::getPath("popcorn_vision");
  std::string debug_file_path = package_path + "/debug_output.csv";
  debug_file_.open(debug_file_path.c_str(), std::ofstream::out | std::ofstream::app);
  debug_file_ << "Trial no.,Optitrack x,Optitrack y,Optitrack z, kinect avg x, kinect avg y, kinect avg z";
  int num_kinect_scans = 2;
  for (int i = 0; i < num_kinect_scans; i++)
  {
    debug_file_ << ",kinect scan " << i << " x";
    debug_file_ << ",kinect scan " << i << " y";
    debug_file_ << ",kinect scan " << i << " z";
  }
  debug_file_ << "\n";

  int trial = 0;
  while (ros::ok())
  {
    cout << "Press enter to record, enter q to quit" << endl;
    std::string str;
    getline(cin, str);
    if (str == "q")
      break;
    popcorn_vision::TabletopObjectLocalizationGoalPtr goal(new popcorn_vision::TabletopObjectLocalizationGoal());
    std::vector<std::string> object_names;
    object_names.push_back("pringles");
    std::vector<std::string> support_names;
    support_names.push_back("table");
    goal->object_names = object_names;
    goal->support_names = support_names;

    popcorn_vision::TabletopObjectLocalizationResult optitrack_result, kinect_result;
    optitrack_result = localizeWithOptitrack(goal);
    kinect_result = localizeWithKinect(goal);

    debug_file_ << trial;
    trial++;
    if (optitrack_result.objects.size() > 0)
    {
      debug_file_ << "," << optitrack_result.objects[0].primitive_poses[0].position.x;
      debug_file_ << "," << optitrack_result.objects[0].primitive_poses[0].position.y;
      debug_file_ << "," << optitrack_result.objects[0].primitive_poses[0].position.z;
    }
    if (kinect_result.objects.size() > 0)
    {
      for (std::size_t i = 0; i < kinect_result.objects[0].primitive_poses.size(); i++)
      {
        debug_file_ << "," << kinect_result.objects[0].primitive_poses[i].position.x;
        debug_file_ << "," << kinect_result.objects[0].primitive_poses[i].position.y;
        debug_file_ << "," << kinect_result.objects[0].primitive_poses[i].position.z;
      }
    }
    debug_file_ << "\n";
  }

  debug_file_.close();
}

void TabletopObjectLocalization::kinectTrackingDebugLoop()
{
  // Get initial pose using table segmentation and clustering
  std::vector<grasp_msgs::Object> objects;
  std::vector<grasp_msgs::Object> supports;
  while (ros::ok())
  {
    objects.clear();
    supports.clear();
    segmentKinect(objects, supports);
    if (objects.size() == 1)
      break;
    ros::Duration(0.5).sleep();
  }
  geometry_msgs::Pose initial_pose = objects[0].primitive_poses[0];

  // Get transform between base and camera
  std::string cam_frame = cloud_->header.frame_id;
  tf::StampedTransform cam_tf;
  geometry_msgs::TransformStamped cam_tf2 = g_tf_buffer.lookupTransform(base_frame_, cam_frame, ros::Time(0), ros::Duration(5.0));
  tf::transformStampedMsgToTF(cam_tf2, cam_tf);

  // Determine which object is in the scene
  int object_index = -1;
  double min_fit_ratio = 100000;
  for (std::size_t i = 0; i < object_clouds_.size(); i++)
  {
    pcl_tracker_.setObject(object_clouds_[i]);
    pcl_tracker_.setInitialPose(initial_pose);
    pcl_tracker_.update(cloud_, cam_tf, initial_pose.position);
    double fit_ratio = pcl_tracker_.getFitRatio();
    ROS_INFO_STREAM("Fit ratio for " << object_names_[i] << " " << fit_ratio);
    if (fit_ratio < min_fit_ratio)
    {
      min_fit_ratio = fit_ratio;
      object_index = i;
    }
  }
  pcl_tracker_.setObject(object_clouds_[object_index]);
  pcl_tracker_.setInitialPose(initial_pose);
  pcl::PointCloud<PointT>::Ptr vis_cloud = object_clouds_[object_index];
  ROS_INFO_STREAM("The object is " << object_names_[object_index]);
  ros::Duration(2.0).sleep();

  // Start tracking
  geometry_msgs::PoseStamped pose;
  pose.pose = initial_pose;
  last_object_pose_ = initial_pose;
  current_obj_ = object_index;

  tracking_timer_ = nh_.createTimer(ros::Duration(0.4), &TabletopObjectLocalization::trackingCallback, this);
  is_tracking_ = true;

  ros::spin();
}

void TabletopObjectLocalization::segmentKinect(std::vector<grasp_msgs::Object>& objects,
    std::vector<grasp_msgs::Object>& planes, pcl::PointCloud<PointT>::Ptr vis_cloud)
{
  // Wait for new cloud
  ROS_INFO_STREAM("Waiting for cloud.");
  waitForNewCloud();
  ROS_INFO("Cloud received with %d points.", static_cast<int>(cloud_->points.size()));

  // Filter out noisy long-range points
  pcl::PointCloud<PointT>::Ptr cloud_z = PclTools::zFilter(cloud_, 0, 1.5);
  ROS_INFO("Filtered for range, now %d points.", static_cast<int>(cloud_z->points.size()));

  // Transform into base frame
  std::string cam_frame = cloud_z->header.frame_id;
  ROS_INFO_STREAM("Transforming cloud to frame " << base_frame_ << " from " << cam_frame);
  tf::StampedTransform cam_tf;
  geometry_msgs::TransformStamped cam_tf2 = g_tf_buffer.lookupTransform(base_frame_, cam_frame, ros::Time(0));
  tf::transformStampedMsgToTF(cam_tf2, cam_tf);
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  pcl_ros::transformPointCloud(*cloud_z, *cloud, cam_tf);
  ROS_INFO("Done.");

  // Crop to workspace
  if (!kinect_debug_mode_)
  {
    //cloud = PclTools::zFilter(cloud, 0.3, 99);
    cloud = PclTools::cropBoxFilter(cloud, workspace_min_x_, workspace_min_y_, workspace_min_z_,
        workspace_max_x_, workspace_max_y_, workspace_max_z_);
    ROS_INFO("Cropped to workspace, now %d points.", static_cast<int>(cloud->points.size()));
  }

  // Filtering
  cloud = PclTools::statisticalOutlierFilter(cloud);
  ROS_INFO("Filtered statistical outliers, now %d points", static_cast<int>(cloud->points.size()));
  //cloud = PclTools::radiusOutlierFilter(cloud);
  //ROS_INFO("Filtered radius outliers, now %d points", static_cast<int>(cloud->points.size()));
  cloud = PclTools::voxelGridFilter(cloud);
  ROS_INFO("Downsampled, now %d points", static_cast<int>(cloud->points.size()));

  // Extract planes
  std::vector<pcl::ModelCoefficients::Ptr> plane_coeffs;
  PclTools::segmentPlanes(cloud, planes, plane_coeffs, true, vis_cloud);
  ROS_INFO("Got %d planes", static_cast<int>(planes.size()));

  // Extract objects
  PclTools::segmentObjects(cloud, objects, planes, plane_coeffs, vis_cloud);
  ROS_INFO("Got %d objects", static_cast<int>(objects.size()));
}

void TabletopObjectLocalization::trackingCallback(const ros::TimerEvent& event)
{
  if (!is_tracking_)
    return;
  //ROS_INFO_STREAM("Last tracking callback took " << event.profile.last_duration.toSec() << " seconds");
  ros::Duration d = ros::Time::now() - event.last_real;
  //ROS_INFO_STREAM("Time since last callback " << d.toSec() << " seconds");
  updateTracker();
}

void TabletopObjectLocalization::updateTracker()
{
  track_success_ = false;
  if (mode_ != "deep" && !cloud_)
    return;
  //ROS_INFO_STREAM("Got cloud with " << cloud_->points.size() << " points");

  // Check if object is grasped or not
  bool grasped;
  nh_.param("/grasp_estimator/right_is_grasping", grasped, false);

  // Get transform between base and camera
  std::string cam_frame = cloud_->header.frame_id;
  tf::StampedTransform cam_tf;
  geometry_msgs::TransformStamped cam_tf2 = g_tf_buffer.lookupTransform(base_frame_, cam_frame, ros::Time(0), ros::Duration(5.0));
  tf::transformStampedMsgToTF(cam_tf2, cam_tf);

  // Update tracker
  geometry_msgs::Pose pose;

  if (mode_ == "deep" || mode_ == "deep_kinect_tracking")
  {
    popcorn_vision::GetMarkers srv;
    get_deep_client_.call(srv);
    visualization_msgs::MarkerArray deep_detections = srv.response.markers;
    if (deep_detections.markers.size() == 0)
      return;
    geometry_msgs::PoseStamped deep_pose;
    deep_pose.pose = deep_detections.markers[0].pose;
    tf2::doTransform(deep_pose, deep_pose, cam_tf2);
    geometry_msgs::Point dpp = deep_pose.pose.position;
    ROS_INFO_STREAM("GOT DEEP POSE " << dpp.x << " " << dpp.y << " " << dpp.z);

    if (isnan(dpp.x) || isnan(dpp.y) || isnan(dpp.z))
      return;

    if (dpp.x > workspace_max_x_ || dpp.x < workspace_min_x_ || dpp.y > workspace_max_y_ ||
        dpp.y < workspace_min_y_ || dpp.z > workspace_max_z_ || dpp.z < workspace_min_z_)
      return;

    if (mode_ == "deep")
    {
      pose.position = dpp;
    }
    else if (mode_ == "deep_kinect_tracking")
    {
      // If the CNN detected the object, force position to that (using grasped function)
      pose = pcl_tracker_.updateWhileGrasped(cloud_, cam_tf, dpp);

      // Check the object fits well (lower is better fit)
      double fit_ratio = pcl_tracker_.getFitRatio();
      double fit_ratio_thresh = -60;
      ROS_INFO_STREAM("Fit ratio is " << fit_ratio << " thresh is " << fit_ratio_thresh);
      if (fit_ratio > fit_ratio_thresh)
          return;
    }
  }
  //else if (grasped)
  //{
  //  // If grasped, use end effector pose
  //  geometry_msgs::TransformStamped eef_tf = g_tf_buffer.lookupTransform(base_frame_, eef_frame_, ros::Time(0), ros::Duration(5.0));
  //  geometry_msgs::Point eef;
  //  eef.x = eef_tf.transform.translation.x;
  //  eef.y = eef_tf.transform.translation.y;
  //  eef.z = eef_tf.transform.translation.z;
  //  pose = pcl_tracker_.updateWhileGrasped(cloud_, cam_tf, eef);
  //}
  else
  {
    pose = pcl_tracker_.update(cloud_, cam_tf, last_object_pose_.position);
  }
  track_success_ = true;
  last_object_pose_ = pose;

  // Send object transform
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = base_frame_;
  transformStamped.child_frame_id = object_names_[current_obj_];
  transformStamped.transform.translation.x = pose.position.x;
  transformStamped.transform.translation.y = pose.position.y;
  transformStamped.transform.translation.z = pose.position.z;
  transformStamped.transform.rotation = pose.orientation;
  tf_br_.sendTransform(transformStamped);

  geometry_msgs::PoseStamped ps;
  ps.pose = pose;
  ps.header.frame_id = base_frame_;
  ps.header.stamp = ros::Time::now();
  tracked_pose_pub_.publish(ps);

  // Publish cloud to visualize object
  pcl::PointCloud<PointT>::Ptr vis_cloud = object_clouds_[current_obj_];
  vis_cloud->header.frame_id = object_names_[current_obj_];
  pcl_conversions::toPCL(ros::Time::now(), vis_cloud->header.stamp);
  cloud_pub_.publish(vis_cloud);

  // Update look position if needed
  //if (mode_ == "deep_kinect_tracking" && (grasped || got_deep) && distance(last_look_target_, pose.position) > 0.2)
  //  lookAt(pose.position, base_frame_);
}

void TabletopObjectLocalization::lookAt(const geometry_msgs::Point& target,
    std::string frame, double duration)
{
  last_look_target_ = target;
  point_head_goal_.target.header.stamp = ros::Time::now();
  point_head_goal_.target.header.frame_id = frame;
  point_head_goal_.target.point = target;
  point_head_goal_.min_duration = ros::Duration(duration);
  ROS_INFO("Sending look goal...");
  point_head_client_->sendGoal(point_head_goal_);
  ros::Duration d(10.0);
  bool finished = point_head_client_->waitForResult(d);
  if (finished)
    ROS_INFO("Done.");
  else
    ROS_INFO("Timed out.");
}

// void TabletopObjectLocalization::setAlvarTracking(bool enable)
// {
//   dynamic_reconfigure::ReconfigureRequest srv_req;
//   dynamic_reconfigure::ReconfigureResponse srv_resp;
//   dynamic_reconfigure::BoolParameter param;
//   dynamic_reconfigure::Config conf;

//   param.name = "enabled";
//   param.value = enable;
//   conf.bools.push_back(param);

//   srv_req.config = conf;
//   ros::service::call(alvar_srv_name_, srv_req, srv_resp);
// }

void TabletopObjectLocalization::waitForNewCloud()
{
  pcl::uint64_t time;
  pcl_conversions::toPCL(ros::Time::now(), time);
  while (ros::ok() and (!cloud_ or cloud_->header.stamp < time))
  {
    ros::Duration(0.5).sleep();
    ros::spinOnce();
  }
}

double TabletopObjectLocalization::distance(const geometry_msgs::Point& p0, const geometry_msgs::Point& p1)
{
  return sqrt(pow(p0.x - p1.x, 2) + pow(p0.y - p1.y, 2) + pow(p0.z - p1.z, 2));
}

popcorn_vision::TabletopObjectLocalizationResult TabletopObjectLocalization::getLocalizationResult(
      const popcorn_vision::TabletopObjectLocalizationGoalConstPtr &goal,
      geometry_msgs::Pose object_pose, geometry_msgs::Pose support_pose,
      shape_msgs::SolidPrimitive support_box,
      ColObjsPtr obstacles)
{
  // TODO: currently just taking the first object
  popcorn_vision::TabletopObjectLocalizationResult result;
  std::string object_name = goal->object_names[0];
  string support_name = goal->support_names[0];

  moveit_msgs::CollisionObject object;
  object.header.frame_id = base_frame_;
  object.id = object_name;
  object.type.key = object_name;
  object.type.db = "popcorn_tabletop_objects";
  string object_param_prefix = string(object.type.db+"/"+object.type.key);

  object.primitives.resize(1);
  int object_primitive_type;
  nh_.getParam(string(object_param_prefix+"/primitive_type"), object_primitive_type);
  object.primitives[0].type = object_primitive_type;
  object.primitives[0].dimensions.resize(2);
  nh_.getParam(string(object_param_prefix+"/dimension/height"),
               object.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT]);
  nh_.getParam(string(object_param_prefix+"/dimension/radius"),
               object.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS]);
  object.operation = moveit_msgs::CollisionObject::ADD;

  moveit_msgs::CollisionObject support;
  support.header.frame_id = base_frame_;
  support.id = support_name;
  support.type.key = support_name;
  support.type.db = "popcorn_tabletop_object_supports";
  string support_param_prefix = string(support.type.db+"/"+support.type.key);

  support.primitives.resize(1);
  int support_primitive_type;
  nh_.getParam(string(support_param_prefix+"/primitive_type"), support_primitive_type);
  support.primitives[0].type = support_primitive_type;

  support.primitives[0].dimensions.resize(3);
  /*
  nh_.getParam(string(support_param_prefix+"/dimension/box_x"),
               support.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X]);
  nh_.getParam(string(support_param_prefix+"/dimension/box_y"),
               support.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y]);
  */
  support.primitives[0].dimensions[0] = support_box.dimensions[0];
  support.primitives[0].dimensions[1] = support_box.dimensions[1];
  nh_.getParam(string(support_param_prefix+"/dimension/box_z"),
               support.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z]);
  support.operation = moveit_msgs::CollisionObject::ADD;

  double table_height = support.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z];
  double object_height = object.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT];

  object.primitive_poses.push_back(object_pose);
  support.primitive_poses.push_back(support_pose);

  // Temporarily hard-coding the object height
  
  object.primitive_poses[0].position.z = table_height + object_height / 2;
  support.primitive_poses[0].position.z = table_height / 2;

  result.objects.push_back(object);
  result.supports.push_back(support);

  if (obstacles)
  {
    for (std::size_t i = 0; i < obstacles->size(); i++)
      result.objects.push_back((*obstacles)[i]);
  }

  return result;
}

void TabletopObjectLocalization::loadObjects()
{
  // Load objects from ply files
  ROS_INFO("Loading ply files");
  std::string package_path = ros::package::getPath("popcorn_vision");
  object_names_.push_back("pringles");
  //object_names_.push_back("cup");
  object_names_.push_back("merlo_cup");
  object_names_.push_back("small_pringles");
  object_names_.push_back("candy_box");
  object_names_.push_back("tube");
  for (std::size_t i = 0; i < object_names_.size(); i++)
  {
    std::string ply_path = package_path + "/meshes/" + object_names_[i] + ".ply";
    pcl::PointCloud<PointT>::Ptr obj_cloud(new pcl::PointCloud<PointT>());
    if (!pcl_tracker_.loadPlyFile(ply_path, obj_cloud))
    {
      ROS_ERROR_STREAM("File not found: " << ply_path);
      return;
    }
    ROS_INFO_STREAM(object_names_[i] << " mesh loaded with " << obj_cloud->points.size() << " points");
    object_clouds_.push_back(boost::move(obj_cloud));
  }
  ROS_INFO("Done loading ply files");
}

// void TabletopObjectLocalization::segment(const pcl::PointCloud<PointT>::ConstPtr& cloud,
//                                          const std::vector<string> object_names,
//                                          vector<moveit_msgs::CollisionObject>* localized_objects) {
//   // moveit_msgs::CollisionObject object;
//   // object.id = object_names[0];
//   // localized_objects.push_back(object);

//   // shape_msgs/SolidPrimitive[] primitives
//   // geometry_msgs/Pose[] primitive_poses

//   // shape_msgs/Plane[] planes
//   // geometry_msgs/Pose[] plane_poses
// }

}  // namespace popcorn_vision

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "tabletop_object_localizer");
  ros::NodeHandle n("~");
  tf2_ros::TransformListener tfListener(g_tf_buffer);
  popcorn_vision::TabletopObjectLocalization localizer(n);
  ros::spin();
  return 0;
}
