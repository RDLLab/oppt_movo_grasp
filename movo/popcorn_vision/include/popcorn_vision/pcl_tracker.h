#ifndef POPCORN_VISION_PCL_TRACKER_H_
#define POPCORN_VISION_PCL_TRACKER_H_

#include <popcorn_vision/pcl_tools.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>

#include <boost/format.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <robot_self_filter/self_see_filter.h>

namespace popcorn_vision
{

typedef pcl::tracking::ParticleXYZRPY ParticleT;
typedef boost::shared_ptr<std::vector<grasp_msgs::Object> > GraspObjectsPtr;

class PclTracker : public pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<MyPoint, ParticleT>
{

public:

  PclTracker(double downsampling_grid_size = 0.01);
  void initRos(ros::NodeHandle& nh);

  /**
   * @brief Identify object
   * @param ref_clouds Reference point clouds of all objects
   * @param obj_pose Estimated pose of the object
   * @param obj_cloud Pre-processed point cloud of object
   * @param obj_index Output index of ref cloud of object. -1 if none matched.
   * @param fit_ratio Output fit ratio. The better the match, the lower this value.
   */
  void identifyObject(
      const std::vector<MyCloud::Ptr>& ref_clouds,
      const geometry_msgs::Pose& obj_pose, MyCloud::ConstPtr obj_cloud,
      std::size_t& obj_index, double& fit_ratio);
  bool setObject(std::string ply_path);
  void setObject(MyCloud::Ptr object_cloud);
  bool loadPlyFile(std::string path, MyCloud::Ptr& cloud);
  void setInitialPose(const geometry_msgs::Pose& pose);

  /**
   * @brief Update the tracker with a new point cloud
   * @param cloud The input point cloud
   * @param trans The transform from the robot base to the camera
   * @param pos Approximate position of the object (cloud is cropped around this)
   * @return Estimated pose of the tracked object
   */
  geometry_msgs::Pose update(MyCloud::ConstPtr cloud, tf::StampedTransform trans,
      const geometry_msgs::Point& pos);

  /**
   * @brief Update the tracker with a new point cloud and end effector pose
   * @param cloud The input point cloud
   * @param trans The transform from the robot base to the camera
   * @param eef Pose of the end effector that is grasping the object
   * @return Estimated pose of the tracked object
   */
  geometry_msgs::Pose updateWhileGrasped(MyCloud::ConstPtr cloud, tf::StampedTransform trans,
      const geometry_msgs::Point& eef);

  /**
   * @brief Update the tracker with a new point cloud that is already pre-processed
   * @param cloud The input point cloud
   * @param planes The support planes
   * @return Estimated pose of the tracked object
   */
  geometry_msgs::Pose updateWithoutPreprocess(MyCloud::ConstPtr cloud,
      GraspObjectsPtr planes = GraspObjectsPtr());

  geometry_msgs::Pose getPose();

  // Override
  double getFitRatio();

  double getBboxFit(MyCloud::ConstPtr cloud);

  void resetCovariance();

protected:
  MyCloud::Ptr preprocessCloud(MyCloud::ConstPtr input, tf::StampedTransform trans,
      const geometry_msgs::Point& pos, double crop_box_size=0.5);

  bool grasped_;
  geometry_msgs::Point eef_;

  // Override
  void resample();
  void weight();

  // Faster implementation
  void myCropInputPointCloud(const PointCloudInConstPtr& cloud, PointCloudIn &output);

  double downsampling_grid_size_;

  MyCloud::Ptr obj_bbox_;
  std::vector<MyCloud::Ptr> transed_bbox_;
  double diag_length_;

  GraspObjectsPtr planes_;

  boost::scoped_ptr<filters::SelfFilter<MyPoint> > self_filter_;

  std::vector<double> default_step_covariance_;

};

} // namespace popcorn_vision

#endif /* POPCORN_VISION_PCL_TRACKER_H_ */
