#include <popcorn_vision/pcl_tracker.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>

using namespace pcl;
using namespace pcl::tracking;

namespace popcorn_vision
{

PclTracker::PclTracker(double downsampling_grid_size) :
    KLDAdaptiveParticleFilterOMPTracker<MyPoint, ParticleT>(4), // 4 threads
    downsampling_grid_size_(downsampling_grid_size),
    grasped_(false), diag_length_(0.0)
{

  // Variance (standard deviation squared) for gaussian sampling
  default_step_covariance_ = std::vector<double>(6, 0.025 * 0.025);
  //default_step_covariance_[3] *= 40.0;
  //default_step_covariance_[4] *= 40.0;
  default_step_covariance_[3] = 0.0;
  default_step_covariance_[4] = 0.0;
  default_step_covariance_[5] *= 40.0;

  std::vector<double> initial_noise_covariance = std::vector<double>(6, 0.00001);

  initial_noise_covariance[3] = 0.0;
  initial_noise_covariance[4] = 0.0;

  std::vector<double> default_initial_mean = std::vector<double>(6, 0.0);

  ParticleT bin_size;
  bin_size.x = 0.04f;
  bin_size.y = 0.04f;
  bin_size.z = 0.04f;
  bin_size.roll = 0.2f;
  bin_size.pitch = 0.2f;
  bin_size.yaw = 0.2f;

  //Set all parameters for KLDAdaptiveParticleFilterOMPTracker
  setMaximumParticleNum(1000);
  setDelta(0.99);
  setEpsilon(0.2);
  setBinSize(bin_size);

  //Set all parameters for ParticleFilterTracker
  setTrans(Eigen::Affine3f::Identity());
  setStepNoiseCovariance(default_step_covariance_);
  setInitialNoiseCovariance(initial_noise_covariance);
  setInitialNoiseMean(default_initial_mean);
  setIterationNum(1);
  setParticleNum(600);
  setResampleLikelihoodThr(0.00);
  setUseNormal(false);

  //Setup coherence object for tracking
  ApproxNearestPairPointCloudCoherence<MyPoint>::Ptr coherence =
      ApproxNearestPairPointCloudCoherence < MyPoint >::Ptr(new ApproxNearestPairPointCloudCoherence<MyPoint>());

  boost::shared_ptr<DistanceCoherence<MyPoint> > distance_coherence =
      boost::shared_ptr<DistanceCoherence<MyPoint> >(new DistanceCoherence<MyPoint>());
  coherence->addPointCoherence(distance_coherence);

  //boost::shared_ptr<HSVColorCoherence<MyPoint> > color_coherence =
  //    boost::shared_ptr<HSVColorCoherence<MyPoint> >(new HSVColorCoherence<MyPoint>());
  //coherence->addPointCoherence(color_coherence);

  boost::shared_ptr<pcl::search::Octree<MyPoint> > search(new pcl::search::Octree<MyPoint>(0.01));
  coherence->setSearchMethod(search);
  coherence->setMaximumDistance(0.01);

  setCloudCoherence(coherence);

  // One time allocation for bounding box clouds
  transed_bbox_.resize(maximum_particle_number_);
  for (int i = 0; i < maximum_particle_number_; i++)
    transed_bbox_[i] = MyCloud::Ptr(new MyCloud());
}

void PclTracker::initRos(ros::NodeHandle& nh)
{
  self_filter_.reset(new filters::SelfFilter<MyPoint>(nh));
}

void PclTracker::identifyObject(
      const std::vector<MyCloud::Ptr>& ref_clouds,
      const geometry_msgs::Pose& obj_pose, MyCloud::ConstPtr obj_cloud,
      std::size_t& obj_index, double& fit_ratio)
{
  double fit_ratio_thresh = -80;
  double min_fit_ratio = 100000;

  for (std::size_t i = 0; i < ref_clouds.size(); i++)
  {
    setObject(ref_clouds[i]);
    setInitialPose(obj_pose);
    setInputCloud(obj_cloud);
    compute();
    double fr = getFitRatio();
    if (fr < min_fit_ratio)
    {
      min_fit_ratio = fr;
      if (fr < fit_ratio_thresh)
      {
        obj_index = i;
        fit_ratio = fr;
      }
    }
  }
}

bool PclTracker::setObject(std::string ply_path)
{
  // Load ply file
  MyCloud::Ptr cloud(new MyCloud());
  if (!loadPlyFile(ply_path, cloud))
    return false;

  setObject(cloud);
  return true;
}

void PclTracker::setObject(MyCloud::Ptr object_cloud)
{
  setReferenceCloud(object_cloud);

  // Get a bounding box with 8 points for faster calculations
  MyPoint p, pmin, pmax;
  PclTools::myGetMinMax(*object_cloud, pmin, pmax);
  obj_bbox_.reset(new MyCloud());
  p = pmin;     // (minx, miny, minz)
  obj_bbox_->push_back(p);
  p.x = pmax.x; // (maxx, miny, minz)
  obj_bbox_->push_back(p);
  p.y = pmax.y; // (maxx, maxy, minz)
  obj_bbox_->push_back(p);
  p.x = pmin.x; // (minx, maxy, minz)
  obj_bbox_->push_back(p);
  p.z = pmax.z; // (minx, maxy, maxz)
  obj_bbox_->push_back(p);
  p.x = pmax.x; // (maxx, maxy, maxz)
  obj_bbox_->push_back(p);
  p.y = pmin.y; // (maxx, miny, maxz)
  obj_bbox_->push_back(p);
  p.x = pmin.x; // (minx, miny, maxz)
  obj_bbox_->push_back(p);

  double dx = pmax.x - pmin.x;
  double dy = pmax.y - pmin.y;
  double dz = pmax.z - pmin.z;
  diag_length_ = sqrt(dx * dx + dy * dy + dz * dz);
}

bool PclTracker::loadPlyFile(std::string path, MyCloud::Ptr& cloud)
{
  bool success = pcl::io::loadPLYFile(path, *cloud) >= 0;
  if (!success)
    return false;

  // Downsample object cloud
  cloud = PclTools::voxelGridFilter(cloud, downsampling_grid_size_);
  return success;

}

void PclTracker::setInitialPose(const geometry_msgs::Pose& pose)
{
  Eigen::Affine3d a3d;
  tf::poseMsgToEigen(pose, a3d);
  Eigen::Affine3f a3f = a3d.cast<float>();
  setTrans(a3f);
  resetTracking();
}

geometry_msgs::Pose PclTracker::update(MyCloud::ConstPtr cloud,
    tf::StampedTransform trans, const geometry_msgs::Point& pos)
{
  MyCloud::Ptr pre = preprocessCloud(cloud, trans, pos);
  if (pre->points.size() == 0)
  {
    //std::cout << "Tracker got empty cloud" << std::endl;;
    return getPose();
  }
  setInputCloud(pre);
  compute();
  return getPose();
}

geometry_msgs::Pose PclTracker::updateWhileGrasped(MyCloud::ConstPtr cloud,
    tf::StampedTransform trans, const geometry_msgs::Point& eef)
{
  eef_ = eef;
  MyCloud::Ptr pre = preprocessCloud(cloud, trans, eef, 0.2);
  if (pre->points.size() == 0)
  {
    //std::cout << "Tracker got empty cloud" << std::endl;;
    return getPose();
  }
  grasped_ = true;
  setInputCloud(pre);
  compute();
  grasped_ = false;
  return getPose();
}

geometry_msgs::Pose PclTracker::updateWithoutPreprocess(MyCloud::ConstPtr cloud,
    GraspObjectsPtr planes)
{
  planes_ = planes;
  setInputCloud(cloud);
  compute();
  return getPose();
}

geometry_msgs::Pose PclTracker::getPose()
{
  ParticleT result = getResult();
  Eigen::Affine3f a3f = toEigenMatrix(result);
  Eigen::Affine3d a3d = a3f.cast<double>();
  geometry_msgs::Pose pose;
  tf::poseEigenToMsg(a3d, pose);
  return pose;
}

double PclTracker::getFitRatio()
{
  ParticleT result = getResult();
  MyCloud::ConstPtr ref = getReferenceCloud();
  MyCloud::Ptr transed(new MyCloud());
  Eigen::Affine3f trans = toEigenMatrix(result);
  pcl::transformPointCloud<MyPoint>(*ref, *transed, trans);
  float weight;
  pcl::IndicesPtr indices;
  getCloudCoherence()->compute(transed, indices, weight);
  return weight;
}

double PclTracker::getBboxFit(MyCloud::ConstPtr cloud)
{
  MyPoint pmin, pmax;
  PclTools::myGetMinMax(*cloud, pmin, pmax);

  double dx = pmax.x - pmin.x;
  double dy = pmax.y - pmin.y;
  double dz = pmax.z - pmin.z;
  double diag = sqrt(dx * dx + dy * dy + dz * dz);

  if (diag == 0) // Shouldn't happen
    return 0;
  else if (diag > diag_length_)
    return diag_length_ / diag;
  else
    return diag / diag_length_;
}

void PclTracker::resetCovariance()
{
  setStepNoiseCovariance(default_step_covariance_);
}

MyCloud::Ptr PclTracker::preprocessCloud(MyCloud::ConstPtr input, tf::StampedTransform trans,
    const geometry_msgs::Point& pos, double crop_box_size)
{
  std::clock_t start;
  start = std::clock();
  MyCloud::Ptr cloud_z = PclTools::zFilter(input, 0, 1.5);
  //std::cout << "Z filter time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;

  // Transform into base frame
  start = std::clock();
  MyCloud::Ptr cloud(new MyCloud());
  pcl_ros::transformPointCloud(*cloud_z, *cloud, trans);
  //std::cout << "Transform time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;

  // Crop around estimated object pose
  double cb = crop_box_size * 0.5;
  start = std::clock();
  cloud = PclTools::cropBoxFilter(cloud, pos.x - cb, pos.y - cb,
      pos.z - cb, pos.x + cb, pos.y + cb, pos.z + cb);
  //std::cout << "Crop time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;

  if (cloud->points.size() == 0)
    return cloud;

  // Filtering
  start = std::clock();
  cloud = PclTools::statisticalOutlierFilter(cloud);
  //cloud = PclTools::voxelGridFilter(cloud);
  cloud = PclTools::approxVoxelGridFilter(cloud, downsampling_grid_size_);
  //std::cout << "Filtering time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;

  // Remove large planes
  if (!grasped_)
  {
    start = std::clock();
    planes_.reset(new std::vector<grasp_msgs::Object>());
    std::vector<pcl::ModelCoefficients::Ptr> plane_coeffs;
    PclTools::segmentPlanes(cloud, *planes_, plane_coeffs, false);
    //std::cout << "Plane segment time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
  }

  // Robot self filtering to remove robot arm from point cloud
  start = std::clock();
  MyCloud::Ptr output(new MyCloud());
  self_filter_->update(*cloud, *output);
  //std::cout << "Self filtering time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;

  return output;
}

void PclTracker::resample()
{
  unsigned int k = 0;
  unsigned int n = 0;
  PointCloudStatePtr S(new PointCloudState);
  std::vector<std::vector<int> > B; // bins

  // initializing for sampling without replacement
  std::vector<int> a(particles_->points.size());
  std::vector<double> q(particles_->points.size());
  genAliasTable(a, q, particles_);

  const std::vector<double> zero_mean(ParticleT::stateDimension(), 0.0);

  // select the particles with KLD sampling
  do
  {
    int j_n = sampleWithReplacement(a, q);
    ParticleT x_t = particles_->points[j_n];

    // Force position if grasped
    if (grasped_)
    {
      x_t.x = eef_.x;
      x_t.y = eef_.y;
      x_t.z = eef_.z;
    }

    x_t.sample(zero_mean, step_noise_covariance_);

    // motion
    if (rand() / double(RAND_MAX) < motion_ratio_)
      x_t = x_t + motion_;

    // Transform the object bounding box cloud and get the lowest point in it
    MyPoint pmin, pmax;
    Eigen::Affine3f trans = toEigenMatrix(x_t);
    pcl::transformPointCloud<MyPoint>(*obj_bbox_, *(transed_bbox_[S->points.size()]), trans);
    PclTools::myGetMinMax(*(transed_bbox_[S->points.size()]), pmin, pmax);

    // Ensure transformed point cloud rests on support surface (e.g. table)
    float z_shift = 0.f;
    if (planes_ && !grasped_) {
      for (std::size_t i = 0; i < planes_->size(); i++)
      {
        // Check if this is the correct support plane by checking bounds
        const grasp_msgs::Object& plane = (*planes_)[i];
        const shape_msgs::SolidPrimitive& box = plane.primitives[0];
        const geometry_msgs::Pose& plane_pose = plane.primitive_poses[0];
        double min_x = plane_pose.position.x - 0.5 * box.dimensions[0];
        double max_x = plane_pose.position.x + 0.5 * box.dimensions[0];
        double min_y = plane_pose.position.y - 0.5 * box.dimensions[1];
        double max_y = plane_pose.position.y + 0.5 * box.dimensions[1];
        if (x_t[0] < min_x || x_t[0] > max_x || x_t[1] < min_y || x_t[1] > max_y)
          continue;

        // Ensure object is not floating above or insersecting with table
        // Calculate distance from point to plane
        Eigen::Vector4f pp(pmin.x, pmin.y, pmin.z, 1);
        Eigen::Vector4f m(plane.surface.coef[0], plane.surface.coef[1], plane.surface.coef[2], plane.surface.coef[3]);
        float distance_to_plane = pp.dot(m);
        if (fabs(distance_to_plane) > 0.02)
          z_shift = -distance_to_plane;
        break;
      }
      x_t.z += z_shift;
    }

    S->points.push_back(x_t);
    // calc bin
    std::vector<int> bin(ParticleT::stateDimension());
    for (int i = 0; i < ParticleT::stateDimension(); i++)
      bin[i] = static_cast<int>(x_t[i] / bin_size_[i]);

    // calc bin index... how?
    if (insertIntoBins(bin, B))
      ++k;
    ++n;
  } while (n < maximum_particle_number_ && (k < 2 || n < calcKLBound(k)));

  particles_ = S;               // swap
  particle_num_ = static_cast<int>(particles_->points.size());
}

void PclTracker::weight()
{
  if (!use_normal_)
  {
#ifdef _OPENMP
#pragma omp parallel for num_threads(threads_)
#endif
    for (int i = 0; i < particle_num_; i++)
    {
      this->computeTransformedPointCloudWithoutNormal(particles_->points[i], *transed_reference_vector_[i]);
      if (!changed_) // If point cloud changed bbox are already calculated in resample function
      {
        // Transform the object bounding box cloud and get the lowest point in it
        MyPoint pmin, pmax;
        ParticleT x_t = particles_->points[i];
        Eigen::Affine3f trans = toEigenMatrix(x_t);
        pcl::transformPointCloud<MyPoint>(*obj_bbox_, *(transed_bbox_[i]), trans);
        PclTools::myGetMinMax(*(transed_bbox_[i]), pmin, pmax);
      }
    }

    PointCloudInPtr coherence_input (new PointCloudIn);
    myCropInputPointCloud (input_, *coherence_input);
    if (change_counter_ == 0)
    {
      // test change detector
      if (!use_change_detector_ || this->testChangeDetection (coherence_input))
      {
        changed_ = true;
        change_counter_ = change_detector_interval_;
        coherence_->setTargetCloud (coherence_input);
        coherence_->initCompute ();
#ifdef _OPENMP
#pragma omp parallel for num_threads(threads_)
#endif
        for (int i = 0; i < particle_num_; i++)
        {
          IndicesPtr indices;
          coherence_->compute (transed_reference_vector_[i], indices, particles_->points[i].weight);
        }
      }
      else
        changed_ = false;
    }
    else
    {
      --change_counter_;
      coherence_->setTargetCloud (coherence_input);
      coherence_->initCompute ();
#ifdef _OPENMP
#pragma omp parallel for num_threads(threads_)
#endif
      for (int i = 0; i < particle_num_; i++)
      {
        IndicesPtr indices;
        coherence_->compute (transed_reference_vector_[i], indices, particles_->points[i].weight);
      }
    }
  }
  else
  {
    std::vector<IndicesPtr> indices_list (particle_num_);
    for (int i = 0; i < particle_num_; i++)
    {
      indices_list[i] = IndicesPtr (new std::vector<int>);
    }
#ifdef _OPENMP
#pragma omp parallel for num_threads(threads_)
#endif
    for (int i = 0; i < particle_num_; i++)
    {
      this->computeTransformedPointCloudWithNormal (particles_->points[i], *indices_list[i], *transed_reference_vector_[i]);
    }

    PointCloudInPtr coherence_input (new PointCloudIn);
    myCropInputPointCloud (input_, *coherence_input);

    coherence_->setTargetCloud (coherence_input);
    coherence_->initCompute ();
#ifdef _OPENMP
#pragma omp parallel for num_threads(threads_)
#endif
    for (int i = 0; i < particle_num_; i++)
    {
      coherence_->compute (transed_reference_vector_[i], indices_list[i], particles_->points[i].weight);
    }
  }

  normalizeWeight ();
}

void PclTracker::myCropInputPointCloud(const PointCloudInConstPtr& cloud, PointCloudIn &output)
{
  double x_min, y_min, z_min, x_max, y_max, z_max;
  x_min = y_min = z_min = std::numeric_limits<double>::max();
  x_max = y_max = z_max = -std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < transed_bbox_.size (); i++)
  {
    MyCloud::Ptr target = transed_bbox_[i];
    MyPoint pmin, pmax;
    PclTools::myGetMinMax(*target, pmin, pmax);
    x_min = fmin(x_min, pmin.x);
    y_min = fmin(y_min, pmin.y);
    z_min = fmin(z_min, pmin.z);
    x_max = fmax(x_max, pmax.x);
    y_max = fmax(y_max, pmax.y);
    z_max = fmax(z_max, pmax.z);
  }
  pcl::CropBox<MyPoint> filter;
  Eigen::Vector4f min(x_min - 0.01, y_min - 0.01, z_min - 0.01, 1);
  Eigen::Vector4f max(x_max + 0.01, y_max + 0.01, z_max + 0.01, 1);
  filter.setInputCloud(cloud);
  filter.setMin(min);
  filter.setMax(max);
  filter.filter(output);
}

} // namespace popcorn_vision
