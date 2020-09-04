#include "popcorn_vision/pcl_tools.h"

#include <map>
#include <algorithm>
#include <iostream>
#include <pcl/common/distances.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <simple_grasping/cloud_tools.h>
#include <simple_grasping/shape_extraction.h>

namespace popcorn_vision
{

int PclTools::planesCount = 0;

MyCloud::Ptr PclTools::zFilter(MyCloud::ConstPtr input, float min_z, float max_z)
{
  pcl::PassThrough<MyPoint> filter;
  filter.setInputCloud(input);
  filter.setFilterFieldName("z");
  filter.setFilterLimits(min_z, max_z);
  MyCloud::Ptr output(new MyCloud());
  filter.filter(*output);
  output->header = input->header;
  return output;
}

MyCloud::Ptr PclTools::cropBoxFilter(const MyCloud::ConstPtr input,
    float x1, float y1, float z1, float x2, float y2, float z2)
{
  pcl::CropBox<MyPoint> filter;
  Eigen::Vector4f min(x1, y1, z1, 1);
  Eigen::Vector4f max(x2, y2, z2, 1);
  filter.setInputCloud(input);
  filter.setMin(min);
  filter.setMax(max);
  MyCloud::Ptr output(new MyCloud());
  filter.filter(*output);
  output->header = input->header;
  return output;
}

MyCloud::Ptr PclTools::radiusOutlierFilter(const MyCloud::ConstPtr input,
    double radius, int min_neighbors)
{
  pcl::RadiusOutlierRemoval<MyPoint> filter;
  filter.setInputCloud(input);
  filter.setRadiusSearch(radius);
  filter.setMinNeighborsInRadius(min_neighbors);
  MyCloud::Ptr output(new MyCloud());
  filter.filter(*output);
  output->header = input->header;
  return output;
}

MyCloud::Ptr PclTools::statisticalOutlierFilter(const MyCloud::ConstPtr input)
{
  pcl::StatisticalOutlierRemoval<MyPoint> filter;
  filter.setInputCloud(input);
  filter.setMeanK(35);
  filter.setStddevMulThresh(0.7);
  MyCloud::Ptr output(new MyCloud());
  filter.filter(*output);
  output->header = input->header;
  return output;
}

MyCloud::Ptr PclTools::voxelGridFilter(const MyCloud::ConstPtr input, float leaf_size)
{
  pcl::VoxelGrid<MyPoint> filter;
  filter.setInputCloud(input);
  filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  MyCloud::Ptr output(new MyCloud());
  filter.filter(*output);
  output->header = input->header;
  return output;
}

MyCloud::Ptr PclTools::approxVoxelGridFilter(const MyCloud::ConstPtr input, float leaf_size)
{
  pcl::ApproximateVoxelGrid<MyPoint> filter;
  filter.setInputCloud(input);
  filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  MyCloud::Ptr output(new MyCloud());
  filter.filter(*output);
  output->header = input->header;
  return output;
}

void PclTools::segmentPlanes(MyCloud::Ptr cloud,
    std::vector<grasp_msgs::Object>& planes,
    std::vector<pcl::ModelCoefficients::Ptr>& plane_coeffs,
    bool remove_horizontal_planes_only,
    MyCloud::Ptr vis_cloud)
{
  // Prepare plane segmenter
  pcl::SACSegmentation<MyPoint> segment;
  segment.setOptimizeCoefficients(true);
  segment.setModelType(pcl::SACMODEL_PLANE);
  segment.setMaxIterations(100);
  segment.setDistanceThreshold(0.01);

  MyCloud::Ptr non_horizontal_planes(new MyCloud);

  int min_plane_size = cloud->points.size()/8;
  while (cloud->points.size() > 500)
  {
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    // Segment the largest planar component from the remaining cloud
    segment.setInputCloud(cloud);
    segment.segment(*inliers, *coefficients);
    if (inliers->indices.size() < (size_t) min_plane_size)
    {
      ROS_DEBUG("No more planes to remove.");
      break;
    }

    // Extract planar part for message
    MyCloud::Ptr plane(new MyCloud());
    pcl::ExtractIndices<MyPoint> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*plane);

    // Check plane is mostly horizontal
    Eigen::Vector3f normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    float angle = acos(Eigen::Vector3f::UnitZ().dot(normal));
    if (angle < 0.15)
    {
      ROS_DEBUG("Removing a plane with %d points.", static_cast<int>(inliers->indices.size()));

      // Run clustering on plane points and take largest cluster
      std::vector<MyCloud::Ptr> plane_clusters = segmentClusters(plane, 0, 99999);
      std::size_t largest = -1;
      std::size_t largest_size = 0;
      for (std::size_t i = 0; i < plane_clusters.size(); i++)
      {
        if (plane_clusters[i]->points.size() > largest_size)
        {
          largest = i;
          largest_size = plane_clusters[i]->points.size();
        }
      }
      plane = plane_clusters[largest];

      // New support object, with cluster, bounding box, and plane
      grasp_msgs::Object object;
      pcl::toROSMsg(*plane, object.point_cluster);
      // give the object a name
      object.name = std::string("surface") + boost::lexical_cast<std::string>(planesCount);
      planesCount++;
      // add shape and pose
      shape_msgs::SolidPrimitive box;
      geometry_msgs::Pose pose;
      //simple_grasping::extractUnorientedBoundingBox(*plane, box, pose);
      robustUnorientedBoundingBox(*plane, box, pose);
      object.primitives.push_back(box);
      object.primitive_poses.push_back(pose);
      // add plane
      for (int i = 0; i < 4; ++i)
        object.surface.coef[i] = coefficients->values[i];
      // stamp and frame
      object.header.stamp = ros::Time::now();
      object.header.frame_id = cloud->header.frame_id;
      // add support surface to object list
      planes.push_back(object);

      // track plane for later use when determining objects
      plane_coeffs.push_back(coefficients);

      if (vis_cloud)
      {
        float hue = (360.0 / 8) * planes.size();
        simple_grasping::colorizeCloud(*plane, hue);
        *vis_cloud += *plane;
      }
    }
    else
    {
      // Add plane to temporary point cloud so we can recover points for object extraction below
      ROS_DEBUG("Plane is not horizontal");
      *non_horizontal_planes += *plane;
    }

    // Extract the non-planar parts and proceed
    extract.setNegative(true);
    extract.filter(*cloud);
  }
  ROS_DEBUG("Cloud now %d points.", static_cast<int>(cloud->points.size()));

  // Add the non-horizontal planes back in
  if (remove_horizontal_planes_only)
    *cloud += *non_horizontal_planes;
}

void PclTools::segmentObjects(MyCloud::Ptr cloud, std::vector<grasp_msgs::Object>& objects,
    const std::vector<grasp_msgs::Object>& planes,
    const std::vector<pcl::ModelCoefficients::Ptr>& plane_coeffs,
    MyCloud::Ptr vis_cloud)
{
  // Cluster
  std::vector<MyCloud::Ptr> temp_clusters = segmentClusters(cloud, 100, 5000);
  ROS_DEBUG("Extracted %d clusters.", static_cast<int>(temp_clusters.size()));
  std::vector<MyCloud::Ptr> clusters;
  std::vector<int> support_plane_indices;
  getTableTopClusters(temp_clusters, planes, plane_coeffs, clusters, support_plane_indices);
  ROS_DEBUG("Got %d clusters on top of table.", static_cast<int>(clusters.size()));

  for (std::size_t i = 0; i < clusters.size(); i++)
  {
    int support_plane_index = support_plane_indices[i];

    grasp_msgs::Object object;
    object.support_surface = planes[support_plane_index].name;
    shape_msgs::SolidPrimitive shape;
    geometry_msgs::Pose pose;
    pcl::PointCloud<pcl::PointXYZRGB> projected_cloud;
    simple_grasping::extractShape(*clusters[i], plane_coeffs[support_plane_index], projected_cloud, shape, pose);

    // Minimum height requirement
    double height = 10.0;
    if (shape.type == shape.CYLINDER)
      height = shape.dimensions[0];
    else if (shape.type == shape.BOX)
      height = shape.dimensions[2];
    if (height < 0.03)
      continue;

    //pcl::toROSMsg(projected_cloud, object.point_cluster);
    pcl::toROSMsg(*(clusters[i]), object.point_cluster);
    object.primitives.push_back(shape);
    object.primitive_poses.push_back(pose);
    // add stamp and frame
    object.header.stamp = ros::Time::now();
    object.header.frame_id = cloud->header.frame_id;
    // add object to object list
    objects.push_back(object);

    if (vis_cloud)
    {
      float hue = (360.0 / clusters.size()) * i;
      simple_grasping::colorizeCloud(*clusters[i], hue);
      *vis_cloud += *clusters[i];
    }
  }

  ROS_INFO("object support segmentation done processing.");
}

std::vector<MyCloud::Ptr> PclTools::segmentClusters(
    const MyCloud::ConstPtr input, int min_cluster_size,
    int max_cluster_size)
{

  /*
   // Compute normals
   pcl::NormalEstimation<MyPoint, pcl::Normal> ne;
   ne.setInputCloud(point_input);
   pcl::search::KdTree<MyPoint>::Ptr tree(new pcl::search::KdTree<MyPoint>());
   ne.setSearchMethod(tree);
   ne.setRadiusSearch(0.03);
   pcl::PointCloud<pcl::Normal> normals;
   ne.compute(normals);

   // Merge normals with xyz data
   pcl::PointCloud<pcl::PointNormal>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointNormal());
   pcl::concatenateFields(*point_input, normals, *pcl_cloud);
   */

  pcl::search::KdTree<MyPoint>::Ptr tree(
      new pcl::search::KdTree<MyPoint>);
  tree->setInputCloud(input);

  std::vector<pcl::PointIndices> indices;
  pcl::EuclideanClusterExtraction<MyPoint> ec;
  ec.setClusterTolerance(0.02); // 2cm
  ec.setMinClusterSize(min_cluster_size);
  ec.setMaxClusterSize(max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(input);
  ec.extract(indices);

  std::vector<MyCloud::Ptr> clusters;
  for (std::vector<pcl::PointIndices>::const_iterator it = indices.begin(); it != indices.end(); ++it)
  {
    MyCloud::Ptr cluster(new MyCloud());
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cluster->points.push_back(input->points[*pit]);
    cluster->header = input->header;
    clusters.push_back(cluster);
  }
  return clusters;
}

void PclTools::getTableTopClusters(const std::vector<MyCloud::Ptr>& clusters,
      const std::vector<grasp_msgs::Object>& planes,
      const std::vector<pcl::ModelCoefficients::Ptr>& plane_coeffs,
      std::vector<MyCloud::Ptr>& top_clusters,
      std::vector<int>& support_plane_indices)
{
  // Fix for situation where e.g. a cup is split into two clusters
  std::vector<std::size_t> top_cluster_ind;
  std::vector<std::size_t> small_floating_clusters;
  float max_dist_to_table = 0.025;
  float max_join_dist = 0.1;
  float max_size = 1000;

  // First calculate lowest point for each cluster
  std::vector<MyPoint> low_points;
  for (std::size_t i = 0; i < clusters.size(); i++)
  {
    MyPoint pmin, pmax, p;
    myGetMinMax(*clusters[i], pmin, pmax);
    p.x = (pmin.x + pmax.x) / 2;
    p.y = (pmin.y + pmax.y) / 2;
    p.z = pmin.z;
    low_points.push_back(p);
  }

  for (std::size_t i = 0; i < clusters.size(); i++)
  {
    // Find supporting plane for this cluster
    int support_plane_index = -1;
    double support_plane_distance = 1000.0;
    for (int p = 0; p < plane_coeffs.size(); ++p)
    {
      // Ensure object is within plane bounds
      const shape_msgs::SolidPrimitive& box = planes[p].primitives[0];
      const geometry_msgs::Pose& plane_pose = planes[p].primitive_poses[0];
      double min_x = plane_pose.position.x - 0.5 * box.dimensions[0];
      double max_x = plane_pose.position.x + 0.5 * box.dimensions[0];
      double min_y = plane_pose.position.y - 0.5 * box.dimensions[1];
      double max_y = plane_pose.position.y + 0.5 * box.dimensions[1];
      if (low_points[i].x < min_x || low_points[i].x > max_x || low_points[i].y < min_y || low_points[i].y > max_y)
        continue;

      // Get distance to the plane centroid
      double distance = simple_grasping::distancePointToPlane(low_points[i], plane_coeffs[p]);
      if (distance > -0.02 && distance < support_plane_distance)
      {
        support_plane_distance = distance;
        support_plane_index = p;
      }
    }

    if (support_plane_index != -1)
    {
      if (support_plane_distance < max_dist_to_table)
      {
        top_clusters.push_back(clusters[i]);
        top_cluster_ind.push_back(i);
        support_plane_indices.push_back(support_plane_index);
      }
      else
        small_floating_clusters.push_back(i);
    }
  }

  for (std::size_t i = 0; i < small_floating_clusters.size(); i++)
  {
    MyCloud::Ptr sfc = clusters[small_floating_clusters[i]];
    MyPoint& sfc_low_point = low_points[small_floating_clusters[i]];
    int closest = -1;
    for (std::size_t j = 0; j < top_clusters.size(); j++)
    {
      MyPoint& tc_low_point = low_points[top_cluster_ind[j]];
      if (sfc->points.size() + top_clusters[j]->points.size() < max_size &&
          pcl::euclideanDistance(sfc_low_point, tc_low_point) < max_join_dist)
        closest = j;
    }
    if (closest >= 0)
      *(top_clusters[closest]) += *sfc;
  }
}

grasp_msgs::Object PclTools::fitCylinder(const MyCloud::ConstPtr input)
{
  //
  pcl::PointCloud<pcl::Normal>::Ptr cluster_normals( new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<MyPoint, pcl::Normal> cluster_ne;
  pcl::search::KdTree<MyPoint>::Ptr cluster_tree( new pcl::search::KdTree<MyPoint>());
  cluster_ne.setSearchMethod(cluster_tree);
  cluster_ne.setInputCloud(input);
  cluster_ne.setKSearch(100);
  cluster_ne.compute(*cluster_normals);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);

  pcl::SACSegmentationFromNormals<MyPoint, pcl::Normal> seg;
  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setOptimizeCoefficients(true);
  seg.setNormalDistanceWeight(0.1);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.05);
  seg.setRadiusLimits(0.02, 0.1);
  seg.setInputCloud(input);
  seg.setInputNormals(cluster_normals);
  seg.segment(*inliers_cylinder, *coefficients_cylinder);
  // std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // Get bounding box to determine cylinder height
  shape_msgs::SolidPrimitive bbox;
  geometry_msgs::Pose bbox_pose;
  simple_grasping::extractUnorientedBoundingBox(*input, bbox, bbox_pose);

  geometry_msgs::Pose pose;
  pose.position.x = coefficients_cylinder->values[0];
  pose.position.y = coefficients_cylinder->values[1];
  pose.position.z = coefficients_cylinder->values[2];
  double axis_dir_x = coefficients_cylinder->values[3];
  double axis_dir_y = coefficients_cylinder->values[4];
  double axis_dir_z = coefficients_cylinder->values[5];
  double axis_dir_r = sqrt(pow(axis_dir_x, 2) + pow(axis_dir_y, 2) + pow(axis_dir_z, 2));
  double axis_dir_yaw = atan2(axis_dir_y, axis_dir_x);
  double axis_dir_pitch = acos(axis_dir_z / axis_dir_r);
  tf::Quaternion q;
  q.setRPY(0, axis_dir_pitch, axis_dir_yaw);
  tf::quaternionTFToMsg(q, pose.orientation);

  grasp_msgs::Object obj;
  obj.primitive_poses.push_back(pose);
  shape_msgs::SolidPrimitive prim;
  prim.type = prim.CYLINDER;
  prim.dimensions.push_back(bbox.dimensions[2]); // height
  prim.dimensions.push_back(coefficients_cylinder->values[6]); //radius
  obj.primitives.push_back(prim);
  return obj;
}

void PclTools::colorCloud(MyCloud::Ptr input, int r, int g, int b)
{
  for (std::size_t i = 0; i < input->points.size(); i++)
  {
    input->points[i].r = r;
    input->points[i].g = g;
    input->points[i].b = b;
  }
}

void PclTools::myGetMinMax(const MyCloud& cloud, MyPoint& min_pt, MyPoint& max_pt)
{
  double min_x = 999, min_y = 999, min_z = 999, max_x = -999, max_y = -999, max_z = -999;
  for (std::size_t i = 0; i < cloud.points.size (); ++i)
  {
    min_x = fmin(min_x, cloud.points[i].x);
    min_y = fmin(min_y, cloud.points[i].y);
    min_z = fmin(min_z, cloud.points[i].z);
    max_x = fmax(min_x, cloud.points[i].x);
    max_y = fmax(min_y, cloud.points[i].y);
    max_z = fmax(min_z, cloud.points[i].z);
  }
  min_pt.x = min_x;
  min_pt.y = min_y;
  min_pt.z = min_z;
  max_pt.x = max_x;
  max_pt.y = max_y;
  max_pt.z = max_z;
}

void PclTools::robustUnorientedBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
    shape_msgs::SolidPrimitive& shape, geometry_msgs::Pose& pose)
{
  double min_x = 999, min_y = 999, min_z = 999, max_x = -999, max_y = -999, max_z = -999;
  double grid_size = 0.01;
  std::map<int, int> max_xs, min_xs, max_ys, min_ys;
  for (std::size_t i = 0; i < cloud.points.size (); ++i)
  {
    int grid_x = int(cloud.points[i].x / grid_size);
    int grid_y = int(cloud.points[i].y / grid_size);
    if (max_xs.count(grid_y) > 0)
      max_xs[grid_y] = std::max(max_xs[grid_y], grid_x);
    else
      max_xs[grid_y] = grid_x;
    if (min_xs.count(grid_y) > 0)
      min_xs[grid_y] = std::min(min_xs[grid_y], grid_x);
    else
      min_xs[grid_y] = grid_x;
    if (max_ys.count(grid_x) > 0)
      max_ys[grid_x] = std::max(max_ys[grid_x], grid_y);
    else
      max_ys[grid_x] = grid_y;
    if (min_ys.count(grid_x) > 0)
      min_ys[grid_x] = std::min(min_ys[grid_x], grid_y);
    else
      min_ys[grid_x] = grid_y;

    // "Non-robust" for z
    min_z = fmin(min_z, cloud.points[i].z);
    max_z = fmax(min_z, cloud.points[i].z);
  }

  // Remove outliers
  std::vector<int> max_xs2 = robustUnorientedBoundingBoxHelper(max_xs);
  std::vector<int> min_xs2 = robustUnorientedBoundingBoxHelper(min_xs);
  std::vector<int> max_ys2 = robustUnorientedBoundingBoxHelper(max_ys);
  std::vector<int> min_ys2 = robustUnorientedBoundingBoxHelper(min_ys);

  // Calculate max and min
  int max_xi = -999, min_xi = 999, max_yi = -999, min_yi = 999;
  for (std::size_t i = 0; i < max_xs2.size(); i++)
    max_xi = std::max(max_xi, max_xs2[i]);
  for (std::size_t i = 0; i < min_xs2.size(); i++)
    min_xi = std::min(min_xi, min_xs2[i]);
  for (std::size_t i = 0; i < max_ys2.size(); i++)
    max_yi = std::max(max_yi, max_ys2[i]);
  for (std::size_t i = 0; i < min_ys2.size(); i++)
    min_yi = std::min(min_yi, min_ys2[i]);
  max_x = max_xi * grid_size;
  min_x = min_xi * grid_size;
  max_y = max_yi * grid_size;
  min_y = min_yi * grid_size;

  // Output
  pose.position.x = (min_x + max_x)/2.0;
  pose.position.y = (min_y + max_y)/2.0;
  pose.position.z = (min_z + max_z)/2.0;

  shape.type = shape.BOX;
  shape.dimensions.push_back(max_x-min_x);
  shape.dimensions.push_back(max_y-min_y);
  shape.dimensions.push_back(max_z-min_z);
}

std::vector<int> PclTools::robustUnorientedBoundingBoxHelper(std::map<int, int>& m)
{
  double mean = 0;
  for(std::map<int, int>::iterator it = m.begin(); it != m.end(); it++)
    mean += it->second;
  mean /= m.size();
  double var = 0;
  for(std::map<int, int>::iterator it = m.begin(); it != m.end(); it++)
    var += (it->second - mean) * (it->second - mean);
  var /= m.size();
  double std_dev = sqrt(var);
  std::vector<int> result;
  for(std::map<int, int>::iterator it = m.begin(); it != m.end(); it++)
  {
    if (std::abs(it->second - mean) < std_dev)
      result.push_back(it->second);
  }
  return result;
}

void PclTools::boundingBox(const MyCloud::ConstPtr input, geometry_msgs::Pose& pose,
      shape_msgs::SolidPrimitive& shape)
{
  pcl::MyMomentOfInertiaEstimation<MyPoint> feature_extractor;
  feature_extractor.setInputCloud(input);
  feature_extractor.compute();
  MyPoint min_point;
  MyPoint max_point;
  MyPoint position;
  Eigen::Matrix3f rotational_matrix;
  feature_extractor.getOBB(min_point, max_point, position, rotational_matrix);
  Eigen::Quaternionf quat(rotational_matrix);

  pose.position.x = position.x;
  pose.position.y = position.y;
  pose.position.z = position.z;
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();

  shape.type = shape_msgs::SolidPrimitive::BOX;
  shape.dimensions.resize(3);
  shape.dimensions[shape_msgs::SolidPrimitive::BOX_X] = max_point.x - min_point.x;
  shape.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = max_point.y - min_point.y;
  shape.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = max_point.z - min_point.z;
}

pcl::PolygonMesh::Ptr PclTools::triangulate(const MyCloud::ConstPtr input)
{
  // Normal estimation*
  pcl::NormalEstimation<MyPoint, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<MyPoint>::Ptr tree (new pcl::search::KdTree<MyPoint>);
  tree->setInputCloud (input);
  n.setInputCloud (input);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZRGB and normal fields
  pcl::PointCloud<MyPointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<MyPointNormal>);
  pcl::concatenateFields (*input, *normals, *cloud_with_normals);

  // Create search tree*
  pcl::search::KdTree<MyPointNormal>::Ptr tree2 (new pcl::search::KdTree<MyPointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<MyPointNormal> gp3;
  pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh());

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.05);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (*triangles);
  return triangles;
}

} // end namespace popcorn_vision

