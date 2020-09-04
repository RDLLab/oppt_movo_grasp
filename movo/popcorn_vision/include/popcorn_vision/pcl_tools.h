#ifndef POPCORN_VISION_PCL_TOOLS_HPP_
#define POPCORN_VISION_PCL_TOOLS_HPP_

#include <vector>
#include <tf/transform_datatypes.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/surface/gp3.h>
//#include <pcl/features/moment_of_inertia_estimation.h
#include <popcorn_vision/moment_of_inertia_estimation.hpp> // Has hack so box only rotates around z
#include <pcl_ros/point_cloud.h>
#include <grasp_msgs/Object.h>
#include <geometry_msgs/Pose.h>
#include <shape_msgs/SolidPrimitive.h>

namespace popcorn_vision {

typedef pcl::PointXYZRGB MyPoint;
typedef pcl::PointXYZRGBNormal MyPointNormal;
typedef pcl::PointCloud<MyPoint> MyCloud;

class PclTools {

public:

  static MyCloud::Ptr zFilter(const MyCloud::ConstPtr input, float min_z,
      float max_z = 9999);

  /**
   * @brief Return cloud with points that are within specified box.
   */
  static MyCloud::Ptr cropBoxFilter(const MyCloud::ConstPtr input,
      float x1, float y1, float z1, float x2, float y2, float z2);

  /**
   * @brief Return cloud with points that have too few neighbours removed.
   * @param input The cloud to filter.
   */
  static MyCloud::Ptr radiusOutlierFilter(const MyCloud::ConstPtr input,
      double radius=0.02, int min_neighbors=5);

  /**
   * @brief Return cloud with statistical outliers removed.
   * @param input The cloud to filter.
   */
  static MyCloud::Ptr statisticalOutlierFilter(const MyCloud::ConstPtr input);

  /**
   * @brief Downsample a point cloud.
   * @param input The cloud to downsample.
   * @param leaf_size The voxel grid leaf size
   */
  static MyCloud::Ptr voxelGridFilter(const MyCloud::ConstPtr input,
      float leaf_size=0.01);

  static MyCloud::Ptr approxVoxelGridFilter(const MyCloud::ConstPtr input,
      float leaf_size=0.01);

  /**
   *  @brief Get supporting surfaces from cloud. Copied from simple_grasping package.
   *  @param cloud The point cloud to segment. This cloud should already be
   *         transformed into a coordinate frame where XY plane is horizontal.
   *         Points forming large planes (e.g. walls, table top) are removed
   *         from the cloud.
   *  @param planes The vector to fill in with support surfaces found.
   *  @param plane_coeffs The vector to fill with plane coefficients.
   *  @param remove_horizontal_planes_only Set to false to remove all large planes
   *  @param vis_cloud Optional cloud to fill with colored points for visualization.
   */
  static void segmentPlanes(MyCloud::Ptr cloud,
      std::vector<grasp_msgs::Object>& planes,
      std::vector<pcl::ModelCoefficients::Ptr>& plane_coeffs,
      bool remove_horizontal_planes_only=true,
      MyCloud::Ptr vis_cloud=MyCloud::Ptr());
  static int planesCount;

  /**
   *  @brief Get objects that are on planes. Copied from simple_grasping package.
   *  @param cloud The cloud from segmentPlanes function.
   *  @param objects The vector to fill in with objects found.
   *  @param plane The plane boxes from segmentPlanes.
   *  @param plane_coeffs The plane coefficients from segmentPlanes.
   *  @param vis_cloud Optional cloud to fill with colored points for visualization.
   */
  static void segmentObjects(MyCloud::Ptr cloud,
      std::vector<grasp_msgs::Object>& objects,
      const std::vector<grasp_msgs::Object>& planes,
      const std::vector<pcl::ModelCoefficients::Ptr>& plane_coeffs,
      MyCloud::Ptr vis_cloud=MyCloud::Ptr());

  static std::vector<MyCloud::Ptr> segmentClusters(const MyCloud::ConstPtr input,
      int min_cluster_size = 50, int max_cluster_size = 1000);

  static void getTableTopClusters(const std::vector<MyCloud::Ptr>& clusters,
      const std::vector<grasp_msgs::Object>& planes,
      const std::vector<pcl::ModelCoefficients::Ptr>& plane_coeffs,
      std::vector<MyCloud::Ptr>& top_clusters,
      std::vector<int>& support_plane_indices);

  static grasp_msgs::Object fitCylinder(const MyCloud::ConstPtr input);

  static void colorCloud(MyCloud::Ptr input, int r, int g, int b);

  static void myGetMinMax(const MyCloud& cloud, MyPoint& min_pt, MyPoint& max_pt);

  static void robustUnorientedBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
    shape_msgs::SolidPrimitive& shape, geometry_msgs::Pose& pose);

  /**
   * @brief Get aligned bounding box.
   * @param input The cloud to get the box for.
   * @param pose The output pose of box center
   * @param shape Contains the output box dimensions
   */
  static void boundingBox(const MyCloud::ConstPtr input, geometry_msgs::Pose& pose,
      shape_msgs::SolidPrimitive& shape);

  static pcl::PolygonMesh::Ptr triangulate(const MyCloud::ConstPtr input);

private:

  /**
   * @brief Helper function that returns vector of values with outliers removed
   */
  static std::vector<int> robustUnorientedBoundingBoxHelper(std::map<int, int>& m);

};

} // end namespace popcorn_vision

#endif // POPCORN_VISION_PCL_TOOLS_HPP_
