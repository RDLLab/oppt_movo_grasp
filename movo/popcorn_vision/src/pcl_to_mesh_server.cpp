#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <popcorn_vision/pcl_tools.h>
#include <popcorn_vision/CloudToMesh.h>
#include <shape_msgs/Mesh.h>
#include <shape_msgs/MeshTriangle.h>
#include <visualization_msgs/Marker.h>
#include <map>

using namespace popcorn_vision;

ros::Publisher marker_pub;

shape_msgs::Mesh run_triangulate(MyCloud::ConstPtr msg)
{
  // Filter out far points, downsample, triangulate
  MyCloud::Ptr cloud = PclTools::zFilter(msg, 0.f, 1.5f);
  cloud = PclTools::statisticalOutlierFilter(cloud);
  cloud = PclTools::approxVoxelGridFilter(cloud, 0.03);
  pcl::PolygonMesh::Ptr mesh = PclTools::triangulate(cloud);

  // Convert to ros mesh msg
  shape_msgs::Mesh out;
  std::map<uint32_t, uint32_t> mapper;
  for (std::size_t i = 0; i < mesh->polygons.size(); i++)
  {
    const pcl::Vertices& triangle = mesh->polygons[i];
    shape_msgs::MeshTriangle out_tri;
    for (std::size_t j = 0; j < 3; j++)
    {
      uint32_t ind = triangle.vertices[j];
      if (mapper.count(ind) == 0)
      {
        // Add new vertex
        const MyPoint& p = cloud->points[ind];
        geometry_msgs::Point p2;
        p2.x = p.x;
        p2.y = p.y;
        p2.z = p.z;
        out.vertices.push_back(p2);
        mapper[ind] = out.vertices.size() - 1;
        out_tri.vertex_indices[j] = out.vertices.size() - 1;
      }
      else
      {
        // We've already added this vertex
        out_tri.vertex_indices[j] = mapper[ind];
      }
    }
    out.triangles.push_back(out_tri);
  }
  return out;
}

bool service_callback(CloudToMesh::Request &req, CloudToMesh::Response &res)
{
  MyCloud::Ptr cloud(new MyCloud());
  pcl::moveFromROSMsg(req.cloud, *cloud);
  res.mesh = run_triangulate(cloud);
  return true;
}

void debug_cloud_callback(MyCloud::ConstPtr msg)
{
  shape_msgs::Mesh mesh = run_triangulate(msg);

  visualization_msgs::Marker marker;
  marker.header.frame_id = msg->header.frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  for (std::size_t i = 0; i < mesh.triangles.size(); i++)
  {
    for (int j = 0; j < 3; j++) {
      marker.points.push_back(mesh.vertices[mesh.triangles[i].vertex_indices[j]]);
    }
  }
  marker_pub.publish(marker);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_to_mesh_server");
  ros::NodeHandle n;

  // For debugging
  //marker_pub = n.advertise<visualization_msgs::Marker>("/pcl_to_mesh_debug", 1);
  //ros::Subscriber sub = n.subscribe("/kinect2/sd/points", 1, debug_cloud_callback);

  ros::ServiceServer service = n.advertiseService("/cloud_to_mesh", service_callback);

  ros::spin();

  return 0;
}
