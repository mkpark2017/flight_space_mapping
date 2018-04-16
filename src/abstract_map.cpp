#include <cmath>        // std::abs
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


class abst_map
{
  public:
    explicit abst_map(ros::NodeHandle nh) : m_nh(nh)
    {
      m_sub1 = m_nh.subscribe ("pcl_edges", 1, &abst_map::MapCallback, this);
      m_pub1 = m_nh.advertise<visualization_msgs::MarkerArray>( "visualization_marker", 2);
    }
  private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_sub1;
    ros::Publisher m_pub1;

  void MapCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
}; // End class

void abst_map::MapCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Convert PointCloud2 to PointXYZ
  pcl::PCLPointCloud2 pcl_PC2;
  pcl_conversions::toPCL(*cloud_msg, pcl_PC2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pXYZ (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_PC2, *pXYZ);

  visualization_msgs::MarkerArray marker_points;
  for (size_t i=0; i<pXYZ -> points.size (); i+= 2)
  {
    double min_x, min_y, min_z, max_x, max_y, max_z;

//    uint32_t rgb = *reinterpret_cast<int*>(pXYZ -> points[i].rgb);
 //   uint8_t r = (rgb >> 16) & 0x0000ff;
  //  uint8_t g = (rgb >> 8)  & 0x0000ff;
   // uint8_t b = (rgb)       & 0x0000ff;

    visualization_msgs::Marker marker_point;

    min_x = pXYZ -> points[i].x;
    min_y = pXYZ -> points[i].y;
    min_z = pXYZ -> points[i].z;

    max_x = pXYZ -> points[i+1].x;
    max_y = pXYZ -> points[i+1].y;
    max_z = pXYZ -> points[i+1].z;

    geometry_msgs::Pose p;
    geometry_msgs::Vector3 s;

    p.position.x = (min_x+max_x)/2;
    p.position.y = (min_y+max_y)/2;
    p.position.z = (min_z+max_z)/2;

    s.x = max_x-min_x;
    s.y = max_y-min_y;
    s.z = max_z-min_z;

    marker_point.pose = p;
    marker_point.scale = s;
    marker_point.color.a = 0.6;
//    marker_point.color.r = pXYZ -> points[i].r;
//    marker_point.color.g = pXYZ -> points[i].g;
//    marker_point.color.b = pXYZ -> points[i].b;

    marker_point.color.r = 1;
    marker_point.color.g = 1;
    marker_point.color.b = 1;



    marker_point.header.frame_id = "world";
    marker_point.header.stamp = ros::Time::now();
    marker_point.ns = "my_namespace";
    marker_point.action = visualization_msgs::Marker::ADD;

    marker_point.id = i;

    marker_point.type = visualization_msgs::Marker::CUBE;

    marker_points.markers.push_back (marker_point);
    std::cout << i << std::endl;
  }

  m_pub1.publish(marker_points);

}


int main (int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "abstract_map");
  ros::NodeHandle nh;

  abst_map map(nh);

  while(ros::ok())
  // Spin
  ros::spin();

  return 0;
}
