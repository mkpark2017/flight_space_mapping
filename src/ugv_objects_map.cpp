#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <cmath>

#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/GetMap.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

class obj_map
{
  public:
  explicit obj_map(ros::NodeHandle nh) : m_nh(nh)
  {
    m_sub1 = m_nh.subscribe ("pcl_edges", 1, &obj_map::ObjmapCallback, this);
    m_pub1 = m_nh.advertise<nav_msgs::OccupancyGrid>("objects_map", 1, true);
  }
  private:
  ros::NodeHandle m_nh;
  ros::Subscriber m_sub1;
  ros::Publisher m_pub1;

  void ObjmapCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
}; //End class

void obj_map::ObjmapCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Setup map config
  nav_msgs::OccupancyGrid o_map;
  float resol = 0.5;
  o_map.header.stamp = ros::Time::now();
  o_map.header.frame_id = "world";
  o_map.info.resolution = 0.5;
  o_map.info.origin.orientation.x = 0.0;
  o_map.info.origin.orientation.y = 0.0;
  o_map.info.origin.orientation.z = 0.0;
  o_map.info.origin.orientation.w = 1.0;

  // Convert PointCloud2 to PointXYZ
  pcl::PCLPointCloud2 pcl_PC2;
  pcl_conversions::toPCL(*cloud_msg, pcl_PC2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pXYZ (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_PC2, *pXYZ);

  uint32_t n = pcl_PC2.width;
  uint32_t m = n/2;
  double *min_x = new double[m];
  double *min_y = new double[m];
  double *min_z = new double[m];
  double *max_x = new double[m];
  double *max_y = new double[m];
  double *max_z = new double[m];
  
  double max_map_x = 0;
  double max_map_y = 0;
  double min_map_x = 10;
  double min_map_y = 10;

  int j = 0;
  for (size_t i=0; i<pXYZ -> points.size (); i+=2)
  {
    min_z[j] = pXYZ -> points[i].z;
    min_x[j] = pXYZ -> points[i].x;
    min_y[j] = pXYZ -> points[i].y;

    max_z[j] = pXYZ -> points[i+1].z;
    max_x[j] = pXYZ -> points[i+1].x;
    max_y[j] = pXYZ -> points[i+1].y;

    if (max_map_x < max_x[j])
      max_map_x = max_x[j];

    if (max_map_y < max_y[j])
      max_map_y = max_y[j];

    if (min_map_x > min_x[j])
      min_map_x = min_x[j];

    if (min_map_y > min_y[j])
      min_map_y = min_y[j];

    j++;

  }
  double map_width = ceil( (max_map_x-min_map_x)/resol );
  double map_height = ceil( (max_map_y-min_map_y)/resol );

  o_map.info.origin.position.x = min_map_x;
  o_map.info.origin.position.y = min_map_y;
  o_map.info.origin.position.z = 0.0;
  o_map.info.width = map_width;
  o_map.info.height = map_height;

  o_map.data.resize(o_map.info.width * o_map.info.height);


  for(int x=0; x<map_width; x++)
  {
    for(int y=0; y<map_height; y++)
    {
        o_map.data[MAP_IDX(o_map.info.width, x, y)] =  0;
    }
  }

  for (int i=0; i<m; i++)
  {
    //when min_z < 0.5m, UGV can't pass through, if max_z > 0.3m the object can't be negligible
    if (min_z[i] <= 0.5 && max_z[i]>= 0.3)
    {
      double x_start = floor( (min_x[i]-min_map_x) /resol);
      double x_end = ceil( (max_x[i]-min_map_x) /resol);
      double y_start = floor( (min_y[i]-min_map_y) /resol);
      double y_end = ceil( (max_y[i]-min_map_y) /resol);

      for (int x=x_start ; x < x_end; x++)
      {
        for (int y = y_start; y < y_end; y++)
        {
          o_map.data[MAP_IDX(o_map.info.width, x, y)] =  100;
        }
      }
    }
  }



  delete [] min_x;
  delete [] min_y;
  delete [] min_z;
  delete [] max_x;
  delete [] max_y;
  delete [] max_z;

  m_pub1.publish(o_map);
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "objects_map");
  ros::NodeHandle nh;

  obj_map map(nh);

  while(ros::ok())
    ros::spin();

  return 0;
}
