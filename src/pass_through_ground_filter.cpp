#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <string.h>



typedef pcl::PointXYZ RefPointType;

class ground_filter {

public:

  explicit ground_filter(ros::NodeHandle nh) : m_nh(nh)  {

    // Define the subscriber and publisher
    m_sub = m_nh.subscribe ("velodyne_points", 1, &ground_filter::cloud_cb, this);
    m_objPub = m_nh.advertise<sensor_msgs::PointCloud2> ("pcl_obj", 1);
  }

private:

ros::NodeHandle m_nh;
ros::Subscriber m_sub;
ros::Publisher m_objPub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

}; // End class

void ground_filter::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  double filter_min;
  double filter_max;

  ros::param::get("~filter_min", filter_min);
  ros::param::get("~filter_max", filter_max);


  if(ros::param::get("~filter_min", filter_min) )
  {
    ROS_INFO("Got param: %d", filter_min);
  }
  else
  {
    ROS_ERROR("Failed to get param 'filter_min'");
  }


  // Convert PointCloud2 to pointXYZ
  pcl::PCLPointCloud2 pcl_PC2;
  pcl_conversions::toPCL(*cloud_msg,pcl_PC2);
  pcl::PointCloud<RefPointType>::Ptr pXYZ (new pcl::PointCloud<RefPointType>);
  pcl::fromPCLPointCloud2(pcl_PC2,*pXYZ);

  // Point cloud for planer segmentation filter
  pcl::PointCloud<RefPointType>::Ptr pXYZ_f (new pcl::PointCloud<RefPointType>);

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (pXYZ);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (filter_min, filter_max);
  pass.filter (*pXYZ_f);
  *pXYZ = *pXYZ_f;
  sensor_msgs::PointCloud2 pcl_obj;
  pcl::PCLPointCloud2 outputPCL;
  // Convert from pcl::PointXYZ to pcl::PCLPointCloud2
  pcl::toPCLPointCloud2(*pXYZ, outputPCL);
  // Convert from pcl::PCLPointCloud2 to ROS messages
  pcl_conversions::fromPCL(outputPCL, pcl_obj);

  // Publish the data.
  m_objPub.publish(pcl_obj);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pass_through_ground_filter");
  ros::NodeHandle nh;

  ground_filter pcl_obj(nh);

  while(ros::ok())
  // Spin
  ros::spin ();
}
