#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <iostream>

double distance_threashold = 0.02;

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
  // Convert PointCloud2 to pointXYZ
  pcl::PCLPointCloud2 pcl_PC2;
  pcl_conversions::toPCL(*cloud_msg,pcl_PC2);
  pcl::PointCloud<RefPointType>::Ptr pXYZ (new pcl::PointCloud<RefPointType>);
  pcl::fromPCLPointCloud2(pcl_PC2,*pXYZ);

  // Point cloud for planer segmentation filter
  pcl::PointCloud<RefPointType>::Ptr pXYZ_f (new pcl::PointCloud<RefPointType>);

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<RefPointType> seg;
  // Remove plane
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  // Create a pcl object to hold the ransac filtered object
  pcl::PointCloud<RefPointType>::Ptr cloud_plane (new pcl::PointCloud<RefPointType> ());

  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (distance_threashold);

  seg.setInputCloud (pXYZ);
  seg.segment (*inliers, *coefficients);

  // Extract the planar inliers from the input cloud
  pcl::ExtractIndices<RefPointType> extract;
  extract.setInputCloud (pXYZ);
  extract.setIndices (inliers);
  extract.setNegative (false);

  // Get the points associated with the planar surface
  extract.filter (*cloud_plane);

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*pXYZ_f);
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
  ros::init (argc, argv, "ransac_ground_filter");
  ros::NodeHandle nh;
  ros::param::get("~distance_threashold", distance_threashold);
  ground_filter pcl_obj(nh);

  while(ros::ok())
  // Spin
  ros::spin ();
}
