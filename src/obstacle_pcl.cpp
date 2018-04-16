#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class abst_map
{
  public:
    explicit abst_map(ros::NodeHandle nh) : m_nh(nh)
    {
      m_sub1 = m_nh.subscribe ("/pcl_edges", 1, &abst_map::PCLCallback, this);
      m_pub1 = m_nh.advertise<snesor_msgs::PointCloud2>( "obstacle_pcl", 0);
    }
  private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_sub1;
    ros::Publisher m_pub1;

  void PCLCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
} // End class

void abst_map::MapCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  double min_x, min_y, min_z, max_x, max_y, max_z;
  // Convert PointCloud2 to PointXYZ
  pcl::PCLPointCloud2 pcl_PC2;
  pcl_conversions::toPCL(*cloud_msg, pcl_PC2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pXYZ (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_PC2, *pXYZ);


  for (size_t i=0; i<pXYZ.points.size (); i++)
  {
    pcl::PointXYZRGB objects;

    min_x = pXYZ -> points[i].x;
    min_y = pXYZ -> points[i].y;
    min_z = pXYZ -> points[i].z;

    max_x = pXYZ -> points[i+1].x;
    max_y = pXYZ -> points[i+1].y;
    max_z = pXYZ -> points[i+1].z;

    length_x = max_x - min_x;
    length_y = max_y - min_y;
    length_z = max_z - min_z;

    num_x = length/
  }
}


int main (int argc, char **argv)
{
  // Initialize ROS
  ros init(argc, argv, "abstract_map");
  ros::NodeHandle nh;


  while(ros::ok())
  // Spin
  ros::spin();

  return 0;
}
