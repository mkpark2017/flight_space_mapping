#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <iostream>




typedef pcl::PointXYZRGB RefPointType;

class ecd_cluster
{

  public:

    explicit ecd_cluster(ros::NodeHandle nh) : m_nh(nh)
    {

      // Define the subscriber and publisher
      m_sub = m_nh.subscribe ("/octomap_point_cloud_centers", 1, &ecd_cluster::cloud_cb, this);
      m_pub = m_nh.advertise<sensor_msgs::PointCloud2> ("pcl_clusters", 1);
//      m_clusterPub = m_nh.advertise<euclidean_cluster::SegmentedClustersArray> ("obj_recognition/pcl_clusters",1);
      m_objPub = m_nh.advertise<sensor_msgs::PointCloud2> ("pcl_objects", 1);
      m_min_edgePub = m_nh.advertise<sensor_msgs::PointCloud2> ("pcl_min_edges", 1);
      m_max_edgePub = m_nh.advertise<sensor_msgs::PointCloud2> ("pcl_max_edges", 1);

      m_edgePub = m_nh.advertise<sensor_msgs::PointCloud2> ("pcl_edges", 1);
    }

  private:

  ros::NodeHandle m_nh;
  ros::Subscriber m_sub;
  ros::Publisher m_pub;
  ros::Publisher m_objPub;
  ros::Publisher m_min_edgePub;
  ros::Publisher m_max_edgePub;
  ros::Publisher m_edgePub;

  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

}; // End class


void ecd_cluster::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  double ransac_distance_threashold = 0.02;
  std::string map_frame_id = "notgiven";
  double kd_min_size = 150;
  double kd_max_size = 25000;
  double kd_tolerance = 0.6;

  ros::param::get("~map_frame_id", map_frame_id);
  ros::param::get("~ransac_distance_threashold", ransac_distance_threashold);
  ros::param::get("~kd_min_size", kd_min_size);
  ros::param::get("~kd_max_size", kd_max_size);
  ros::param::get("~kd_tolerance", kd_tolerance);




  // Convert PointCloud2 to pointXYZ
  pcl::PCLPointCloud2 pcl_PC2;
  pcl_conversions::toPCL(*cloud_msg,pcl_PC2);
  pcl::PointCloud<RefPointType>::Ptr pXYZ (new pcl::PointCloud<RefPointType>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pXYZ_mono (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromPCLPointCloud2(pcl_PC2,*pXYZ_mono);

  copyPointCloud(*pXYZ_mono, *pXYZ);

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
  seg.setDistanceThreshold (ransac_distance_threashold);
  seg.setDistanceThreshold (0.3);

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

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<RefPointType>::Ptr tree (new pcl::search::KdTree<RefPointType>);
  tree->setInputCloud (pXYZ);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<RefPointType> ec;
  // Specify euclidean cluster parameters
  ec.setClusterTolerance (kd_tolerance); // 60cm
  ec.setMinClusterSize (kd_min_size);
  ec.setMaxClusterSize (kd_max_size);
  ec.setClusterTolerance (0.6); // 60cm
  ec.setMinClusterSize (150);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (pXYZ);
  ec.extract (cluster_indices);

  // Set pointcloud2 variable
  pcl::PCLPointCloud2 outputOBJ;
  pcl::PCLPointCloud2 outputPCL;
  pcl::PCLPointCloud2 outputMIN;
  pcl::PCLPointCloud2 outputMAX;
  pcl::PCLPointCloud2 outputEDGE;
  // Declare centroid point of clustered cloud
  Eigen::Vector4f crd;
  pcl::PointCloud<RefPointType> obj_center_XYZ;
  // Declare edge points of clustered cloud
  pcl::PointXYZRGB minPt, maxPt;
  pcl::PointCloud<RefPointType> obj_min_edges_XYZ, obj_max_edges_XYZ;
  pcl::PointCloud<RefPointType> obj_edges_XYZ;

  // Declare ROS output variable
  sensor_msgs::PointCloud2 pcl_clusters;
  sensor_msgs::PointCloud2 obj_centers;
  sensor_msgs::PointCloud2 obj_min_edges, obj_max_edges;
  sensor_msgs::PointCloud2 obj_edges;
  int j = 1;
  uint32_t color=0x00000F;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
     pcl::PointCloud<RefPointType>::Ptr cloud_cluster (new pcl::PointCloud<RefPointType>);

    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      cloud_cluster->points.push_back(pXYZ->points[*pit]);
      pXYZ->points[*pit].rgb =  color ;
    }

    // Get point and 3D min, max points from each clustered cloud
    pcl::getMinMax3D (*cloud_cluster, minPt, maxPt);
    minPt.rgb = color;
    maxPt.rgb = color;
    obj_min_edges_XYZ.push_back (minPt);
    obj_max_edges_XYZ.push_back (maxPt);
    obj_edges_XYZ.push_back (minPt);
    obj_edges_XYZ.push_back (maxPt);

    // Get centroid point of objects
    pcl::compute3DCentroid<RefPointType> (*cloud_cluster,  crd);
    pcl::PointXYZRGB ppp;
    ppp.x = crd[0];
    ppp.y = crd[1];
    ppp.z = crd[2];
    ppp.rgb = color;
    obj_center_XYZ.points.push_back (ppp);

    ++j;
    color += 0x00000a << (((j)*5)%25);

//std::vector<int>::const_iterator k = it;
//std::cout << k << std::endl;
  }


  // Convert from pcl::PointXYZ to pcl::PCLPointCloud2
  pcl::toPCLPointCloud2(*pXYZ, outputPCL);
  pcl::toPCLPointCloud2(obj_center_XYZ, outputOBJ);
  pcl::toPCLPointCloud2(obj_min_edges_XYZ, outputMIN);
  pcl::toPCLPointCloud2(obj_max_edges_XYZ, outputMAX);
  pcl::toPCLPointCloud2(obj_edges_XYZ, outputEDGE);

  // Convert from pcl::PCLPointCloud2 to ROS messages
  pcl_conversions::fromPCL(outputOBJ, obj_centers);
  pcl_conversions::fromPCL(outputPCL, pcl_clusters);
  pcl_conversions::fromPCL(outputMIN, obj_min_edges);
  pcl_conversions::fromPCL(outputMAX, obj_max_edges);
  pcl_conversions::fromPCL(outputEDGE, obj_edges);

  obj_centers.header.frame_id = map_frame_id;
  obj_min_edges.header.frame_id = map_frame_id;
  obj_max_edges.header.frame_id = map_frame_id;
  obj_edges.header.frame_id = map_frame_id;

  // Publish the data.
  m_pub.publish(pcl_clusters);
  m_objPub.publish (obj_centers);
  m_min_edgePub.publish(obj_min_edges);
  m_max_edgePub.publish(obj_max_edges);
  m_edgePub.publish(obj_edges);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "euclidean_cluster");
  ros::NodeHandle nh;


  ecd_cluster clusters(nh);

  while(ros::ok())
  // Spin
  ros::spin ();
}
