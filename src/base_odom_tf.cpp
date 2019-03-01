#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


void poseCallback(const nav_msgs::OdometryConstPtr& msg){
  std::string parent_frame_id = "notgiven";
  std::string child_frame_id = "notgiven";
  std::vector<float> offset; //[x y z, x y z w];

//  ros::param::set("parent_frame_id", "odom");
//  ros::param::set("child_frame_id", "base_link");
  ros::param::get("~parent_frame_id", parent_frame_id);
  ros::param::get("~child_frame_id", child_frame_id);
  ros::param::get("~offset", offset);
std::cout << offset[0] << std::endl;
  if(ros::param::get("~offset", offset) )
  {
    ROS_INFO("Got param: %f", offset[0]);
  }
  else
  {
    ROS_ERROR("Failed to get param 'offset'");
  }

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  float offset_x = 100;
  transform.setOrigin( tf::Vector3(
    msg->pose.pose.position.x + offset[0],
    msg->pose.pose.position.y + offset[1],
    msg->pose.pose.position.z + offset[2]
  ) );
  tf::Quaternion q(
    msg->pose.pose.orientation.x + offset[3],
    msg->pose.pose.orientation.y + offset[4],
    msg->pose.pose.orientation.z + offset[5],
    msg->pose.pose.orientation.w + offset[6]
  );
  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame_id, child_frame_id));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "base_odom_tf");
  ros::NodeHandle node;


  ros::Subscriber sub = node.subscribe("odometry_sensor1/odometry", 10, &poseCallback);

  while(ros::ok())
    ros::spin();

  return 0;
};
