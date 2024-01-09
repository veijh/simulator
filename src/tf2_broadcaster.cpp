#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

std::string namespace_str;

void base_link_cb(const nav_msgs::Odometry::ConstPtr& msg){
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = msg->header.frame_id;
  transformStamped.child_frame_id = namespace_str + "_" + "base_link";
  transformStamped.transform.translation.x = msg->pose.pose.position.x;
  transformStamped.transform.translation.y = msg->pose.pose.position.y;
  transformStamped.transform.translation.z = msg->pose.pose.position.z;

  transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
  transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
  transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
  transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;

  br.sendTransform(transformStamped);

  static ros::NodeHandle nh("");
  static ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
  nav_msgs::Odometry odom_msg(*msg);
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.child_frame_id = namespace_str + "_" + "base_link";
  pub.publish(odom_msg);
}

void camera_color_frame_cb(const nav_msgs::Odometry::ConstPtr& msg){
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = msg->header.frame_id;
  transformStamped.child_frame_id = namespace_str + "_" + "gazebo_color_frame";
  transformStamped.transform.translation.x = msg->pose.pose.position.x;
  transformStamped.transform.translation.y = msg->pose.pose.position.y;
  transformStamped.transform.translation.z = msg->pose.pose.position.z;

  transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
  transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
  transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
  transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;

  br.sendTransform(transformStamped);

  tf2::Quaternion q_xyz(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf2::Quaternion q_znxny;
  q_znxny.setRPY(-M_PI/2, 0, -M_PI/2);

  transformStamped.header.frame_id = namespace_str + "_" + "gazebo_color_frame";
  transformStamped.child_frame_id = namespace_str + "_" + "camera_color_frame";
  transformStamped.transform.translation.x = 0;
  transformStamped.transform.translation.y = 0;
  transformStamped.transform.translation.z = 0;
  transformStamped.transform.rotation.x = q_znxny.x();
  transformStamped.transform.rotation.y = q_znxny.y();
  transformStamped.transform.rotation.z = q_znxny.z();
  transformStamped.transform.rotation.w = q_znxny.w();

  br.sendTransform(transformStamped);
}

void camera_depth_frame_cb(const nav_msgs::Odometry::ConstPtr& msg){
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = msg->header.frame_id;
  transformStamped.child_frame_id = namespace_str + "_" + "gazebo_depth_frame";
  transformStamped.transform.translation.x = msg->pose.pose.position.x;
  transformStamped.transform.translation.y = msg->pose.pose.position.y;
  transformStamped.transform.translation.z = msg->pose.pose.position.z;

  transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
  transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
  transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
  transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;

  br.sendTransform(transformStamped);

  tf2::Quaternion q_xyz(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf2::Quaternion q_znxny;
  q_znxny.setRPY(-M_PI/2, 0, -M_PI/2);

  transformStamped.header.frame_id = namespace_str + "_" + "gazebo_depth_frame";
  transformStamped.child_frame_id = namespace_str + "_" + "camera_depth_frame";
  transformStamped.transform.translation.x = 0;
  transformStamped.transform.translation.y = 0;
  transformStamped.transform.translation.z = 0;
  transformStamped.transform.rotation.x = q_znxny.x();
  transformStamped.transform.rotation.y = q_znxny.y();
  transformStamped.transform.rotation.z = q_znxny.z();
  transformStamped.transform.rotation.w = q_znxny.w();

  br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf2_broadcaster");

  ros::NodeHandle nh("");
  namespace_str = nh.getNamespace();
  if (!namespace_str.empty() && namespace_str[0] == '/') {
        namespace_str = namespace_str.substr(1);
  }
  ros::Subscriber base_link = nh.subscribe<nav_msgs::Odometry>("ground_truth/base_link", 1, &base_link_cb);
  ros::Subscriber camera_color_frame = nh.subscribe<nav_msgs::Odometry>("ground_truth/camera_color_frame", 1, &camera_color_frame_cb);
  ros::Subscriber camera_depth_frame = nh.subscribe<nav_msgs::Odometry>("ground_truth/camera_depth_frame", 1, &camera_depth_frame_cb);

  ros::spin();
  return 0;
}
