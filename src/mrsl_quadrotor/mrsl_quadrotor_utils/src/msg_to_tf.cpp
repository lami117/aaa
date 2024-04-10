#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <topic_tools/shape_shifter.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

std::string child_frame_id;
std::string child_without_Y_frame_id;
std::string tare_frame_id;

void process(const geometry_msgs::Pose &pose, const std_msgs::Header& header, std::string child_frame_id) {
  static tf2_ros::TransformBroadcaster odom_br;
  static tf2_ros::TransformBroadcaster odom_br2;
  static tf2_ros::TransformBroadcaster odom_br_tare;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header = header;
  transformStamped.child_frame_id = child_frame_id;
  transformStamped.transform.translation.x = pose.position.x;
  transformStamped.transform.translation.y = pose.position.y;
  transformStamped.transform.translation.z = pose.position.z;
  transformStamped.transform.rotation.x = pose.orientation.x;
  transformStamped.transform.rotation.y = pose.orientation.y;
  transformStamped.transform.rotation.z = pose.orientation.z;
  transformStamped.transform.rotation.w = pose.orientation.w;
  odom_br.sendTransform(transformStamped);
  ROS_WARN_ONCE("publish tf from [%s] to [%s]",
                transformStamped.header.frame_id.c_str(),
                transformStamped.child_frame_id.c_str());

  geometry_msgs::TransformStamped transformStamped2;
  transformStamped2.header = header;
  // transformStamped2.header.frame_id = transformStamped.child_frame_id;
  transformStamped2.child_frame_id = child_without_Y_frame_id;
  transformStamped2.transform.translation.x = pose.position.x;
  transformStamped2.transform.translation.y = pose.position.y;
  transformStamped2.transform.translation.z = 0.0f;//pose.position.z;
  double yaw = tf::getYaw(pose.orientation);// + shift_yaw_;

  transformStamped2.transform.rotation.w = cos(yaw / 2);
  transformStamped2.transform.rotation.x = 0.0;
  transformStamped2.transform.rotation.y = 0.0;
  transformStamped2.transform.rotation.z = sin(yaw / 2);

  // transformStamped2.transform.rotation.x = pose.orientation.x;
  // transformStamped2.transform.rotation.y = pose.orientation.y;
  // transformStamped2.transform.rotation.z = pose.orientation.z;
  // transformStamped2.transform.rotation.w = pose.orientation.w;
  odom_br2.sendTransform(transformStamped2);
  ROS_WARN_ONCE("publish tf from [%s] to [%s]",
                transformStamped2.header.frame_id.c_str(),
                transformStamped2.child_frame_id.c_str());
  
  
  geometry_msgs::TransformStamped transformStampedForTare;
  transformStampedForTare.header = header;
  transformStampedForTare.child_frame_id = tare_frame_id;
  transformStampedForTare.transform.translation.x = 0;
  transformStampedForTare.transform.translation.y = 0;
  transformStampedForTare.transform.translation.z = 0;
  transformStampedForTare.transform.rotation.x = pose.orientation.x;
  transformStampedForTare.transform.rotation.y = pose.orientation.y;
  transformStampedForTare.transform.rotation.z = pose.orientation.z;
  transformStampedForTare.transform.rotation.w = pose.orientation.w;
  odom_br_tare.sendTransform(transformStampedForTare);
}

void msgCallback(const topic_tools::ShapeShifter::ConstPtr &msg) {
  if(msg->getDataType() == "nav_msgs/Odometry") {
    auto odom = msg->instantiate<nav_msgs::Odometry>();
    process(odom->pose.pose, odom->header, child_frame_id);
  }
  else if(msg->getDataType() == "geometry_msgs/PoseStamped") {
    auto pose = msg->instantiate<geometry_msgs::PoseStamped>();
    process(pose->pose, pose->header, child_frame_id);
  }
  else 
    ROS_WARN_ONCE("Unrecognized msg type! [%s]", msg->getDataType().c_str());
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "msg_to_tf");
  ros::NodeHandle nh("~");

  std::string sub_topic_name;
  std::string uav_name;
  nh.param<std::string>("uav_name", uav_name, "juliett");

  child_frame_id = uav_name + "/velodyne";
  sub_topic_name = "/" + uav_name + "/ground_truth/odom";
  
  child_without_Y_frame_id = uav_name + "_with_yaw";
  // nh.param("child_frame_id", child_frame_id, std::string("null_frame"));
  // nh.param("sub_topic_name", sub_topic_name, std::string("null_topic"));

  tare_frame_id = uav_name + "_for_tare";
  ros::Subscriber sub = nh.subscribe(sub_topic_name, 10, msgCallback,
                                     ros::TransportHints().tcpNoDelay());

  ros::spin();
  return 0;
};
