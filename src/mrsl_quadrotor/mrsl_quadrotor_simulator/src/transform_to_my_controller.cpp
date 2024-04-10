#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <quadrotor_msgs/SO3Command.h>

// static Eigen::Affine3d Rwc_;
ros::Publisher cmd_pub;

// nav_msgs::Odometry tfOdom(nav_msgs::Odometry odom) {
//   tf::Pose odom_tf;
//   tf::poseMsgToTF(odom.pose.pose, odom_tf);
//   Eigen::Affine3d odom_eigen;
//   tf::poseTFToEigen(odom_tf, odom_eigen);
//   odom_eigen = Rwc_ * odom_eigen;
//   tf::poseEigenToTF(odom_eigen, odom_tf);

//   nav_msgs::Odometry odom_out;
//   tf::poseTFToMsg(odom_tf, odom_out.pose.pose);

//   odom_out.header = odom.header;
//   return odom_out;
// }

// void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
//   nav_msgs::Odometry odom_out = *msg;
//   odom_out = tfOdom(odom_out);
//   odom_pub.publish(odom_out);
// }

int main(int argc, char **argv) {
  ros::init(argc, argv, "transform_to_my_controller");
  ros::NodeHandle nh("~");
  // ros::Subscriber odom_sub = nh.subscribe("/juliett/ground_truth/odom", 10, &odomCallback,
  //                                         ros::TransportHints().tcpNoDelay());
  // odom_pub = nh.advertise<nav_msgs::Odometry>("odom_out", 10);
  cmd_pub = nh.advertise<quadrotor_msgs::SO3Command>("/juliett/so3_cmd", 10);


  // double shift_x;
  // double shift_y;
  // double shift_z;
  // double shift_yaw;
  // double shift_pitch;
  // double shift_roll;

  // nh.param("shift_x", shift_x, 0.0);
  // nh.param("shift_y", shift_y, 0.0);
  // nh.param("shift_z", shift_z, 0.0);
  // nh.param("shift_yaw", shift_yaw, 0.0);
  // nh.param("shift_pitch", shift_pitch, 0.0);
  // nh.param("shift_roll", shift_roll, 0.0);

  // Rwc_ = Eigen::Translation3d(shift_x, shift_y, shift_z) *
  //        Eigen::AngleAxisd(shift_yaw * M_PI / 180, Eigen::Vector3d::UnitZ()) *
  //        Eigen::AngleAxisd(shift_roll * M_PI / 180, Eigen::Vector3d::UnitX()) *
  //        Eigen::AngleAxisd(shift_pitch * M_PI / 180, Eigen::Vector3d::UnitY());

  // printTF(Rwc_);

  quadrotor_msgs::SO3Command temp_att_cmd;
  temp_att_cmd.aux.use_external_yaw = false;
  temp_att_cmd.aux.enable_motors = true;
  temp_att_cmd.force.z = 5.0f;
  temp_att_cmd.orientation.w = 1.0f;
  temp_att_cmd.orientation.x = 0.0f;
  temp_att_cmd.orientation.y = 0.2f;
  temp_att_cmd.orientation.z = 0.0f;
  temp_att_cmd.kR[0] = 1.0f;
  temp_att_cmd.kR[1] = 1.0f;
  temp_att_cmd.kR[2] = 1.0f;
  temp_att_cmd.kOm[0] = 1.0f;
  temp_att_cmd.kOm[1] = 1.0f;
  temp_att_cmd.kOm[2] = 1.0f;
  temp_att_cmd.angular_velocity.x = 0.0f;
  temp_att_cmd.angular_velocity.y = 0.0f;
  temp_att_cmd.angular_velocity.z = 0.0f;

  ros::Rate loop_rate(10);
  while(ros::ok()) {
    temp_att_cmd.header.stamp = ros::Time::now();
    cmd_pub.publish(temp_att_cmd);
    ros::spinOnce();
    ROS_INFO("hello");
    loop_rate.sleep();
  }
  return 0;
}
