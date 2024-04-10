#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <quadrotor_msgs/SO3Command.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>

// static Eigen::Affine3d Rwc_;
ros::Publisher cmd_pub, pos_vio_pub, vel_vio_pub, att_vio_pub, acc_vio_pub;

ros::Publisher mavros_state_pub;

bool has_armed = false;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  // nav_msgs::Odometry odom_out = *msg;
  // odom_out = tfOdom(odom_out);
  // odom_pub.publish(odom_out);
  geometry_msgs::PoseStamped pos_msg;
  geometry_msgs::Vector3Stamped vel_msg;
  geometry_msgs::Vector3Stamped acc_msg;
  geometry_msgs::PoseStamped att_msg;

  pos_msg.header = msg->header;
  pos_msg.pose.position.x = msg->pose.pose.position.x;
  pos_msg.pose.position.y = -msg->pose.pose.position.y;
  pos_msg.pose.position.z = -msg->pose.pose.position.z;

  vel_msg.header = msg->header;
  vel_msg.vector.x = msg->twist.twist.linear.x;
  vel_msg.vector.y = -msg->twist.twist.linear.y;
  vel_msg.vector.z = -msg->twist.twist.linear.z;

  acc_msg.header = msg->header;
  acc_msg.vector.x = 0.0f;
  acc_msg.vector.y = 0.0f;
  acc_msg.vector.z = 0.0f;

  att_msg.header = msg->header;
  att_msg.pose.orientation.w = msg->pose.pose.orientation.w;
  att_msg.pose.orientation.x = msg->pose.pose.orientation.x;
  att_msg.pose.orientation.y = -msg->pose.pose.orientation.y;
  att_msg.pose.orientation.z = -msg->pose.pose.orientation.z;

  pos_vio_pub.publish(pos_msg);
  vel_vio_pub.publish(vel_msg);
  att_vio_pub.publish(att_msg);
  acc_vio_pub.publish(acc_msg);

  mavros_msgs::State uav_state_msg;
  uav_state_msg.header = msg->header;
  uav_state_msg.connected = true;
  uav_state_msg.armed = has_armed;
  uav_state_msg.mode = "OFFBOARD";
  mavros_state_pub.publish(uav_state_msg);
}
void cmdCallback(const mavros_msgs::AttitudeTarget::ConstPtr &msg) {

  quadrotor_msgs::SO3Command temp_att_cmd;
  temp_att_cmd.aux.use_external_yaw = false;
  temp_att_cmd.aux.enable_motors = true;
  float temp_trust = msg->thrust;
  if (has_armed) {
    temp_trust = temp_trust * 10.0f;
    // std::cout << "f: " << temp_trust << std::endl;
  } else {
    temp_trust = 0.0f;
  }
  temp_att_cmd.force.z = temp_trust;
  temp_att_cmd.orientation.w = msg->orientation.w;
  temp_att_cmd.orientation.x = msg->orientation.x;
  temp_att_cmd.orientation.y = -msg->orientation.y;
  temp_att_cmd.orientation.z = -msg->orientation.z;
  temp_att_cmd.kR[0] = 1.0f;
  temp_att_cmd.kR[1] = 1.0f;
  temp_att_cmd.kR[2] = 0.2f;
  temp_att_cmd.kOm[0] = 1.0f;
  temp_att_cmd.kOm[1] = 1.0f;
  temp_att_cmd.kOm[2] = 0.3f;
  temp_att_cmd.angular_velocity.x = 0.0f;
  temp_att_cmd.angular_velocity.y = 0.0f;
  temp_att_cmd.angular_velocity.z = 0.0f;
  temp_att_cmd.header.stamp = ros::Time::now();
    cmd_pub.publish(temp_att_cmd);
}

bool arm_srv(mavros_msgs::CommandBool::Request & req,
            mavros_msgs::CommandBool::Response & res) {
                has_armed = req.value;
                res.success = true;
                return true;
            }

int main(int argc, char **argv) {
  ros::init(argc, argv, "transform_to_my_controller");
  ros::NodeHandle nh("~");
  int id;
  std::string uav_name;
  nh.param<int>("uav_id", id, 1);
  nh.param<std::string>("uav_name", uav_name, "juliett");

  std::string odom_name = "/";
  odom_name += uav_name;
  odom_name += "/ground_truth/odom";

  std::string mavros_parent_name = "/mavros";
  std::string mavros_cmd_name = mavros_parent_name + std::to_string(id);
  mavros_cmd_name += "/setpoint_raw/attitude";

  std::string cmd_pub_name = "/";
  cmd_pub_name += uav_name;
  cmd_pub_name += "/so3_cmd";

  std::string vio_parent_name = "/vio_data_rigid";

  std::string vio_pos_pub_name = vio_parent_name + std::to_string(id);
  vio_pos_pub_name += "/pos";
  
  std::string vio_vel_pub_name = vio_parent_name + std::to_string(id);
  vio_vel_pub_name += "/vel";

  std::string vio_acc_pub_name = vio_parent_name + std::to_string(id);
  vio_acc_pub_name += "/acc";
  
  std::string vio_att_pub_name = vio_parent_name + std::to_string(id);
  vio_att_pub_name += "/att";

  std::string mavros_state_name = mavros_parent_name + std::to_string(id);
  mavros_state_name += "/state";

  std::string mavros_srv_name = mavros_parent_name + std::to_string(id);
  mavros_srv_name += "/cmd/arming";

  ros::Subscriber odom_sub = nh.subscribe(odom_name, 10, &odomCallback,
                                          ros::TransportHints().tcpNoDelay());

  ros::Subscriber mavros_cmd_sub = nh.subscribe(mavros_cmd_name, 10, &cmdCallback,
                                          ros::TransportHints().tcpNoDelay());
  // odom_pub = nh.advertise<nav_msgs::Odometry>("odom_out", 10);
  cmd_pub = nh.advertise<quadrotor_msgs::SO3Command>(cmd_pub_name, 10);

  pos_vio_pub = nh.advertise<geometry_msgs::PoseStamped>(vio_pos_pub_name, 10);
  vel_vio_pub = nh.advertise<geometry_msgs::Vector3Stamped>(vio_vel_pub_name, 10);
  acc_vio_pub = nh.advertise<geometry_msgs::Vector3Stamped>(vio_acc_pub_name, 10);
  att_vio_pub = nh.advertise<geometry_msgs::PoseStamped>(vio_att_pub_name, 10);
 
  mavros_state_pub = nh.advertise<mavros_msgs::State>(mavros_state_name, 10);

  ros::ServiceServer service = nh.advertiseService(mavros_srv_name, arm_srv);
  
  ROS_INFO("hello");

  // ros::Rate loop_rate(10);
  // while(ros::ok()) {
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }
  ros::spin();
  return 0;
}
