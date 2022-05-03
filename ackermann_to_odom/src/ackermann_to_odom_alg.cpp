#include "ackermann_to_odom_alg.h"
#include <time.h>

AckermannToOdomAlgorithm::AckermannToOdomAlgorithm(void)
{
  pthread_mutex_init(&this->access_, NULL);
}

AckermannToOdomAlgorithm::~AckermannToOdomAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void AckermannToOdomAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_ = config;

  this->unlock();
}

// AckermannToOdomAlgorithm Public API
void AckermannToOdomAlgorithm::generateNewOdometryMsg2D(ackermann_msgs::AckermannDriveStamped estimated_ackermann_state,
                                                        sensor_msgs::Imu virtual_imu_msg,
                                                        geometry_msgs::PoseWithCovarianceStamped& odometry_pose,
                                                        nav_msgs::Odometry& odometry,
                                                        geometry_msgs::TransformStamped& odom_trans)
{

  int i, j;
  float pose_yaw = 0;
  static float pose_yaw_prev = 0;
  static float pose_x_prev = 0;
  static float pose_y_prev = 0;
  static double t_1;
  static double t_2;
  static bool first_exec = true;
  float d_vehicle = 1.08; // TODO: get from param and modify git .rm
  bool flag_imu = true; // TODO: get from param and modify git .rm
  tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, 0);;

  /////////////////////////////////////////////////
  //// POSE AND VELOCITY
  //calculate increment of time
  if (first_exec)
  {
    t_2 = (double)ros::Time::now().toSec();
    first_exec = false;
  }
  t_1 = (double)ros::Time::now().toSec();
  float delta_t = (float)(t_1 - t_2);
  t_2 = (double)ros::Time::now().toSec();

  //read information of low-level sensor
  float lineal_speed = estimated_ackermann_state.drive.speed;
  float steering_radians = estimated_ackermann_state.drive.steering_angle * M_PI / 180.0;

  //angle
  if (flag_imu)
  {
    tf::Quaternion q(virtual_imu_msg.orientation.x, virtual_imu_msg.orientation.y, virtual_imu_msg.orientation.z,
                     virtual_imu_msg.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    pose_yaw = yaw;
    quaternion = tf::createQuaternionFromRPY(0, 0, pose_yaw);
  }
  else
  {
    float angular_speed_yaw = (lineal_speed / d_vehicle) * sin(steering_radians);
    pose_yaw = pose_yaw_prev + angular_speed_yaw * delta_t;
    quaternion = tf::createQuaternionFromRPY(0, 0, pose_yaw);
  }

  //pose
  float lineal_speed_x = lineal_speed * cos(pose_yaw) * cos(steering_radians);
  float lineal_speed_y = lineal_speed * sin(pose_yaw) * cos(steering_radians);
  float pose_x = pose_x_prev + lineal_speed_x * delta_t;
  float pose_y = pose_y_prev + lineal_speed_y * delta_t;
  if (isnan(pose_yaw))
  {
    lineal_speed_x = 0.0;
    lineal_speed_y = 0.0;
    pose_x = 0.0;
    pose_y = 0.0;
    quaternion = tf::createQuaternionFromRPY(0, 0, 0);
    ROS_INFO("isnan(pose_yaw)");
  }

  // For next step
  if (abs(pose_x_prev - pose_x) < MAX_DIFF && abs(pose_y_prev - pose_y) < MAX_DIFF)
  {
    pose_x_prev = pose_x;
    pose_y_prev = pose_y;
    pose_yaw_prev = pose_yaw;
  }
  else
  {
    pose_x = pose_x_prev;
    pose_y = pose_y_prev;
    pose_yaw = pose_yaw_prev;
    quaternion = tf::createQuaternionFromRPY(0, 0, pose_yaw);
  }
  /////////////////////////////////////////////////

  /////////////////////////////////////////////////
  //// GENERATE MESSAGE
  // Header
  odometry.header.stamp = ros::Time::now();
  odometry.header.frame_id = "odom";
  odometry.child_frame_id = "base_link";
  odometry_pose.header.stamp = ros::Time::now();
  odometry_pose.header.frame_id = "odom";

  // Twist
  odometry.twist.twist.linear.x = lineal_speed_x;
  odometry.twist.twist.linear.y = lineal_speed_y;
  odometry.twist.twist.linear.z = 0;
  odometry.twist.twist.angular.x = 0;
  odometry.twist.twist.angular.y = 0;
  odometry.twist.twist.angular.z = 0;

  // Pose
  odometry.pose.pose.position.x = pose_x;
  odometry.pose.pose.position.y = pose_y;
  odometry.pose.pose.position.z = 0;
  odometry.pose.pose.orientation.x = quaternion[0];
  odometry.pose.pose.orientation.y = quaternion[1];
  odometry.pose.pose.orientation.z = quaternion[2];
  odometry.pose.pose.orientation.w = quaternion[3];
  odometry_pose.pose.pose.position.x = pose_x;
  odometry_pose.pose.pose.position.y = pose_y;
  odometry_pose.pose.pose.position.z = 0;
  odometry_pose.pose.pose.orientation.x = quaternion[0];
  odometry_pose.pose.pose.orientation.y = quaternion[1];
  odometry_pose.pose.pose.orientation.z = quaternion[2];
  odometry_pose.pose.pose.orientation.w = quaternion[3];
  odometry_pose.pose.covariance[0] = 0.5; // TODO: calculation of variances !!!
  odometry_pose.pose.covariance[7] = 0.5;
  odometry_pose.pose.covariance[35] = 0.5;
  /////////////////////////////////////////////////

  ////////////////////////////////////////////////////////////////
  //// GENERATE MESSAGES TF
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.transform.translation.x = pose_x;
  odom_trans.transform.translation.y = pose_y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(pose_yaw);
  ////////////////////////////////////////////////////////////////
}
