#include "ackermann_to_odom_alg_node.h"

AckermannToOdomAlgNode::AckermannToOdomAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<AckermannToOdomAlgorithm>()
{
  //init class attributes if necessary
  //this->loop_rate_ = 10; //in [Hz]
  this->virtual_imu_msg_.orientation.x = 0.0;
  this->virtual_imu_msg_.orientation.y = 0.0;
  this->virtual_imu_msg_.orientation.z = 0.0;
  this->virtual_imu_msg_.orientation.w = 1.0;
  this->estimated_ackermann_state_.drive.speed = 0.0;
  this->estimated_ackermann_state_.drive.steering_angle = 0.0;

  // [init publishers]
  this->odometry_publisher_ = this->public_node_handle_.advertise < nav_msgs::Odometry > ("/odometry", 1);
  this->pose_publisher_ = this->public_node_handle_.advertise < geometry_msgs::PoseWithCovarianceStamped > ("/odometry_pose", 1);

  // [init subscribers]
  this->estimated_ackermann_subscriber_ = this->public_node_handle_.subscribe(
      "/estimated_ackermann_state", 1, &AckermannToOdomAlgNode::cb_ackermannState, this);
  this->virtual_imu_subscriber_ = this->public_node_handle_.subscribe("/virtual_imu_data", 1,
                                                                      &AckermannToOdomAlgNode::cb_imuData, this);

  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

AckermannToOdomAlgNode::~AckermannToOdomAlgNode(void)
{
  // [free dynamic memory]
}

void AckermannToOdomAlgNode::mainNodeThread(void)
{
  static bool first_exec = true;
  static bool odom_in_tf;
  static bool scan_in_tf;
  static std::string frame_id;
  static std::string child_id;

  // [read parameters]
  if (first_exec)
  {
    this->public_node_handle_.getParam("/odom_in_tf", odom_in_tf);
    this->public_node_handle_.getParam("/scan_in_tf", scan_in_tf);
    this->public_node_handle_.getParam("/frame_id", frame_id);
    this->public_node_handle_.getParam("/child_id", child_id);
    first_exec = false;
  }

  // [listen transform]
  try
  {
    this->listener_.lookupTransform(frame_id, child_id, ros::Time(0), this->scan_trans_);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  // [fill msg structures]
  this->alg_.generateNewOdometryMsg2D(this->estimated_ackermann_state_, this->virtual_imu_msg_, this->odometry_pose_,
                                      this->odometry_, this->odom_trans_);

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
  if (odom_in_tf)
  {
    this->broadcaster_.sendTransform(this->odom_trans_);
  }
  if (scan_in_tf)
  {
    this->broadcaster_.sendTransform(this->scan_trans_);
  }

  this->odometry_publisher_.publish(this->odometry_);
  this->pose_publisher_.publish(this->odometry_pose_);
}

/*  [subscriber callbacks] */
void AckermannToOdomAlgNode::cb_ackermannState(
    const ackermann_msgs::AckermannDriveStamped::ConstPtr& estimated_ackermann_state_msg)
{
  this->alg_.lock();

  this->estimated_ackermann_state_.drive.speed = estimated_ackermann_state_msg->drive.speed;
  this->estimated_ackermann_state_.drive.steering_angle = estimated_ackermann_state_msg->drive.steering_angle;

  this->alg_.unlock();
}

void AckermannToOdomAlgNode::cb_imuData(const sensor_msgs::Imu::ConstPtr& Imu_msg)
{
  this->alg_.lock();

  this->virtual_imu_msg_.orientation.x = Imu_msg->orientation.x;
  this->virtual_imu_msg_.orientation.y = Imu_msg->orientation.y;
  this->virtual_imu_msg_.orientation.z = Imu_msg->orientation.z;
  this->virtual_imu_msg_.orientation.w = Imu_msg->orientation.w;

  this->alg_.unlock();
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void AckermannToOdomAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_ = config;
  this->alg_.unlock();
}

void AckermannToOdomAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc, char *argv[])
{
  return algorithm_base::main < AckermannToOdomAlgNode > (argc, argv, "ackermann_to_odom_alg_node");
}
