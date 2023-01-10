#include "gps_to_odom_alg_node.h"

GpsToOdomAlgNode::GpsToOdomAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<GpsToOdomAlgorithm>()
{
  //init class attributes if necessary
  this->flag_publish_odom_ = false;
  this->flag_gnss_position_received_ = false;
  this->flag_gnss_velocity_received_ = false;
  //this->loop_rate_ = 10; //in [Hz]
  this->public_node_handle_.getParam("/gps_to_odom/frame_id", this->frame_id_);
  this->public_node_handle_.getParam("/gps_to_odom/min_speed", this->min_speed_);
  this->public_node_handle_.getParam("/gps_to_odom/max_speed", this->max_speed_);

  this->odom_gps_.pose.pose.orientation.x = 0.0;
  this->odom_gps_.pose.pose.orientation.y = 0.0;
  this->odom_gps_.pose.pose.orientation.z = 0.0;
  this->odom_gps_.pose.pose.orientation.w = 1.0;

  // [init publishers]
  this->odom_gps_pub_ = this->public_node_handle_.advertise < nav_msgs::Odometry > ("/odometry_gps", 1);

  // [init subscribers]
  this->sim_fix_sub_ = this->public_node_handle_.subscribe("/fix", 1, &GpsToOdomAlgNode::cb_getSimGpsFixMsg, this);
  this->sim_vel_sub_ = this->public_node_handle_.subscribe("/fix_vel", 1, &GpsToOdomAlgNode::cb_getSimGpsVelMsg, this);
  this->bot_vel_sub_ = this->public_node_handle_.subscribe("/rover/fix_velocity", 1,
                                                           &GpsToOdomAlgNode::cb_getBotGpsVelMsg, this);

  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

GpsToOdomAlgNode::~GpsToOdomAlgNode(void)
{
  // [free dynamic memory]
}

void GpsToOdomAlgNode::mainNodeThread(void)
{

  // [fill msg structures]

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
  this->flag_gnss_velocity_received_ = true;
  if (this->flag_gnss_position_received_ && this->flag_gnss_velocity_received_)
  {
    this->odom_gps_pub_.publish(this->odom_gps_);
    this->flag_gnss_position_received_ = false;
    this->flag_gnss_velocity_received_ = false;
  }
}

/*  [subscriber callbacks] */
void GpsToOdomAlgNode::cb_getSimGpsFixMsg(const sensor_msgs::NavSatFix::ConstPtr &fix_msg)
{
  this->alg_.lock();

  Ellipsoid utm;
  double utm_x;
  double utm_y;
  char utm_zone[30];
  int ref_ellipsoid = 23;

  utm.LLtoUTM(ref_ellipsoid, fix_msg->latitude, fix_msg->longitude, utm_y, utm_x, utm_zone);

  ///////////////////////////////////////////////////////////
  ///// TRANSFORM TO TF FARME
  geometry_msgs::PointStamped fix_tf;
  geometry_msgs::PointStamped fix_utm;
  fix_utm.header.frame_id = "utm";
  fix_utm.header.stamp = ros::Time(0); //ros::Time::now();
  fix_utm.point.x = utm_x;
  fix_utm.point.y = utm_y;
  fix_utm.point.z = 0.0;
  try
  {
    this->listener_.transformPoint(this->frame_id_, fix_utm, fix_tf);
  }
  catch (tf::TransformException &ex)
  {
    ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
    return;
  }
  ///////////////////////////////////////////////////////////

  this->odom_gps_.header = fix_msg->header;
  this->odom_gps_.header.frame_id = this->frame_id_;
  this->odom_gps_.pose.pose.position.x = fix_tf.point.x;
  this->odom_gps_.pose.pose.position.y = fix_tf.point.y;
  this->odom_gps_.pose.pose.position.z = 0.0;
  this->odom_gps_.pose.covariance[0] = fix_msg->position_covariance[0];
  this->odom_gps_.pose.covariance[7] = fix_msg->position_covariance[4];
  this->odom_gps_.pose.covariance[14] = fix_msg->position_covariance[8];

  this->flag_gnss_position_received_ = true;

  this->alg_.unlock();
}

void GpsToOdomAlgNode::cb_getSimGpsVelMsg(const geometry_msgs::Vector3Stamped::ConstPtr &vel_msg)
{
  this->alg_.lock();

  // no transform because the orientation is in north of frame "world" that coincides with frame "map" 
  this->odom_gps_.twist.twist.linear.x = vel_msg->vector.x;
  this->odom_gps_.twist.twist.linear.y = vel_msg->vector.y;
  this->odom_gps_.twist.twist.linear.z = vel_msg->vector.z;

  double yaw = atan2(vel_msg->vector.y, vel_msg->vector.x);
  tf::Quaternion quat_world = tf::createQuaternionFromRPY(0, 0, yaw);

  double speed = sqrt(pow(vel_msg->vector.y, 2) + pow(vel_msg->vector.x, 2));

  if (speed > this->min_speed_)
  {
    this->odom_gps_.pose.pose.orientation.x = quat_world[0];
    this->odom_gps_.pose.pose.orientation.y = quat_world[1];
    this->odom_gps_.pose.pose.orientation.z = quat_world[2];
    this->odom_gps_.pose.pose.orientation.w = quat_world[3];

    double min_variance_yaw = (2 * 3.1416) / 180.0;
    double max_variance_yaw = (360 * 3.1416) / 180.0;
    double dif_variance_yaw = max_variance_yaw - min_variance_yaw;
    double variance_yaw = max_variance_yaw - dif_variance_yaw * (speed / this->max_speed_);

    if (variance_yaw < min_variance_yaw)
    {
      variance_yaw = min_variance_yaw;
    }

    this->odom_gps_.pose.covariance[35] = variance_yaw;

  }

  this->flag_gnss_velocity_received_ = true;

  this->alg_.unlock();
}

void GpsToOdomAlgNode::cb_getBotGpsVelMsg(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &vel_msg)
{
  this->alg_.lock();

  //////////////////// Extract velocities in "map" frame /////////////////////////////////
  // Get the velocities expressed in UTM from the message
  Eigen::Vector3d UTM_velocities = Eigen::Vector3d::Zero();
  UTM_velocities(0) = vel_msg->twist.twist.linear.x; // In UTM frame
  UTM_velocities(1) = vel_msg->twist.twist.linear.y;
  UTM_velocities(2) = vel_msg->twist.twist.linear.z;

  // Get transform from UTM to MAP
  try
  {
    this->listener_.lookupTransform(this->frame_id_, "utm", ros::Time(0), this->utm_trans_);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  // Convert it to Eigen
  Eigen::Affine3d UTM_to_map_rotation;
  tf::transformTFToEigen(this->utm_trans_, UTM_to_map_rotation);

  // Convert the velocities to "map" frame
  Eigen::Vector3d map_velocities = Eigen::Vector3d::Zero();
  map_velocities = UTM_to_map_rotation.linear() * UTM_velocities;

  // And pass it to the output message
  this->odom_gps_.twist.twist.linear.x = map_velocities(0);
  this->odom_gps_.twist.twist.linear.y = map_velocities(1);
  this->odom_gps_.twist.twist.linear.z = map_velocities(2);

  //////// Extract orientations in "map" frame /////////////////////////////////////////////
  Eigen::Vector3d map_orientations_RPY = Eigen::Vector3d::Zero();

  double vx = map_velocities(0); // just to improve readability
  double vy = map_velocities(1);
  double vz = map_velocities(2);

  if (sqrt(vx * vx + vy * vy) > this->min_speed_)
  {

    map_orientations_RPY(0) = 0.0; //We assume that the 3D movement of a ground vehicle is due to pitch and yaw only
    map_orientations_RPY(1) = -1.0 * atan2(vz, sqrt(vx * vx + vy * vy)); // The -1.0 is to point the heading direction
                                                                         // up when z > 0 (because positive pitch angles make
                                                                         // the nose go down when using a front (x) , left (y)
                                                                         // up (z) representation
    map_orientations_RPY(2) = atan2(vy, vx);

    // Converting to quarternion to fill the ROS message
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(map_orientations_RPY(0), map_orientations_RPY(1),
                                                            map_orientations_RPY(2));

    //std::cout << "Roll = " << map_orientations_RPY(0) * 180.0 / M_PI << "    Pitch = "
    //    << map_orientations_RPY(1) * 180.0 / M_PI << "    Yaw = " << map_orientations_RPY(2) * 180.0 / M_PI
    //    << std::endl;

    // Pass it to the output message
    this->odom_gps_.pose.pose.orientation.x = quaternion[0];
    this->odom_gps_.pose.pose.orientation.y = quaternion[1];
    this->odom_gps_.pose.pose.orientation.z = quaternion[2];
    this->odom_gps_.pose.pose.orientation.w = quaternion[3];

    // Now we need to compute the covariance matrix of those orientations,
    // as we have an explicit non-linear relation, we use the a first order approximation
    // to describe the variance propagation:

    // First, we need to apply the UTM to map transformation to the linear velocities covariance matrix
    // given by the input message (it will be just diagonal using an UBLOX M8P sensor)

    Eigen::Matrix3d UTM_vel_covariance = Eigen::Matrix3d::Zero();
    UTM_vel_covariance(0, 0) = vel_msg->twist.covariance[0];
    UTM_vel_covariance(0, 1) = vel_msg->twist.covariance[1];
    UTM_vel_covariance(0, 2) = vel_msg->twist.covariance[2];

    UTM_vel_covariance(1, 0) = vel_msg->twist.covariance[6];
    UTM_vel_covariance(1, 1) = vel_msg->twist.covariance[7];
    UTM_vel_covariance(1, 2) = vel_msg->twist.covariance[8];

    UTM_vel_covariance(2, 0) = vel_msg->twist.covariance[12];
    UTM_vel_covariance(2, 1) = vel_msg->twist.covariance[13];
    UTM_vel_covariance(2, 2) = vel_msg->twist.covariance[14];

    Eigen::Matrix3d map_vel_cov = Eigen::Matrix3d::Zero();
    map_vel_cov = UTM_to_map_rotation.linear() * UTM_vel_covariance * UTM_to_map_rotation.linear().transpose();
    // Explanation of the covariance rotation:
    // C = E(X*X^T)            --> Covariance definition
    // X'= R*X                 --> X' is X rotated using matrix R
    // C'= E(X'*X'^T)          --> and just making some substitution
    // C'= E(R*X * X^T*R^T)    --> we get to the
    // C'= R * E(X*X^T) * R^T  --> formula used above

    // And we can now pass this linear velocity covariance matrix to the output message
    this->odom_gps_.twist.covariance[0] = map_vel_cov(0, 0);
    this->odom_gps_.twist.covariance[1] = map_vel_cov(0, 1);
    this->odom_gps_.twist.covariance[2] = map_vel_cov(0, 2);

    this->odom_gps_.twist.covariance[6] = map_vel_cov(1, 0);
    this->odom_gps_.twist.covariance[7] = map_vel_cov(1, 1);
    this->odom_gps_.twist.covariance[8] = map_vel_cov(1, 2);

    this->odom_gps_.twist.covariance[12] = map_vel_cov(2, 0);
    this->odom_gps_.twist.covariance[13] = map_vel_cov(2, 1);
    this->odom_gps_.twist.covariance[14] = map_vel_cov(2, 2);

    // now we calcule variance of yaw (DEPENDING ON VELOCITY MAGNITUDE!!!)
    double speed = sqrt(vx * vx + vy * vy);
    double min_variance_yaw = (2 * 3.1416) / 180.0;
    double max_variance_yaw = (360 * 3.1416) / 180.0;
    double dif_variance_yaw = max_variance_yaw - min_variance_yaw;
    double variance_yaw = max_variance_yaw - dif_variance_yaw * (speed / this->max_speed_);

    if (variance_yaw < min_variance_yaw)
    {
      variance_yaw = min_variance_yaw;
    }

    // To compute the orientation covariance (in this case DEPENDING ON VELOCITIES COV!!!)
    // we need the jacobian of the function that maps velocities to orientation
    Eigen::Matrix3d map_orientation_RPY_jacobian = Eigen::Matrix3d::Zero();

    double mod_xy = sqrt(vx * vx + vy * vy);
    double mod_xyz_squared = vx * vx + vy * vy + vz * vz;

    // Roll is set as constant, so any derivative is just zero
    map_orientation_RPY_jacobian(0, 0) = 0.0;                     // partial derivative of roll wrt vx
    map_orientation_RPY_jacobian(0, 1) = 0.0;                     // partial derivative of roll wrt vy
    map_orientation_RPY_jacobian(0, 2) = 0.0;                     // partial derivative of roll wrt vz

    map_orientation_RPY_jacobian(1, 0) = (vz * vx) / (mod_xy * mod_xyz_squared); // partial derivative of pitch wrt vx
    map_orientation_RPY_jacobian(1, 1) = (vz * vy) / (mod_xy * mod_xyz_squared); // partial derivative of pitch wrt vy
    map_orientation_RPY_jacobian(1, 2) = -1.0 * mod_xy / mod_xyz_squared;        // partial derivative of pitch wrt vz

    map_orientation_RPY_jacobian(2, 0) = -1 * vy / mod_xy; // partial derivative of yaw wrt vx
    map_orientation_RPY_jacobian(2, 1) = vx / mod_xy; // partial derivative of yaw wrt vy
    map_orientation_RPY_jacobian(2, 2) = 0.0;            // partial derivative of yaw wrt vz

    // Finally, we compute the first order approximation
    Eigen::Matrix3d map_orientation_RPY_cov = Eigen::Matrix3d::Zero();
    map_orientation_RPY_cov = map_orientation_RPY_jacobian * map_vel_cov * map_orientation_RPY_jacobian.transpose();

    // Passing to ROS message
    this->odom_gps_.pose.covariance[21] = map_orientation_RPY_cov(0, 0);
    this->odom_gps_.pose.covariance[22] = map_orientation_RPY_cov(0, 1);
    this->odom_gps_.pose.covariance[23] = map_orientation_RPY_cov(0, 2);

    this->odom_gps_.pose.covariance[27] = map_orientation_RPY_cov(1, 0);
    this->odom_gps_.pose.covariance[28] = map_orientation_RPY_cov(1, 1);
    this->odom_gps_.pose.covariance[29] = map_orientation_RPY_cov(1, 2);

    this->odom_gps_.pose.covariance[33] = map_orientation_RPY_cov(2, 0);
    this->odom_gps_.pose.covariance[34] = map_orientation_RPY_cov(2, 1);
    this->odom_gps_.pose.covariance[35] = map_orientation_RPY_cov(2, 2) + variance_yaw;

  }

  this->flag_gnss_velocity_received_ = true;
  this->alg_.unlock();
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void GpsToOdomAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_ = config;
  this->alg_.unlock();
}

void GpsToOdomAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc, char *argv[])
{
  return algorithm_base::main < GpsToOdomAlgNode > (argc, argv, "gps_to_odom_alg_node");
}
