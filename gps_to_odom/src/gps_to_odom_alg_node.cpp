#include "gps_to_odom_alg_node.h"

GpsToOdomAlgNode::GpsToOdomAlgNode(void) :
    Node("gpstoodom")
{
  broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  //init class attributes if necessary
  this->flag_publish_odom_ = false;
  this->flag_gnss_position_received_ = false;
  this->flag_gnss_velocity_received_ = false;
  //this->loop_rate_ = 10; //in [Hz]
  this->declare_parameter("frame_id", "map");
  this->get_parameter("frame_id", this->frame_id_);
  this->declare_parameter("min_speed", 0.4);
  this->get_parameter("min_speed", this->min_speed_);
  this->declare_parameter("max_speed", 1.3);
  this->get_parameter("max_speed", this->max_speed_);
  auto parameter_callback = this->add_on_set_parameters_callback(std::bind(&GpsToOdomAlgNode::node_config_update, this, std::placeholders::_1));


  this->odom_gps_.pose.pose.orientation.x = 0.0;
  this->odom_gps_.pose.pose.orientation.y = 0.0;
  this->odom_gps_.pose.pose.orientation.z = 0.0;
  this->odom_gps_.pose.pose.orientation.w = 1.0;

  // [init publishers]
  this->odom_gps_pub_ = this->create_publisher < nav_msgs::msg::Odometry > ("/odometry_gps", 1);

  // [init create_subscription<msgs::msg::YourMessageType>rs]
  this->sim_fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/fix", 1, std::bind(&GpsToOdomAlgNode::cb_getSimGpsFixMsg, this, std::placeholders::_1));
  this->sim_vel_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("/fix_vel", 1, std::bind(&GpsToOdomAlgNode::cb_getSimGpsVelMsg, this, std::placeholders::_1));
  this->bot_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>("/rover/fix_velocity", 1, std::bind(&GpsToOdomAlgNode::cb_getBotGpsVelMsg, this, std::placeholders::_1));
  this->nco_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("/vel", 1, std::bind(&GpsToOdomAlgNode::cb_getNcoGpsVelMsg, this, std::placeholders::_1));

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
    this->odom_gps_pub_->publish(this->odom_gps_);
    this->flag_gnss_position_received_ = false;
    this->flag_gnss_velocity_received_ = false;
  }

}

/*  [create_subscription<msgs::msg::YourMessageType>r callbacks] */
void GpsToOdomAlgNode::cb_getSimGpsFixMsg(const sensor_msgs::msg::NavSatFix::SharedPtr fix_msg)
{

  Ellipsoid utm;
  double utm_x;
  double utm_y;
  char utm_zone[30];
  int ref_ellipsoid = 23;

  utm.LLtoUTM(ref_ellipsoid, fix_msg->latitude, fix_msg->longitude, utm_y, utm_x, utm_zone);

  ///////////////////////////////////////////////////////////
  ///// TRANSFORM TO TF FARME
  geometry_msgs::msg::PointStamped fix_tf;
  geometry_msgs::msg::PointStamped fix_utm;
  fix_utm.header.frame_id = "utm";
  fix_utm.header.stamp = rclcpp::Time(0); //this->now();
  fix_utm.point.x = utm_x;
  fix_utm.point.y = utm_y;
  fix_utm.point.z = 0.0;
  try
  {
    this->tf_buffer_->transform( fix_utm,  fix_tf, this->frame_id_);

  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN(this->get_logger(),"[draw_frames] TF exception:\n%s", ex.what());
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

}

void GpsToOdomAlgNode::cb_getSimGpsVelMsg(const geometry_msgs::msg::Vector3Stamped::SharedPtr vel_msg)
{


  // no transform because the orientation is in north of frame "world" that coincides with frame "map" 
  this->odom_gps_.twist.twist.linear.x = vel_msg->vector.x;
  this->odom_gps_.twist.twist.linear.y = vel_msg->vector.y;
  this->odom_gps_.twist.twist.linear.z = vel_msg->vector.z;

  double yaw = atan2(vel_msg->vector.y, vel_msg->vector.x);
  tf2::Quaternion quat_world; quat_world.setRPY(0, 0, yaw);

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


}

void GpsToOdomAlgNode::cb_getBotGpsVelMsg(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr vel_msg)
{


  //////////////////// Extract velocities in "map" frame /////////////////////////////////
  // Get the velocities expressed in UTM from the message
  Eigen::Vector3d UTM_velocities = Eigen::Vector3d::Zero();
  UTM_velocities(0) = vel_msg->twist.twist.linear.x; // In UTM frame
  UTM_velocities(1) = vel_msg->twist.twist.linear.y;
  UTM_velocities(2) = vel_msg->twist.twist.linear.z;

  // Get transform from UTM to MAP
  try
  {
     this->utm_trans_ = this->tf_buffer_->lookupTransform(this->frame_id_,  "utm", tf2::TimePointZero);

  }
  catch (tf2::TransformException ex)
  {
    RCLCPP_ERROR(this->get_logger(),"%s", ex.what());
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  // Convert it to Eigen
  Eigen::Affine3d UTM_to_map_rotation;
  UTM_to_map_rotation = tf2::transformToEigen(this->utm_trans_);


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
    tf2::Quaternion quaternion; quaternion.setRPY(map_orientations_RPY(0), map_orientations_RPY(1),
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

}

void GpsToOdomAlgNode::cb_getNcoGpsVelMsg(const geometry_msgs::msg::TwistStamped::SharedPtr vel_msg)
{


  //////////////////// Extract velocities in "map" frame /////////////////////////////////
  // Get the velocities expressed in UTM from the message
  Eigen::Vector3d UTM_velocities = Eigen::Vector3d::Zero();
  UTM_velocities(0) = vel_msg->twist.linear.x; // In UTM frame
  UTM_velocities(1) = vel_msg->twist.linear.y;
  UTM_velocities(2) = vel_msg->twist.linear.z;

  // Get transform from UTM to MAP
  try
  {
     this->utm_trans_ = this->tf_buffer_->lookupTransform(this->frame_id_,  "utm", tf2::TimePointZero);

  }
  catch (tf2::TransformException ex)
  {
    RCLCPP_ERROR(this->get_logger(),"%s", ex.what());
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  // Convert it to Eigen
  Eigen::Affine3d UTM_to_map_rotation;
   UTM_to_map_rotation = tf2::transformToEigen(this->utm_trans_);


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
    tf2::Quaternion quaternion; quaternion.setRPY(map_orientations_RPY(0), map_orientations_RPY(1),
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

    // WE PUT COVARIANCES BY HAND BECAUSE THIS MESSAGE IS WITH NO-COVARIANCE

    Eigen::Matrix3d UTM_vel_covariance = Eigen::Matrix3d::Zero();
    UTM_vel_covariance(0, 0) = 0.1;
    UTM_vel_covariance(0, 1) = 0.0;
    UTM_vel_covariance(0, 2) = 0.0;

    UTM_vel_covariance(1, 0) = 0.0;
    UTM_vel_covariance(1, 1) = 0.1;
    UTM_vel_covariance(1, 2) = 0.0;

    UTM_vel_covariance(2, 0) = 0.0;
    UTM_vel_covariance(2, 1) = 0.0;
    UTM_vel_covariance(2, 2) = 0.1;

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

  ///////////////////////////////////////////////////////////////////////////////
  //// FOR REPRESENTATION PURPO0SES
  map_orientations_RPY(0) = 0.0;
  map_orientations_RPY(1) = -1.57;
  map_orientations_RPY(2) = 0.0;

  // Converting to quarternion to fill the ROS message
  tf2::Quaternion quaternion; quaternion.setRPY(map_orientations_RPY(0), map_orientations_RPY(1),
                                                          map_orientations_RPY(2));
      // Pass it to the output message
  this->odom_gps_.pose.pose.orientation.x = quaternion[0];
  this->odom_gps_.pose.pose.orientation.y = quaternion[1];
  this->odom_gps_.pose.pose.orientation.z = quaternion[2];
  this->odom_gps_.pose.pose.orientation.w = quaternion[3];
  ///////////////////////////////////////////////////////////////////////////////

  this->flag_gnss_velocity_received_ = true;

}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

rcl_interfaces::msg::SetParametersResult GpsToOdomAlgNode::node_config_update(const std::vector<rclcpp::Parameter> &parameters)
{
  this->get_parameter("/gps_to_odom/frame_id", this->frame_id_);
  this->get_parameter("/gps_to_odom/min_speed", this->min_speed_);
  this->get_parameter("/gps_to_odom/max_speed", this->max_speed_);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}


/* main function */
int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared< GpsToOdomAlgNode >();
	node->rate = node->declare_parameter<double>("rate", 10.0);
	node->loop_rate = std::make_shared<rclcpp::Rate>(node->rate);

	while (rclcpp::ok()) {
		node->mainNodeThread();
		rclcpp::spin_some(node);
		node->loop_rate->sleep();
	}
	rclcpp::shutdown();
	return 0;
}
