// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author 
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// IMPORTANT NOTE: This code has been generated through a script from the 
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab
/**
 * \file ackermann_to_odom_alg_node.h
 *
 *  Created on: 04 Sep 2018
 *      Author: m.a.munoz
 */

#ifndef _ackermann_to_odom_alg_node_h_
#define _ackermann_to_odom_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "ackermann_to_odom_alg.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

// [publisher subscriber headers]

// [service client headers]

// [action server client headers]

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 * Interface with ROS. In this class we specify publishers, subscribers, and callbacks.
 */
class AckermannToOdomAlgNode : public algorithm_base::IriBaseAlgorithm<AckermannToOdomAlgorithm>
{
private:

  ackermann_msgs::AckermannDriveStamped estimated_ackermann_state_;
  sensor_msgs::Imu virtual_imu_msg_;
  geometry_msgs::TransformStamped odom_trans_;
  tf::StampedTransform scan_trans_;
  tf::TransformBroadcaster broadcaster_;
  tf::TransformListener listener_;
  nav_msgs::Odometry odometry_;
  geometry_msgs::PoseWithCovarianceStamped odometry_pose_;

  // [publisher attributes]
  ros::Publisher odometry_publisher_;
  ros::Publisher pose_publisher_;

  // [subscriber attributes]
  ros::Subscriber estimated_ackermann_subscriber_;
  ros::Subscriber covariance_ackermann_subscriber_;
  ros::Subscriber virtual_imu_subscriber_;

  /**
   * \brief Callback for read imu messages.
   */
  void cb_imuData(const sensor_msgs::Imu::ConstPtr& Imu_msg);

  /**
   * \brief Callback for read ackermann messages.
   */
  void cb_ackermannState(const ackermann_msgs::AckermannDriveStamped::ConstPtr& estimated_ackermann_state_msg);

  // [service attributes]

  // [client attributes]

  // [action server attributes]

  // [action client attributes]

  /**
   * \brief config variable
   *
   * This variable has all the driver parameters defined in the cfg config file.
   * Is updated everytime function config_update() is called.
   */
  Config config_;
public:
  /**
   * \brief Constructor
   *
   * This constructor initializes specific class attributes and all ROS
   * communications variables to enable message exchange.
   */
  AckermannToOdomAlgNode(void);

  /**
   * \brief Destructor
   *
   * This destructor frees all necessary dynamic memory allocated within this
   * this class.
   */
  ~AckermannToOdomAlgNode(void);

protected:
  /**
   * \brief main node thread
   *
   * This is the main thread node function. Code written here will be executed
   * in every node loop while the algorithm is on running state. Loop frequency
   * can be tuned by modifying loop_rate attribute.
   *
   * Here data related to the process loop or to ROS topics (mainly data structs
   * related to the MSG and SRV files) must be updated. ROS publisher objects
   * must publish their data in this process. ROS client servers may also
   * request data to the corresponding server topics.
   *
   * In this point we need to read some parameters from rosparam:
   *
   * @param odom_in_tf (bool) if this parameter is true, the odometry will publish in the /tf messages
   * @param x_tf (float) is the transform of the laser in x axis.
   * @param yaw_tf (float) is the transform of the laser in yaw angle.
   * @param frame_id (string) is the name of origin frame.
   * @param child_id (string) is the name of laser frame.
   *
   */
  void mainNodeThread(void);

  /**
   * \brief dynamic reconfigure server callback
   *
   * This method is called whenever a new configuration is received through
   * the dynamic reconfigure. The derivated generic algorithm class must
   * implement it.
   *
   * \param config an object with new configuration from all algorithm
   *               parameters defined in the config file.
   * \param level  integer referring the level in which the configuration
   *               has been changed.
   */
  void node_config_update(Config &config, uint32_t level);

  /**
   * \brief node add diagnostics
   *
   * In this abstract function additional ROS diagnostics applied to the
   * specific algorithms may be added.
   */
  void addNodeDiagnostics(void);

  // [diagnostic functions]

  // [test functions]
};

#endif
