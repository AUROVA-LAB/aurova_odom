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
 * \file gps_to_odom_alg_node.h
 *
 *  Created on: 04 Dec 2018
 *      Author: m.a.munoz
 */

#ifndef _gps_to_odom_alg_node_h_
#define _gps_to_odom_alg_node_h_


#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <Eigen/Dense>
#include "math.h"
#include "planning/graph.h"
#include <chrono>

// [publisher subscriber headers]

// [service client headers]

// [action server client headers]

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class GpsToOdomAlgNode : public rclcpp::Node
{
private:

  bool flag_publish_odom_;
  bool flag_gnss_position_received_;
  bool flag_gnss_velocity_received_;
  float max_speed_;
  float min_speed_;
  std::string frame_id_;
  nav_msgs::msg::Odometry odom_gps_;
  geometry_msgs::msg::TransformStamped utm_trans_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

  // [publisher attributes]
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_gps_pub_;

  // [subscriber attributes]
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sim_fix_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sim_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr bot_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr nco_vel_sub_; // nco = no covariance


	/**
   * \brief callback for read gps fix messages (from gazebo)
   * This message can be read from different localization sources by remapping in the
   * execution of the node.
   */
  void cb_getSimGpsFixMsg(const sensor_msgs::msg::NavSatFix::SharedPtr  fix_msg);
  
  /**
   * \brief callback for read fix velocity messages (from gazebo)
   * This message can be read from different localization sources by remapping in the
   * execution of the node.
   */
  void cb_getSimGpsVelMsg(const geometry_msgs::msg::Vector3Stamped::SharedPtr  vel_msg);
  
  /**
   * \brief callback for read fix velocity messages (from sensor)
   * This message can be read from different localization sources by remapping in the
   * execution of the node.
   */
  void cb_getBotGpsVelMsg(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr  vel_msg);

    /**
   * \brief callback for read fix velocity messages (from sensor)
   * This message can be read from different localization sources by remapping in the
   * execution of the node.
   */
  void cb_getNcoGpsVelMsg(const geometry_msgs::msg::TwistStamped::SharedPtr  vel_msg);

  // [service attributes]

  // [client attributes]

  // [action server attributes]

  // [action client attributes]


public:
  std::shared_ptr<rclcpp::Rate> loop_rate;
  double rate;

  /**
   * \brief Constructor
   *
   * This constructor initializes specific class attributes and all ROS
   * communications variables to enable message exchange.
   */
  GpsToOdomAlgNode(void);

  /**
   * \brief Destructor
   *
   * This destructor frees all necessary dynamic memory allocated within this
   * this class.
   */
  ~GpsToOdomAlgNode(void);

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
   */
  void mainNodeThread(void);

protected:
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
		rcl_interfaces::msg::SetParametersResult node_config_update(const std::vector<rclcpp::Parameter> & parameters);

  // [diagnostic functions]

  // [test functions]
};

#endif
