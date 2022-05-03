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
 * \file ackermann_to_odom_alg.h
 *
 *  Created on: 04 Sep 2018
 *      Author: m.a.munoz
 */

#ifndef _ackermann_to_odom_alg_h_
#define _ackermann_to_odom_alg_h_

#include <ackermann_to_odom/AckermannToOdomConfig.h>
#include "ackermann_to_odom_alg.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_broadcaster.h>

#define MAX_DIFF 1.5

//include ackermann_to_odom_alg main library

/**
 * \brief IRI ROS Specific Driver Class
 *
 * This class implements the main algorithm that performs the package.
 */
class AckermannToOdomAlgorithm
{
protected:
  /**
   * \brief define config type
   *
   * Define a Config type with the AckermannToOdomConfig. All driver implementations
   * will then use the same variable type Config.
   */
  pthread_mutex_t access_;

  // private attributes and methods

public:
  /**
   * \brief define config type
   *
   * Define a Config type with the AckermannToOdomConfig. All driver implementations
   * will then use the same variable type Config.
   */
  typedef ackermann_to_odom::AckermannToOdomConfig Config;

  /**
   * \brief config variable
   *
   * This variable has all the driver parameters defined in the cfg config file.
   * Is updated everytime function config_update() is called.
   */
  Config config_;

  /**
   * \brief constructor
   *
   * In this constructor parameters related to the specific driver can be
   * initalized. Those parameters can be also set in the openDriver() function.
   * Attributes from the main node driver class IriBaseDriver such as loop_rate,
   * may be also overload here.
   */
  AckermannToOdomAlgorithm(void);

  /**
   * \brief Lock Algorithm
   *
   * Locks access to the Algorithm class
   */
  void lock(void)
  {
    pthread_mutex_lock(&this->access_);
  }
  ;

  /**
   * \brief Unlock Algorithm
   *
   * Unlocks access to the Algorithm class
   */
  void unlock(void)
  {
    pthread_mutex_unlock(&this->access_);
  }
  ;

  /**
   * \brief Tries Access to Algorithm
   *
   * Tries access to Algorithm
   *
   * \return true if the lock was adquired, false otherwise
   */
  bool try_enter(void)
  {
    if (pthread_mutex_trylock(&this->access_) == 0)
      return true;
    else
      return false;
  }
  ;

  /**
   * \brief config update
   *
   * In this function the driver parameters must be updated with the input
   * config variable. Then the new configuration state will be stored in the
   * Config attribute.
   *
   * \param new_cfg the new driver configuration state
   *
   * \param level level in which the update is taken place
   */
  void config_update(Config& config, uint32_t level = 0);

  // here define all ackermann_to_odom_alg interface methods to retrieve and set
  // the driver parameters

  /**
   * \brief Destructor
   *
   * This destructor is called when the object is about to be destroyed.
   *
   */
  ~AckermannToOdomAlgorithm(void);

  /**
   * \brief Generate new message of odometry
   *
   * This function gets information of the ackermann state, and the imu, and parse this to new topic
   * of odometry type.
   *
   * @param estimated_ackermann_state is the state of the robot.
   * @param covariance is the covariance value of the ackermann state.
   * @param virtual_imu_ms is the message of the imu sensor.
   * @param odometry is the output of the function. Is odometry message.
   * @param odom_trans is the odometry transform in /tf message.
   * @param base_trans is the laser transform in /tf message.
   */
  void generateNewOdometryMsg2D(ackermann_msgs::AckermannDriveStamped estimated_ackermann_state,
                                sensor_msgs::Imu virtual_imu_ms,
                                geometry_msgs::PoseWithCovarianceStamped& odometry_pose, nav_msgs::Odometry& odometry,
                                geometry_msgs::TransformStamped& odom_trans);
};

#endif
