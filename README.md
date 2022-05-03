### Example of usage:

You can run an example following the instructions in: [application_navigation](https://github.com/AUROVA-LAB/application_navigation) and [application_localization](https://github.com/AUROVA-LAB/application_localization).

# aurova_odom
This is a metapackage that contains different packages that perform different kind of odometry nodes. Compiling this metapackage into ROS will compile all the packages at once. This metapackage is grouped as a project for eclipse C++. Each package contains a "name_doxygen_config" configuration file for generate doxygen documentation. The packages contained in this metapackage are:

**gps_to_odom**
This package contains a node that, as input, reads the topics /fix, of type sensor_msgs::NavSatFix, and /rover/fix_velocity of type geometry_msgs::TwistWithCovariance (if from gazebo /fix_vel of type geometry_msgs::Vector3Stamped). This node transform gps location from /fix in utm frame to a frame defined in ~gps_to_odom/frame_id. Also calculates the orientation using the velocities from gps, and generate new odometry (message type nav_msgs::Odometry). This message is published in output topic called /odometry_gps.

Parameters:
* ~gps_to_odom/frame_id (default: ""): This is the frame to transform from utm (usually set as "map").
* ~gps_to_odom/min_speed (default: null): This is used as a threshold. Under this velocity, the orientation is not computed from the GPS velocity message.
* ~gps_to_odom/max_speed (default: null): We add additive variance to the orientation message inversely proportional to this parameter.
 
**ackermann_to_odom**
This package contains a node that, as input, reads the topics /estimated_ackermann_state and /covariance_ackermann_state, of type ackermann_msgs::AckermannDriveStamped, and /virtual_imu_data of type sensor_msgs::Imu. This node parse this information as a new message type nav_msgs::Odometry using the 2D tricicle model. This message is published in an output topic called /odometry.

Parameters:
* ~odom_in_tf (default: false): If this parameter is set to true, the odometry is also published in /tf topic.
* ~scan_in_tf (default: false): If this parameter is set to true, the laser transform read from the static robot transformation is published in /tf topic.
* ~frame_id (default: ""): This parameter is the name of frame to transform if scan_in_tf is true.
* ~child_id (default: ""): This parameter is the name of child frame to transform if scan_in_tf is true.
