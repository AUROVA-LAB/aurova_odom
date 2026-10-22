// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

// Author of Lilo: Edison Velasco 
// Email evs25@alu.ua.es

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros lib
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>


//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "lidar.h"
#include "odomEstimationClass.h"

OdomEstimationClass odomEstimation;
std::mutex mutex_lock;
std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> pointCloudEdgeBuf;
std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> pointCloudSurfBuf;

std::string baseframeID = "base_link";
std::string edge_pcl = "/pcl_edge";
std::string surf_pcl = "/pcl_surf";

lidar::Lidar lidar_param;

rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometry;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdometryDiff;
rclcpp::Node::SharedPtr nh;
std::shared_ptr<tf2_ros::Buffer> tf_buffer_{nullptr};
std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
std::shared_ptr<tf2_ros::TransformBroadcaster> br{nullptr};

rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr time_average;


void velodyneSurfHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudSurfBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}
void velodyneEdgeHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg)
{

    mutex_lock.lock();
    pointCloudEdgeBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}

bool is_odom_inited = false;
double total_time =0, cropBox_len, surf_limit;
int total_frame=0;
bool clear_map;
void odom_estimation(){

    float time_delay  = 0;

    Eigen::Quaterniond q_diff;
    Eigen::Vector3d t_diff;
    Eigen::Isometry3d odom_prev = Eigen::Isometry3d::Identity();;


    while(1){
        if(!pointCloudEdgeBuf.empty() && !pointCloudSurfBuf.empty()){

            mutex_lock.lock();

            pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*pointCloudEdgeBuf.front(), *pointcloud_edge_in);
            pcl::fromROSMsg(*pointCloudSurfBuf.front(), *pointcloud_surf_in);
            rclcpp::Time pointcloud_time = (pointCloudSurfBuf.front())->header.stamp;
            std::string lidar_frame = pointCloudEdgeBuf.front()->header.frame_id;
            pointCloudEdgeBuf.pop();
            pointCloudSurfBuf.pop();
            mutex_lock.unlock();

            if(is_odom_inited == false){
                odomEstimation.initMapWithPoints(pointcloud_edge_in, pointcloud_surf_in);
                is_odom_inited = true;
                RCLCPP_INFO(nh->get_logger(),"odom inited");
            }else{
                std::chrono::time_point<std::chrono::system_clock> start, end;
                start = std::chrono::system_clock::now();
                odomEstimation.updatePointsToMap(pointcloud_edge_in, pointcloud_surf_in, clear_map, cropBox_len);
                end = std::chrono::system_clock::now();
                std::chrono::duration<float> elapsed_seconds = end - start;
                total_frame++;
                float time_temp = elapsed_seconds.count() * 1000.0;
                total_time+=time_temp;
                time_delay = total_time/total_frame;
                RCLCPP_INFO(nh->get_logger(),"average odom estimation time %f mS", time_delay);
                time_delay = time_delay/1000.0;
            }


            Eigen::Quaterniond q_current(odomEstimation.odom.rotation());
            Eigen::Vector3d t_current = odomEstimation.odom.translation();

            ///////////////////////////////////////////////////////
            // Project to 2D!!!
            t_current.z() = 0.0;
            double siny_cosp = 2 * (q_current.w() * q_current.z() + q_current.x() * q_current.y());
            double cosy_cosp = 1 - 2 * (q_current.y() * q_current.y() + q_current.z() * q_current.z());
            double yaw = std::atan2(siny_cosp, cosy_cosp);
            Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
            q_current = yaw_angle;
            ///////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////
            tf2::Transform T_odom_lidar;
            T_odom_lidar.setOrigin( tf2::Vector3(t_current.x(), t_current.y(), t_current.z()) );
            tf2::Quaternion q(q_current.x(),q_current.y(),q_current.z(),q_current.w());
            T_odom_lidar.setRotation(q);

            geometry_msgs::msg::TransformStamped tf_lidar2base;
            try
            {
            tf_lidar2base = tf_buffer_->lookupTransform(lidar_frame,  baseframeID, pointcloud_time);
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(nh->get_logger(),"TF exception lidar2base:\n%s", ex.what());
            }
            tf2::Transform T_lidar_base;
            tf2::fromMsg(tf_lidar2base.transform, T_lidar_base);
            tf2::Transform T_odom_base = T_odom_lidar * T_lidar_base;
            
            geometry_msgs::msg::TransformStamped tf_odom2base;
            tf_odom2base.header.stamp = pointcloud_time;
            tf_odom2base.header.frame_id = "odom";
            tf_odom2base.child_frame_id = baseframeID;
            tf_odom2base.transform = tf2::toMsg(T_odom_base); 
            
            br->sendTransform(tf_odom2base);
          
            Eigen::Isometry3d odom = Eigen::Isometry3d::Identity();
            odom.linear() = q_current.toRotationMatrix();
            odom.translation() = t_current;

            Eigen::Isometry3d odomdiff = (odom_prev.inverse() * odom);
            q_diff = Eigen::Quaterniond(odomdiff.rotation());
            t_diff = odomdiff.translation();
           
            // Eigen::Isometry3d odom_curr = odom_prev * odomdiff;
            // Eigen::Quaterniond q_c = Eigen::Quaterniond(odom_curr.rotation());;
            // Eigen::Vector3d t_c = odom_curr.translation();

            odom_prev = odom;


             
            // publish odometry
            nav_msgs::msg::Odometry laserOdometry;
            laserOdometry.header.frame_id = "odom";
            laserOdometry.child_frame_id = lidar_frame;
            laserOdometry.header.stamp = pointcloud_time;
            laserOdometry.pose.pose.orientation.x = q_current.x();
            laserOdometry.pose.pose.orientation.y = q_current.y();
            laserOdometry.pose.pose.orientation.z = q_current.z();
            laserOdometry.pose.pose.orientation.w = q_current.w();
            laserOdometry.pose.pose.position.x = t_current.x();
            laserOdometry.pose.pose.position.y = t_current.y();
            laserOdometry.pose.pose.position.z = t_current.z();

            nav_msgs::msg::Odometry odomDiff;
            odomDiff.header.frame_id = "odom";
            odomDiff.child_frame_id = lidar_frame;
            odomDiff.header.stamp = pointcloud_time;
            odomDiff.pose.pose.orientation.x = q_diff.x();
            odomDiff.pose.pose.orientation.y = q_diff.y();
            odomDiff.pose.pose.orientation.z = q_diff.z();
            odomDiff.pose.pose.orientation.w = q_diff.w();
            odomDiff.pose.pose.position.x = t_diff.x();
            odomDiff.pose.pose.position.y = t_diff.y();
            odomDiff.pose.pose.position.z = t_diff.z();



            for(int i = 0; i<36; i++) {
              if(i == 0 || i == 7 || i == 14) {
                laserOdometry.pose.covariance[i] = .01;
               }
               else if (i == 21 || i == 28 || i== 35) {
                 laserOdometry.pose.covariance[i] += 0.1;
               }
               else {
                 laserOdometry.pose.covariance[i] = 0;
               }
            }

            pubLaserOdometry->publish(laserOdometry);
            pubOdometryDiff->publish(odomDiff);

            //publish time

            std_msgs::msg::Float64 time_msg;
            time_msg.data = time_delay*1000.0;
            time_average->publish(time_msg);

         }

        std::chrono::milliseconds dura(2);
        rclcpp::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    nh = std::make_shared<rclcpp::Node>("odom_estimation");


    int scan_line = 16;
    double scan_period= 0.1;
    double vertical_angle = 0.0;    
    double max_dis = 60.0;
    double min_dis =0;
    double edge_resolution = 0.3;
    double surf_resolution = 0.6;
    bool validation_angle = false;

    clear_map = true;
    cropBox_len = 10000;

    nh->declare_parameter("scan_period", scan_period); 
    nh->get_parameter("scan_period", scan_period); 
    nh->declare_parameter("vertical_angle", vertical_angle ); 
    nh->get_parameter("vertical_angle", vertical_angle); 
    nh->declare_parameter("max_dis", max_dis);
    nh->get_parameter("max_dis", max_dis);
    nh->declare_parameter("min_dis", min_dis);
    nh->get_parameter("min_dis", min_dis);
    nh->declare_parameter("scan_line", scan_line);
    nh->get_parameter("scan_line", scan_line);
    nh->declare_parameter("edge_resolution", edge_resolution);
    nh->get_parameter("edge_resolution", edge_resolution);
    nh->declare_parameter("surf_resolution", surf_resolution);
    nh->get_parameter("surf_resolution", surf_resolution);
    nh->declare_parameter("clear_map", clear_map);
    nh->get_parameter("clear_map", clear_map);

    nh->declare_parameter("cropBox_len", cropBox_len);
    nh->get_parameter("cropBox_len", cropBox_len);

    nh->declare_parameter("baseframeID", baseframeID);
    nh->get_parameter("baseframeID",baseframeID);
    nh->declare_parameter("pcl_edge", edge_pcl);
    nh->get_parameter("pcl_edge",edge_pcl);
    nh->declare_parameter("pcl_surf", surf_pcl);
    nh->get_parameter("pcl_surf",surf_pcl);

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setValidationAngle(validation_angle);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    odomEstimation.init(lidar_param, edge_resolution, surf_resolution);
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subEdgeLaserCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>(edge_pcl, 1,std::bind(&velodyneEdgeHandler, std::placeholders::_1));
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subSurfLaserCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>(surf_pcl, 1,std::bind(&velodyneSurfHandler, std::placeholders::_1));

    pubLaserOdometry = nh->create_publisher<nav_msgs::msg::Odometry>("/odom", 1);
    pubOdometryDiff = nh->create_publisher<nav_msgs::msg::Odometry>("/odom_diff", 1);
    time_average = nh->create_publisher<std_msgs::msg::Float64>("/time_average", 1);

    br = std::make_unique<tf2_ros::TransformBroadcaster>(nh);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(nh->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    std::thread odom_estimation_process{odom_estimation};

    rclcpp::spin(nh);

    return 0;
}
