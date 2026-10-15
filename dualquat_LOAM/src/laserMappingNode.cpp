// The mapping process is based on the FLOAM code.
// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Author of LiDAR-Odometry_DQ: Edison Velasco
// Email edison.velasco@ua.es

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
#include <nav_msgs/msg/odometry.hpp>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "laserMappingClass.h"


LaserMappingClass laserMapping;
std::mutex mutex_lock;
std::queue<nav_msgs::msg::Odometry::SharedPtr> odometryBuf;
std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> pointCloudBuf;
float scan_period = 0.1; // time in seconds

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub;
void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    mutex_lock.lock();
    odometryBuf.push(msg);
    mutex_lock.unlock();
}

void velodyneHandler(const sensor_msgs::msg::PointCloud2SharedPtr laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}


void laser_mapping(){
    while(1){
        if(!odometryBuf.empty() && !pointCloudBuf.empty()){

            //read data
            mutex_lock.lock();
            if(!pointCloudBuf.empty() && pointCloudBuf.front()->header.stamp.seconds()<odometryBuf.front()->header.stamp.seconds()-0.5*scan_period){
                RCLCPP_INFO(nh->get_logger(),"time stamp unaligned error and pointcloud discarded, pls check your data --> laser mapping node"); 
                pointCloudBuf.pop();
                mutex_lock.unlock();
                continue;              
            }

            if(!odometryBuf.empty() && odometryBuf.front()->header.stamp.seconds() < pointCloudBuf.front()->header.stamp.seconds()-0.5*scan_period){
                odometryBuf.pop();
                RCLCPP_INFO(nh->get_logger(),"time stamp unaligned with path final, pls check your data --> laser mapping node");
                mutex_lock.unlock();
                continue;  
            }

            //if time aligned 
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
            rclcpp::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;

            Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
            current_pose.rotate(Eigen::Quaterniond(odometryBuf.front()->pose.pose.orientation.w,odometryBuf.front()->pose.pose.orientation.x,odometryBuf.front()->pose.pose.orientation.y,odometryBuf.front()->pose.pose.orientation.z));  
            current_pose.pretranslate(Eigen::Vector3d(odometryBuf.front()->pose.pose.position.x,odometryBuf.front()->pose.pose.position.y,odometryBuf.front()->pose.pose.position.z));
            pointCloudBuf.pop();
            odometryBuf.pop();
            mutex_lock.unlock();
            
            laserMapping.updateCurrentPointsToMap(pointcloud_in,current_pose);

            pcl::PointCloud<pcl::PointXYZI>::Ptr pc_map = laserMapping.getMap();
            sensor_msgs::msg::PointCloud2 PointsMsg;
            pcl::toROSMsg(*pc_map, PointsMsg);
            PointsMsg.header.stamp = pointcloud_time;
            PointsMsg.header.frame_id = "map";
            map_pub->publish(PointsMsg); 
            
        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    nh = std::make_shared<rclcpp::Node>("LaserMapping");


    double scan_period= 0.5;
    double max_dis = 100.0;
    double min_dis = 2.0;
    double map_resolution = 0.2;

    nh->declare_parameter("scan_period",scan_period);
    nh->get_parameter("scan_period", scan_period);
    nh->declare_parameter("max_dis", max_dis);
    nh->get_parameter("max_dis", max_dis);
    nh->declare_parameter("min_dis", min_dis);
    nh->get_parameter("min_dis", min_dis);
    nh->declare_parameter("map_resolution", map_resolution);
    nh->get_parameter("map_resolution", map_resolution);

    
    laserMapping.init(map_resolution);
    rclcpp::Subscriber subLaserCloud = nh.create_subscription<sensor_msgs::msg::PointCloud2>("/ouster/points", 100,std::bind(& velodyneHandler, std::placeholders::_1));
    rclcpp::Subscriber subOdometry = nh.create_subscription<nav_msgs::msg::Odometry>("/odom_dq", 100,std::bind(& odomCallback, std::placeholders::_1));

    map_pub = nh.create_publisher<sensor_msgs::msg::PointCloud2>("/map", 100);
    std::thread laser_mapping_process{laser_mapping};

    rclcpp::spin(nh);

    return 0;
}
