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
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "lidar.h"
#include "odomEstimationClass.h"

OdomEstimationClass odomEstimation;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;

std::string childframeID = "os_sesnor";
std::string edge_pcl = "/pcl_edge";
std::string surf_pcl = "/pcl_surf";

lidar::Lidar lidar_param;

ros::Publisher pubLaserOdometry;
ros::Publisher pubOdometryDiff;

ros::Publisher time_average;


void velodyneSurfHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudSurfBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}
void velodyneEdgeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
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
            ros::Time pointcloud_time = (pointCloudSurfBuf.front())->header.stamp;
            pointCloudEdgeBuf.pop();
            pointCloudSurfBuf.pop();
            mutex_lock.unlock();

            if(is_odom_inited == false){
                odomEstimation.initMapWithPoints(pointcloud_edge_in, pointcloud_surf_in);
                is_odom_inited = true;
                ROS_INFO("odom inited");
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
                ROS_INFO("average odom estimation time %f mS", time_delay);
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

            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(t_current.x(), t_current.y(), t_current.z()) );
            tf::Quaternion q(q_current.x(),q_current.y(),q_current.z(),q_current.w());
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "os_sensor"));

            //// TODO: FROM URDF!!!!!!!!
            transform.setOrigin( tf::Vector3(-0.55, 0.0, -0.645) );
            tf::Quaternion q2(0.0, 0.0, 0.0, 1.0);
            transform.setRotation(q2);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "os_sensor", "base_link"));

          
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
            nav_msgs::Odometry laserOdometry;
            laserOdometry.header.frame_id = "odom";
            laserOdometry.child_frame_id = childframeID;
            laserOdometry.header.stamp = pointcloud_time;
            laserOdometry.pose.pose.orientation.x = q_current.x();
            laserOdometry.pose.pose.orientation.y = q_current.y();
            laserOdometry.pose.pose.orientation.z = q_current.z();
            laserOdometry.pose.pose.orientation.w = q_current.w();
            laserOdometry.pose.pose.position.x = t_current.x();
            laserOdometry.pose.pose.position.y = t_current.y();
            laserOdometry.pose.pose.position.z = t_current.z();

            nav_msgs::Odometry odomDiff;
            odomDiff.header.frame_id = "odom";
            odomDiff.child_frame_id = childframeID;
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

            pubLaserOdometry.publish(laserOdometry);
            pubOdometryDiff.publish(odomDiff);

            //publish time

            std_msgs::Float64 time_msg;
            time_msg.data = time_delay*1000.0;
            time_average.publish(time_msg);

         }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

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

    nh.getParam("/scan_period", scan_period); 
    nh.getParam("/vertical_angle", vertical_angle); 
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);
    nh.getParam("/edge_resolution", edge_resolution);
    nh.getParam("/surf_resolution", surf_resolution);
    nh.getParam("/clear_map", clear_map);

    nh.getParam("/cropBox_len", cropBox_len);

    nh.getParam("/childframeID",childframeID);
    nh.getParam("/pcl_edge",edge_pcl);
    nh.getParam("/pcl_surf",surf_pcl);

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setValidationAngle(validation_angle);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    odomEstimation.init(lidar_param, edge_resolution, surf_resolution);
    
    ros::Subscriber subEdgeLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(edge_pcl, 100, velodyneEdgeHandler);
    ros::Subscriber subSurfLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(surf_pcl, 100, velodyneSurfHandler);

    pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    pubOdometryDiff = nh.advertise<nav_msgs::Odometry>("/odom_diff", 100);
    time_average = nh.advertise<std_msgs::Float64>("/time_average", 100);


    std::thread odom_estimation_process{odom_estimation};

    ros::spin();

    return 0;
}

