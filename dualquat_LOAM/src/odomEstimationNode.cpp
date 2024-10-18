// Author of dualquat_loam: Edison Velasco
// Email edison.velasco@ua.es

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
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_msgs/TFMessage.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "odomEstimationClass.h"
#include "STDFunctions.h"

// NanoFlann library for kdtree
#include <nanoflann.hpp>


typedef pcl::PointXYZI PointType;  // only for std
typedef pcl::PointCloud<PointType> pcSTD;
ConfigSetting config_setting; // configs for STDescritor
STDescManager *std_manager;

OdomEstimationClass odomEstimation;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
std::queue<geometry_msgs::PoseArrayConstPtr> stdPoseBuf;

std::string childframeID = "os_sesnor";
std::string edge_pcl = "/pcl_edge";
std::string surf_pcl = "/pcl_surf";
std::string stdescri = "/std_curr_poses";
std::string stdMap   = "/std_map_poses";
std::string pcTopic   = "/velodyne_points";
//std::string path_odom =  "/home/ws/src/resultados_dualquat_loam/00.txt";

ros::Publisher pubLaserOdometry;
ros::Publisher pubOdometryDiff;

/// STD publishers
ros::Publisher pubSTD ;
ros::Publisher cloud_pub;

// ros::Publisher std_pub_Map;
// ros::Publisher std_pub_Cur;


ros::Publisher time_average;
bool save_data = true;


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
  
// read the pointcloud to extract STD descriptor
std::queue<sensor_msgs::PointCloud2::ConstPtr> laser_buffer;
sensor_msgs::PointCloud2::ConstPtr msg_point;
std::mutex laser_mtx;

void laserCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    std::unique_lock<std::mutex> lock(laser_mtx);
    msg_point = msg;
    laser_buffer.push(msg);
}

bool readPC(pcSTD::Ptr &cloud) {
    if (laser_buffer.empty())
        return false;

    auto laser_msg = laser_buffer.front();
    //double laser_timestamp = laser_msg->header.stamp.toSec();
    pcl::fromROSMsg(*laser_msg, *cloud);
    std::unique_lock<std::mutex> l_lock(laser_mtx);
    laser_buffer.pop();
    return true;
}

Eigen::Matrix4d imu_to_cam = Eigen::Matrix4d::Identity();
Eigen::Matrix4d imu_to_velo = Eigen::Matrix4d::Identity();
Eigen::Matrix4d velo_to_cam = Eigen::Matrix4d::Identity();

int cont_map = 0;

void STD_matching(std::vector<STDesc>& stds_curr_body, std::vector<STDesc>& stds_curr_world, std::deque<STDesc>&  std_local_map,
                  std::vector<STDesc>& stdC_pair, std::vector<STDesc>& stdM_pair,
                  std::unique_ptr<nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXf>>& index, ros::Publisher pubSTD){

    int cont_desc_pairs= 0;
    //int id = 0;
    visualization_msgs::MarkerArray marker_array;

    for (size_t i = 0; i < stds_curr_world.size(); ++i) {

        const auto& descC_W = stds_curr_world[i];
        const auto& descC_B = stds_curr_body[i];

        std::vector<float> query;
        Eigen::Vector3f side_length = descC_W.side_length_.cast<float>();
        //Eigen::Vector3f angle = descC_W.angle_.cast<float>();
        Eigen::Vector3f center = descC_W.center_.cast<float>();
        Eigen::Vector3f vertex_A = descC_W.vertex_A_.cast<float>();
        Eigen::Vector3f vertex_B = descC_W.vertex_B_.cast<float>();
        Eigen::Vector3f vertex_C = descC_W.vertex_C_.cast<float>();
        Eigen::Vector3f norms1 = descC_W.normal1_.cast<float>();
        Eigen::Vector3f norms2 = descC_W.normal2_.cast<float>();
        Eigen::Vector3f norms3 = descC_W.normal3_.cast<float>();
        //Eigen::Matrix3d axes_f = descC_W.calculateReferenceFrame();


        query.insert(query.end(), side_length.data(), side_length.data() + 3);
        //query.insert(query.end(), angle.data(), angle.data() + 3);
        query.insert(query.end(), center.data(), center.data() + 3);
        query.insert(query.end(), vertex_A.data(), vertex_A.data() + 3);
        query.insert(query.end(), vertex_B.data(), vertex_B.data() + 3);
        query.insert(query.end(), vertex_C.data(), vertex_C.data() + 3);
        query.insert(query.end(), norms1.data(), norms1.data() + 3);
        query.insert(query.end(), norms2.data(), norms2.data() + 3);
        query.insert(query.end(), norms3.data(), norms3.data() + 3);
        //query.insert(query.end(), axes_f.data(), axes_f.data() + axes_f.size());

        // Find the near STD with nanoflann 
        const size_t num_results = 1;
        std::vector<size_t> ret_indexes(num_results);
        std::vector<float> out_dists_sqr(num_results);

        nanoflann::KNNResultSet<float> resultSet(num_results);
        resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
        index->index_->findNeighbors(resultSet, query.data());

        for (size_t i = 0; i < resultSet.size(); i++) {
            
            if (ret_indexes[i] < std_local_map.size() && out_dists_sqr[i] < config_setting.kdtree_threshold_) {
                cont_desc_pairs++;

                // function to draw STD arrows
                // int id = 0;
                // generateArrow(descC_W, std_local_map[ret_indexes[i]], marker_array, id, msg_point->header);

                stdM_pair.push_back(std_local_map[ret_indexes[i]]);
                stdC_pair.push_back(descC_B);
                
            }

        }        
    }
    //Number of matchs
    std::cout<<"Number of STD matchs: "<<cont_desc_pairs<<std::endl;

    // Publish the arrows in rviz
    pubSTD.publish(marker_array);
    visualization_msgs::Marker delete_marker_curr;
    delete_marker_curr.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.clear();
    marker_array.markers.push_back(delete_marker_curr);
    pubSTD.publish(marker_array);
}


bool is_odom_inited = false;
double total_time =0, cropBox_len, surf_limit, cont_opti;
int total_frame=0;
bool clear_map;
double voxel_cloud_world = 0.5;

void odom_estimation(){

    float time_delay  = 0;

    Eigen::Quaterniond q_diff;
    Eigen::Vector3d t_diff;
    Eigen::Isometry3d odom_prev = Eigen::Isometry3d::Identity();

    //////////// STD descriptor inicialization ///////////////////////////////////////////////////////
    std::vector<STDesc> stds_curr_w;    
    std::vector<STDesc> stds_curr_body;
    std::deque<STDesc> std_local_map;

    pcSTD::Ptr current_cloud(new pcSTD); // pointcloud of the original sensor. It is used to std extractor
    pcSTD::Ptr current_cloud_world(new pcSTD); // pointcloud of the original sensor. It is used to std extractor
    // pcl::PointCloud<pcl::PointXYZI>::Ptr orig_cloud(new pcl::PointCloud<pcl::PointXYZI>()); // point cloud for output visualization

    Eigen::MatrixXf mat(0, 24);   // matrix for elements in nanoflann
    std::unique_ptr<nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXf>> index;
    Eigen::Affine3d poseSTD = Eigen::Affine3d::Identity();
    std::deque<int> counts_per_iteration; // use for the cropping data of std_local_map

    //////////////////////////////////////////////////////////////////////////////////////////////////

    // path to save the trajectory/////////////////////
    //std::ofstream outputFile(path_odom);

   // size_t found = path_odom.find_last_of(".");
   // std::string orig_path = path_odom.substr(0, found);
   // orig_path += "_time.txt";
  //  std::ofstream org_outputFile(orig_path);    
    ///////////////////////////////////////////////////

    ////////////Saveing data initialization


   
    Eigen::Isometry3d odom = Eigen::Isometry3d::Identity();
    
    
    int cont=0; // cont for map
    while(1){
        
        std::chrono::time_point<std::chrono::system_clock> start, end;
        start = std::chrono::system_clock::now();

        // STD descriptors (current and map matching)
        std::vector<STDesc> stdC_pair;
        std::vector<STDesc> stdM_pair;

        if(!pointCloudEdgeBuf.empty() && !pointCloudSurfBuf.empty()){

            mutex_lock.lock();

            pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZ>());  
            pcSTD::Ptr pointcloud_ds(new pcSTD);

            pcl::fromROSMsg(*pointCloudEdgeBuf.front(), *pointcloud_edge_in);
            pcl::fromROSMsg(*pointCloudSurfBuf.front(), *pointcloud_surf_in);

            ros::Time pointcloud_time = (pointCloudSurfBuf.front())->header.stamp;
            pointCloudEdgeBuf.pop();
            pointCloudSurfBuf.pop();

            ////////////// read original pc to extract STD 
            if (readPC(current_cloud)){
                // *orig_cloud=*current_cloud;
                // down_sampling_voxel(*orig_cloud,voxel_cloud_world);
                down_sampling_voxel(*current_cloud, config_setting.ds_size_);                
            }

            mutex_lock.unlock();          

            if(is_odom_inited == false){
                odomEstimation.initMapWithPoints(pointcloud_edge_in, pointcloud_surf_in);
                is_odom_inited = true;
                ROS_INFO("odom inited");

            }else{

                //////////////////////////////// STD extractor
                poseSTD = odom_prev;
                pcl::transformPointCloud(*current_cloud, *current_cloud_world, poseSTD);
                std_manager->GenerateSTDescs(current_cloud_world, stds_curr_w);
                std_manager->GenerateSTDescs(current_cloud, stds_curr_body);
                ////////////////////////////////////////////////////////////////////////////

                ///////////////////////////////// STD matching
                STD_matching(stds_curr_body, stds_curr_w, std_local_map, stdC_pair, stdM_pair, index, pubSTD);
                //////////////////////////////////////////////////////////////////////////////////
                odomEstimation.updatePointsToMap(pointcloud_edge_in, pointcloud_surf_in, stdC_pair, stdM_pair, clear_map, cropBox_len, cont_opti);
            }


            Eigen::Quaterniond q_current(odomEstimation.odom.rotation());
            Eigen::Vector3d t_current = odomEstimation.odom.translation();

            //////////// save data/////////////////////////////////////////////////
            if (save_data){

                Eigen::Matrix4d homogeneous_matrix = Eigen::Matrix4d::Identity();
                homogeneous_matrix.block<3, 3>(0, 0) = q_current.normalized().toRotationMatrix();
                homogeneous_matrix.block<3, 1>(0, 3) = t_current;    

               // outputFile << std::scientific;

                ////// saving data for kitti
                // outputFile <<   homogeneous_matrix(1,1) <<" "
                //            <<   homogeneous_matrix(1,2) <<" "
                //            <<  -homogeneous_matrix(1,0) <<" "
                //            <<  -homogeneous_matrix(1,3) <<" "
                //            <<   homogeneous_matrix(2,1) <<" "
                //            <<   homogeneous_matrix(2,2) <<" "
                //            <<  -homogeneous_matrix(2,0) <<" "
                //            <<  -homogeneous_matrix(2,3) <<" "
                //            <<  -homogeneous_matrix(0,1) <<" "
                //            <<  -homogeneous_matrix(0,2) <<" "
                //            <<   homogeneous_matrix(0,0) <<" "
                //            <<   homogeneous_matrix(0,3) << std::endl;  


                // saving data for HeliPR.

                // uint64_t time_in_nanoseconds = pointcloud_time.sec * 1000000ULL + pointcloud_time.nsec/1000.0;
                // outputFile <<  time_in_nanoseconds <<" "
                //            <<   t_current.x() <<" "
                //            <<   t_current.y() <<" "
                //            <<   t_current.z() <<" "
                //            <<   q_current.w() <<" "
                //            <<   q_current.x() <<" "
                //            <<   q_current.y() <<" "
                //            <<   q_current.z() <<std::endl;  
                                    
                // org_outputFile << time_delay <<", ";
            }
            ///////////////////////////////////////////////////////

            //Project to 2D!!!
            // t_current.z() = 0.0;
            // double siny_cosp = 2 * (q_current.w() * q_current.z() + q_current.x() * q_current.y());
            // double cosy_cosp = 1 - 2 * (q_current.y() * q_current.y() + q_current.z() * q_current.z());
            // double yaw = std::atan2(siny_cosp, cosy_cosp);
            // Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
            // q_current = yaw_angle;
            // ///////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////

            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(t_current.x(), t_current.y(), t_current.z()) );
            tf::Quaternion q(q_current.x(),q_current.y(),q_current.z(),q_current.w());
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", childframeID));      

            // //// TODO: FROM URDF!!!!!!!!
            // transform.setOrigin( tf::Vector3(-0.55, 0.0, -0.645) );
            // tf::Quaternion q2(0.0, 0.0, 0.0, 1.0);
            // transform.setRotation(q2);
            // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), childframeID, "base_link"));        
            
            odom.linear() = q_current.toRotationMatrix();
            odom.translation() = t_current;

            Eigen::Isometry3d odomdiff = (odom_prev.inverse() * odom);
            q_diff = Eigen::Quaterniond(odomdiff.rotation());
            t_diff = odomdiff.translation();
           
            // Eigen::Isometry3d odom_curr = odom_prev * odomdiff;
            // Eigen::Quaterniond q_c = Eigen::Quaterniond(odom_curr.rotation());;
            // Eigen::Vector3d t_c = odom_curr.translation();            

            odom_prev = odom;


            ////////////////////////////// update STD map //////////////////////////////////////
            Eigen::Affine3d pose_estimated = odom;
            pcl::transformPointCloud(*current_cloud, *current_cloud_world, pose_estimated);
            std_manager->GenerateSTDescs(current_cloud_world, stds_curr_w);
            std_local_map.insert(std_local_map.end(), stds_curr_w.begin(), stds_curr_w.end());

                    ///////////////////// cropping elements per window in std_local_map ///////////////
            counts_per_iteration.push_back(stds_curr_w.size());
            while (int(counts_per_iteration.size()) > config_setting.max_window_size_) {
                int count_to_remove = counts_per_iteration.front();
                counts_per_iteration.pop_front();
                for (int i = 0; i < count_to_remove; ++i) {
                    std_local_map.pop_front();                    
                }
            }

            // update mat matrix with filtering elements. it's necesary for the kdtree matching
            //updateMatrixAndKDTree(mat, index, std_local_map);
            updateMatrixAndKDTreeWithFiltering(mat, index, std_local_map, config_setting);

            // std_manager->publishPoses(std_pub_Map, stdM_pair, msg_point->header,"map");
            // std_manager->publishPoses(std_pub_Cur, stdC_pair, msg_point->header,"map");

            end = std::chrono::system_clock::now();
            std::chrono::duration<float> elapsed_seconds = end - start;
            total_frame++;
            float time_temp = elapsed_seconds.count() * 1000.0;
            total_time+=time_temp;
            time_delay = total_time/total_frame;
            ROS_INFO("average odom estimation time %f mS", time_delay);
            time_delay = time_delay/1000.0;

            //////////////////////////////////////////////////////////////////////////////

            ///////////// original point cloud to map (only for visual representation)
            // pcl::PointCloud<pcl::PointXYZI>::Ptr orig_cloud_world(new pcl::PointCloud<pcl::PointXYZI>());
            // pcl::transformPointCloud(*orig_cloud, *orig_cloud_world, pose_estimated);

            //////////////////////////////////////////////////////////////////////////////
             
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


            //////////////// TO-DO covaraince calculation ////////////////
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
            //////////////////////////////////////////////////////////////

            pubLaserOdometry.publish(laserOdometry);
            pubOdometryDiff.publish(odomDiff);

            //publish time

            std_msgs::Float64 time_msg;
            time_msg.data = time_delay*1000.0;
            time_average.publish(time_msg);

            // publish map:
            sensor_msgs::PointCloud2 output_cloud;
            pcl::toROSMsg(*current_cloud_world, output_cloud);
            output_cloud.header.frame_id = "map";  

            
            if(cont>cont_map){
                cont = 0;
                cloud_pub.publish(output_cloud);
            }
            cont++;
         }

        std::chrono::milliseconds dura(1);
        std::this_thread::sleep_for(dura);
    }
    // if (save_data)
    //     outputFile.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    /////////////// Read confing of STD
    read_parameters(nh, config_setting);
    std_manager = new STDescManager(config_setting);
 
    double max_dis = 60.0;
    double min_dis =0;
    double edge_resolution = 0.3;
    double surf_resolution = 0.6;

    clear_map = true;
    cropBox_len = 10000;
    cont_opti =  2; // 2 for robot Blue, 1 for KITTI

    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/edge_resolution", edge_resolution);
    nh.getParam("/surf_resolution", surf_resolution); 
    nh.getParam("/clear_map", clear_map);
    nh.getParam("/save_data", save_data);    
    nh.getParam("/cropBox_len", cropBox_len);
    nh.getParam("/cont_opti", cont_opti);    
    nh.getParam("/childframeID",childframeID);
    nh.getParam("/pcl_edge",edge_pcl);
    nh.getParam("/pcl_surf",surf_pcl);
    nh.getParam("/pcTopic",pcTopic);        
   // nh.getParam("/path_odom",path_odom);    
    nh.getParam("/cont_for_map",cont_map);
    nh.getParam("/voxel_cloud_world",voxel_cloud_world);    

   
    odomEstimation.init(edge_resolution, surf_resolution);
    
    ros::Subscriber subEdgeLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(edge_pcl, 100, velodyneEdgeHandler);
    ros::Subscriber subSurfLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(surf_pcl, 100, velodyneSurfHandler);
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pcTopic, 100, laserCloudHandler);

    pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/odom_dq", 100);
    pubOdometryDiff = nh.advertise<nav_msgs::Odometry>("/odom_diff", 100);
    time_average = nh.advertise<std_msgs::Float64>("/time_average", 100);
    pubSTD = nh.advertise<visualization_msgs::MarkerArray>("pair_std", 10);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("output_cloud", 10);

    std::thread odom_estimation_process{odom_estimation};

    ros::spin();

    return 0;
}

