#include <nanoflann.hpp>
#include <deque>
#include <Eigen/Dense>

#include <Eigen/Geometry>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>
#include <iostream>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

// PCL
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

// Internal library
#include "STDesc.h"
#include "KDTree_STD.h"


// ROS
#include <ros/ros.h>
#include "ros/init.h"
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>


// Time
#include <chrono>
#include <random>

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloud;

std::mutex laser_mtx;
std::mutex odom_mtx;

std::string pcTopic   = "/velodyne_points";

bool init_std = true;

std::queue<sensor_msgs::PointCloud2::ConstPtr> laser_buffer;
std::queue<nav_msgs::Odometry::ConstPtr> odom_buffer;

sensor_msgs::PointCloud2::ConstPtr msg_point;
int current_frame_id_=0;

void laserCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    std::unique_lock<std::mutex> lock(laser_mtx);
    msg_point = msg;
    laser_buffer.push(msg);
}

void OdomHandler(const nav_msgs::Odometry::ConstPtr &msg) {
    std::unique_lock<std::mutex> lock(odom_mtx);
    odom_buffer.push(msg);
}

//////////////////////////////// Data synchronization:
bool syncPackages(PointCloud::Ptr &cloud, Eigen::Affine3d &pose) {
    if (laser_buffer.empty() || odom_buffer.empty())
        return false;

    auto laser_msg = laser_buffer.front();
    double laser_timestamp = laser_msg->header.stamp.toSec();

    auto odom_msg = odom_buffer.front();
    double odom_timestamp = odom_msg->header.stamp.toSec();

    // check if timestamps are matched
    if (abs(odom_timestamp - laser_timestamp) < 1e-3) {
        pcl::fromROSMsg(*laser_msg, *cloud);

        Eigen::Quaterniond r(
            odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
            odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
        Eigen::Vector3d t(odom_msg->pose.pose.position.x,
                          odom_msg->pose.pose.position.y,
                          odom_msg->pose.pose.position.z);

        pose = Eigen::Affine3d::Identity();
        pose.translate(t);
        pose.rotate(r);

        std::unique_lock<std::mutex> l_lock(laser_mtx);
        std::unique_lock<std::mutex> o_lock(odom_mtx);

        laser_buffer.pop();
        odom_buffer.pop();

    } else if (odom_timestamp < laser_timestamp) {
        ROS_WARN("Current odometry is earlier than laser scan, discard one "
                 "odometry data.");
        std::unique_lock<std::mutex> o_lock(odom_mtx);
        odom_buffer.pop();
        return false;
    } else {
        ROS_WARN(
            "Current laser scan is earlier than odometry, discard one laser scan.");
        std::unique_lock<std::mutex> l_lock(laser_mtx);
        laser_buffer.pop();
        return false;
    }

    return true;
}
////////////////////////////////////////////////////////////////

//////////////////// Read Input Point Cloud //////////////////////
bool readPC(PointCloud::Ptr &cloud) {
    if (laser_buffer.empty())
        return false;

    auto laser_msg = laser_buffer.front();
    double laser_timestamp = laser_msg->header.stamp.toSec();
    pcl::fromROSMsg(*laser_msg, *cloud);
    std::unique_lock<std::mutex> l_lock(laser_mtx);
    laser_buffer.pop();
    return true;
}
//////////////////////////////////////////////////////////////////

void convertToMarkers(const std::vector<STDesc>& stds, visualization_msgs::MarkerArray& marker_array, const Eigen::Vector3f& color, float alpha = 1.0, float scale = 0.03) {
    int id = 0;

    for (const auto& std : stds) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "std_descriptors";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = scale;
        marker.color.r = color(0);  
        marker.color.g = color(1);  
        marker.color.b = color(2);  
        marker.color.a = alpha;     

        geometry_msgs::Point p1, p2, p3;
        p1.x = std.vertex_A_(0);
        p1.y = std.vertex_A_(1);
        p1.z = std.vertex_A_(2);
        p2.x = std.vertex_B_(0);
        p2.y = std.vertex_B_(1);
        p2.z = std.vertex_B_(2);
        p3.x = std.vertex_C_(0);
        p3.y = std.vertex_C_(1);
        p3.z = std.vertex_C_(2);

        marker.points.push_back(p1);
        marker.points.push_back(p2);
        marker.points.push_back(p2);
        marker.points.push_back(p3);
        marker.points.push_back(p3);
        marker.points.push_back(p1);

        marker_array.markers.push_back(marker);
    }
}

void MAPconvertToMarkers(const Eigen::MatrixXf& data, visualization_msgs::MarkerArray& marker_array, const Eigen::Vector3f& color, float alpha = 1.0, float scale = 0.03) {
    int id = 0;

    for (int i = 0; i < data.rows(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "std_descriptors";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = scale;
        marker.color.r = color(0);  
        marker.color.g = color(1);  
        marker.color.b = color(2);  
        marker.color.a = alpha;     

        geometry_msgs::Point p1, p2, p3;
        p1.x = data(i,9);
        p1.y = data(i,10);
        p1.z = data(i,11);
        p2.x = data(i,12);
        p2.y = data(i,13);
        p2.z = data(i,14);
        p3.x = data(i,15);
        p3.y = data(i,16);
        p3.z = data(i,17);

        marker.points.push_back(p1);
        marker.points.push_back(p2);
        marker.points.push_back(p2);
        marker.points.push_back(p3);
        marker.points.push_back(p3);
        marker.points.push_back(p1);

        marker_array.markers.push_back(marker);
    }
}

void convertToPointCloud(const std::vector<STDesc>& stds, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    for (const auto& std : stds) {
        pcl::PointXYZ p1, p2, p3;
        p1.x = std.vertex_A_(0);
        p1.y = std.vertex_A_(1);
        p1.z = std.vertex_A_(2);
        p2.x = std.vertex_B_(0);
        p2.y = std.vertex_B_(1);
        p2.z = std.vertex_B_(2);
        p3.x = std.vertex_C_(0);
        p3.y = std.vertex_C_(1);
        p3.z = std.vertex_C_(2);

        cloud->points.push_back(p1);
        cloud->points.push_back(p2);
        cloud->points.push_back(p3);
    }
}

void MAPconvertToPointCloud(const Eigen::MatrixXf& data, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    for (int i = 0; i < data.rows(); ++i) {
        pcl::PointXYZ p1, p2, p3;

        // the vertices are in positions 9-17
        p1.x = data(i, 9);
        p1.y = data(i, 10);
        p1.z = data(i, 11);
        
        p2.x = data(i, 12);
        p2.y = data(i, 13);
        p2.z = data(i, 14);

        p3.x = data(i, 15);
        p3.y = data(i, 16);
        p3.z = data(i, 17);

        cloud->points.push_back(p1);
        cloud->points.push_back(p2);
        cloud->points.push_back(p3);
    }
}

void printSTDesc(const STDesc& desc) {
    std::cout << "Side Lengths: " << desc.side_length_.transpose() << std::endl;
    std::cout << "Angles: " << desc.angle_.transpose() << std::endl;
    std::cout << "Center: " << desc.center_.transpose() << std::endl;
    std::cout << "Vertex A: " << desc.vertex_A_.transpose() << std::endl;
    std::cout << "Vertex B: " << desc.vertex_B_.transpose() << std::endl;
    std::cout << "Vertex C: " << desc.vertex_C_.transpose() << std::endl;
    //std::cout << "Norms : " << desc.norms.transpose() << std::endl;
    std::cout << "Frame ID: " << desc.frame_id_ << std::endl;
}

template <typename T>
void printVector(const std::vector<T>& vec) {
    std::cout << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        std::cout << vec[i];
        if (i != vec.size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;
}

void addDescriptorToMatrix(Eigen::MatrixXf& mat, const STDesc& desc, int row) {
    // the matrix has 36 elements
    Eigen::Vector3f side_length = desc.side_length_.cast<float>();
    Eigen::Vector3f angle = desc.angle_.cast<float>();
    Eigen::Vector3f center = desc.center_.cast<float>();
    Eigen::Vector3f vertex_A = desc.vertex_A_.cast<float>();
    Eigen::Vector3f vertex_B = desc.vertex_B_.cast<float>();
    Eigen::Vector3f vertex_C = desc.vertex_C_.cast<float>();
    Eigen::Vector3f normal1 = desc.normal1_.cast<float>();
    Eigen::Vector3f normal2 = desc.normal2_.cast<float>();
    Eigen::Vector3f normal3 = desc.normal3_.cast<float>();
    Eigen::Matrix3d axes = desc.calculateReferenceFrame();
    Eigen::Matrix<float, 9, 1> axes_vec;
    axes_vec << axes(0),axes(1),axes(2),axes(3),axes(4),axes(5),axes(6),axes(7),axes(8);
    mat.block<1, 3>(row, 0) = side_length.transpose();
    mat.block<1, 3>(row, 3) = angle.transpose();
    mat.block<1, 3>(row, 6) = center.transpose();
    mat.block<1, 3>(row, 9) = vertex_A.transpose();
    mat.block<1, 3>(row, 12) = vertex_B.transpose();
    mat.block<1, 3>(row, 15) = vertex_C.transpose();
    mat.block<1, 3>(row, 18) = normal1.transpose();
    mat.block<1, 3>(row, 21) = normal2.transpose();
    mat.block<1, 3>(row, 24) = normal3.transpose();
    mat.block<1, 9>(row, 27) = axes_vec.transpose();
}

void updateMatrixAndKDTree(Eigen::MatrixXf& mat, std::unique_ptr<nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXf>>& index, const std::deque<STDesc>& std_local_map) {
    int num_desc = std_local_map.size();
    mat.resize(num_desc, 36);

    // Fill the matrix with the current descriptors
    for (size_t i = 0; i < std_local_map.size(); ++i) {
        addDescriptorToMatrix(mat, std_local_map[i], i);
    }
    // Recreate the KD-Tree with the updated matrix
    index = std::make_unique<nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXf>>(36, std::cref(mat), 10 /* max leaf */);
    index->index_->buildIndex();
}

void publishLocalMap(const std::deque<STDesc>& std_local_map, visualization_msgs::MarkerArray& marker_array, const Eigen::Vector3f& color, float alpha = 1.0) {
    std::vector<STDesc> temp_vector;
    temp_vector.reserve(std_local_map.size());
    for (const auto& desc : std_local_map) {
        temp_vector.push_back(desc);
    }
    // std::cout << "publishLocalMap**********: " << std_local_map.size() << std::endl;
    // std::cout << "temp_vector " << temp_vector.size() << std::endl;
    convertToMarkers(temp_vector, marker_array, color, alpha,0.03);
}

// Function to generate random colors
std::tuple<float, float, float> getRandomColor() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<> dis(0.0, 1.0);
    return std::make_tuple(dis(gen), dis(gen), dis(gen));
}

void generateArrow(const STDesc& desc1, const STDesc& desc2, visualization_msgs::MarkerArray& marker_array, int& id, const std_msgs::Header& header) {
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "map";
    arrow.ns = "std_matches";
    arrow.id = id++;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.scale.x = 0.05;  // Grosor del cuerpo de la flecha
    arrow.scale.y = 0.2;  // Grosor de la cabeza de la flecha
    arrow.scale.z = 0.4;   // Longitud de la cabeza de la flecha
    
    // random color for the arrow
    auto [r, g, b] = getRandomColor();
    arrow.color.r = r;
    arrow.color.g = g;
    arrow.color.b = b;
    arrow.color.a = 1.0;

    // Starting point (center of descriptor 1)
    geometry_msgs::Point start;
    start.x = desc1.center_(0);
    start.y = desc1.center_(1);
    start.z = desc1.center_(2);

    // End point (center of descriptor 2)
    geometry_msgs::Point end;
    end.x = desc2.center_(0);
    end.y = desc2.center_(1);
    end.z = desc2.center_(2);

    arrow.points.push_back(start);
    arrow.points.push_back(end);

    marker_array.markers.push_back(arrow);
}

// Function to calculate the Euclidean distance between two vertices
float calcularDistancia(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2) {
    return (v1 - v2).norm();
}

void extractVerticesToMatrix(const std::deque<STDesc>& std_local_map, Eigen::MatrixXf& all_vertices) {
    const int num_desc = std_local_map.size();
    all_vertices.resize(3 * num_desc, 3); // 3 rows per descriptor, each with 3 coordinates

    for (size_t i = 0; i < num_desc; ++i) {
        all_vertices.row(3 * i) = std_local_map[i].vertex_A_.transpose().cast<float>();   // vertex_A
        all_vertices.row(3 * i + 1) = std_local_map[i].vertex_B_.transpose().cast<float>(); // vertex_B
        all_vertices.row(3 * i + 2) = std_local_map[i].vertex_C_.transpose().cast<float>(); // vertex_C
    }
}

void build_std_filter( const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &corner_points, std::vector<STDesc> &stds_vec, const ConfigSetting &config_setting_) {

    double scale = 1.0 / config_setting_.std_side_resolution_;   
    unsigned int current_frame_id_ = 0; // It is assumed that this value is incremented on each call to the function

    for (size_t i = 0; i < corner_points->size(); i += 3) {
        if (i + 2 >= corner_points->size()) break; // Make sure there are at least three points to form a triangle.

        pcl::PointXYZINormal p1 = corner_points->points[i];
        pcl::PointXYZINormal p2 = corner_points->points[i + 1];
        pcl::PointXYZINormal p3 = corner_points->points[i + 2];

        Eigen::Vector3d A, B, C;
        A << p1.x, p1.y, p1.z;
        B << p2.x, p2.y, p2.z;
        C << p3.x, p3.y, p3.z;

        double a = (A - B).norm();
        double b = (A - C).norm();
        double c = (B - C).norm();

        Eigen::Vector3d normal_1(p1.normal_x, p1.normal_y, p1.normal_z);
        Eigen::Vector3d normal_2(p2.normal_x, p2.normal_y, p2.normal_z);
        Eigen::Vector3d normal_3(p3.normal_x, p3.normal_y, p3.normal_z);

        STDesc single_descriptor;
        current_frame_id_++;
        single_descriptor.vertex_A_ = A;
        single_descriptor.vertex_B_ = B;
        single_descriptor.vertex_C_ = C;
        single_descriptor.center_ = (A + B + C) / 3;
        single_descriptor.side_length_ << scale * a, scale * b, scale * c;
        single_descriptor.angle_[0] = fabs(5 * normal_1.dot(normal_2));
        single_descriptor.angle_[1] = fabs(5 * normal_1.dot(normal_3));
        single_descriptor.angle_[2] = fabs(5 * normal_3.dot(normal_2));
        single_descriptor.normal1_ = normal_1;
        single_descriptor.normal2_ = normal_2;
        single_descriptor.normal3_ = normal_3;
        single_descriptor.frame_id_ = current_frame_id_;

        stds_vec.push_back(single_descriptor);

    }
}
   
// Function to check and group vertices within a radius
void cluster_vertx(const Eigen::MatrixXf &vertices, std::vector<int> &labels, const float EPSILON) {
    std::cout<<"epsilon: "<<EPSILON<<std::endl;
    const int num_points = vertices.rows();
    labels.assign(num_points, -1); // Initialize tags to -1 (not visited)
    int current_label = 0;

    for (int i = 0; i < num_points; ++i) {
        if (labels[i] != -1) continue; // If already labeled, continue to the next one

        // Label the current point with a new cluster label
        labels[i] = current_label;

        // Scan all points from the next to the current one to find neighbors
        for (int j = i + 1; j < num_points; ++j) {
            if (calcularDistancia(vertices.row(i).transpose(), vertices.row(j).transpose()) <= EPSILON) {
                labels[j] = current_label; // Label the neighboring point with the same cluster label
            }
        }
        current_label++;
    }


}

// Function for averaging vertices grouped by labels
Eigen::MatrixXf meanVertex(const Eigen::MatrixXf &vertices, const std::vector<int> &labels) {
    std::map<int, Eigen::Vector3f> sum_vertices;
    std::map<int, int> count_vertices;

    // Sum the vertices per label
    for (int i = 0; i < vertices.rows(); ++i) {
        int label = labels[i];
        if (label >= 0) {
            if (sum_vertices.find(label) == sum_vertices.end()) {
                sum_vertices[label] = Eigen::Vector3f::Zero();
                count_vertices[label] = 0;
            }
            sum_vertices[label] += vertices.row(i).transpose();
            count_vertices[label] += 1;
        }
    }

    // Create a new vertex matrix with the averages
    Eigen::MatrixXf new_vertices(vertices.rows(), vertices.cols());

    for (int i = 0; i < vertices.rows(); ++i) {
        int label = labels[i];
        if (label >= 0 && count_vertices[label] > 0) {
            new_vertices.row(i) = (sum_vertices[label] / count_vertices[label]).transpose();
        } else {
            new_vertices.row(i) = vertices.row(i);
        }
    }

    return new_vertices;
}

void updateMatrixAndKDTreeWithFiltering(Eigen::MatrixXf& mat, std::unique_ptr<nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXf>>& index, std::deque<STDesc>& std_local_map,  ConfigSetting config_setting) {
    std::cout << "Size of std_local_map: " << std_local_map.size() << std::endl;
    std::cout << "Size of a mat: " << mat.size()/36 << std::endl;

    int num_desc = std_local_map.size();
    mat.resize(num_desc, 36);

    for (size_t i = 0; i < std_local_map.size(); ++i) {
        addDescriptorToMatrix(mat, std_local_map[i], i);
    }
    std::cout << "Size of std_local_map and a mat: " << mat.size()/36 << std::endl;

 

    /////////////////// Vertex filtering
    Eigen::MatrixXf all_vertices;
    extractVerticesToMatrix(std_local_map, all_vertices);

    // Apply cluster to all vertices
     std::vector<int> vertex_labels;
    cluster_vertx(all_vertices, vertex_labels, config_setting.epsilon_);

    // std::cout << "Labels after the clustering:" << std::endl;
    // for (int label : vertex_labels) {
    //     std::cout << label << " ";
    // }
    // std::cout << std::endl;

    Eigen::MatrixXf new_vertices = meanVertex(all_vertices, vertex_labels);

    // Rebuild std_local_map with merged vertices
    std::deque<STDesc> filtered_std_local_map;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr corner_points(new pcl::PointCloud<pcl::PointXYZINormal>);

    for (size_t i = 0; i < std_local_map.size(); ++i) {
        pcl::PointXYZINormal p1, p2, p3;

        Eigen::Vector3d vertex_A = new_vertices.row(3 * i).cast<double>();
        Eigen::Vector3d vertex_B = new_vertices.row(3 * i + 1).cast<double>();
        Eigen::Vector3d vertex_C = new_vertices.row(3 * i + 2).cast<double>();

        p1.x = vertex_A[0]; p1.y = vertex_A[1]; p1.z = vertex_A[2];
        p1.normal_x = std_local_map[i].normal1_[0]; p1.normal_y = std_local_map[i].normal1_[1]; p1.normal_z = std_local_map[i].normal1_[2];
        p1.intensity = std_local_map[i].vertex_attached_[0];

        p2.x = vertex_B[0]; p2.y = vertex_B[1]; p2.z = vertex_B[2];
        p2.normal_x = std_local_map[i].normal2_[0]; p2.normal_y = std_local_map[i].normal2_[1]; p2.normal_z = std_local_map[i].normal2_[2];
        p2.intensity = std_local_map[i].vertex_attached_[1];

        p3.x = vertex_C[0]; p3.y = vertex_C[1]; p3.z = vertex_C[2];
        p3.normal_x = std_local_map[i].normal3_[0]; p3.normal_y = std_local_map[i].normal3_[1]; p3.normal_z = std_local_map[i].normal3_[2];
        p3.intensity = std_local_map[i].vertex_attached_[2];

        corner_points->points.push_back(p1);
        corner_points->points.push_back(p2);
        corner_points->points.push_back(p3);
    }

    //Use corner_points to construct stds_vec
    std::vector<STDesc> stds_vec;
    build_std_filter(corner_points, stds_vec, config_setting);
    filtered_std_local_map.assign(stds_vec.begin(), stds_vec.end());
    // Update std_local_map with filtered and merged descriptors
    std_local_map = std::move(filtered_std_local_map);

    // Update matrix and KD-Tree with filtered and merged descriptors
    num_desc = std_local_map.size();
    mat.resize(num_desc, 36);

    for (size_t i = 0; i < std_local_map.size(); ++i) {
        addDescriptorToMatrix(mat, std_local_map[i], i); // here the filtered descriptions are already added to the STD_local map

    }

    // Recreate the KD-Tree with the updated matrix
    index = std::make_unique<nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXf>>(36, std::cref(mat), 10 /* max leaf */);
    index->index_->buildIndex();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "STD_descriptor");
    ros::NodeHandle nh;

    ConfigSetting config_setting;
    read_parameters(nh, config_setting);

    nh.getParam("/pcTopic",pcTopic);        



    ros::Publisher pubkeycurr = nh.advertise<visualization_msgs::MarkerArray>("std_curr", 10);
    ros::Publisher pubkeyprev = nh.advertise<visualization_msgs::MarkerArray>("std_prev", 10);    
    ros::Publisher pubkeymap = nh.advertise<visualization_msgs::MarkerArray>("std_map", 10); 
    ros::Publisher pubkeymap_filter = nh.advertise<visualization_msgs::MarkerArray>("std_map_filter", 10);  
    
    ros::Publisher pub_curr_points = nh.advertise<sensor_msgs::PointCloud2>("std_curr_points", 10);
    ros::Publisher pub_prev_points = nh.advertise<sensor_msgs::PointCloud2>("std_prev_points", 10);
    ros::Publisher pub_map_points = nh.advertise<sensor_msgs::PointCloud2>("std_map_points", 10);
    
    ros::Publisher pubSTD = nh.advertise<visualization_msgs::MarkerArray>("pair_std", 10);
    // ros::Publisher marker_pub_prev = nh.advertise<visualization_msgs::MarkerArray>("Axes_prev_STD", 10);
    // ros::Publisher marker_pub_curr = nh.advertise<visualization_msgs::MarkerArray>("Axes_curr_STD", 10);
    ros::Publisher pose_pub_prev = nh.advertise<geometry_msgs::PoseArray>("std_prev_poses", 10);
    ros::Publisher pose_pub_curr = nh.advertise<geometry_msgs::PoseArray>("std_curr_poses", 10);

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pcTopic, 100, laserCloudHandler);
    ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry>("/odom_dq", 100, OdomHandler);
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("output_cloud_std_ex", 1);
    ros::Publisher cloud_pub_prev = nh.advertise<sensor_msgs::PointCloud2>("output_cloud_prev", 1);


    STDescManager *std_manager = new STDescManager(config_setting);
    std::vector<STDesc> stds_curr;
    std::vector<STDesc> stds_prev;
    
    std::deque<STDesc> std_local_map;
    std::deque<int> counts_per_iteration;

    Eigen::MatrixXf mat(0, 36);
    std::unique_ptr<nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXf>> index;

    PointCloud::Ptr current_cloud_world(new PointCloud);
    PointCloud::Ptr current_cloud_diff(new PointCloud);    
    PointCloud::Ptr current_cloud(new PointCloud);
    Eigen::Affine3d pose;
    Eigen::Affine3d pose_prev = Eigen::Affine3d::Identity();
    std::deque<Eigen::Affine3d> pose_vec;
    int cont_itera = 0;

    while (ros::ok()) {
        ros::spinOnce();
        std::vector<STDesc> stds_curr_pair;
        std::vector<STDesc> stds_map_pair;
        std::vector<STDesc> stds_map; 

        if (syncPackages(current_cloud, pose)) {           
            pose_vec.push_back(pose);
            auto start = std::chrono::high_resolution_clock::now();       
            down_sampling_voxel(*current_cloud, config_setting.ds_size_);              
            
            int cont_desc_pairs = 0;
            if (init_std) {
                init_std = false;
                
                std_manager->GenerateSTDescs(current_cloud, stds_curr);
                *current_cloud_world = *current_cloud;
                stds_prev = stds_curr;
                stds_map = stds_curr;
                 
                ROS_INFO("++++++++++ Iniciando Extraccion de STD ++++++++");
            } else { 
                
                pcl::transformPointCloud(*current_cloud, *current_cloud_world, pose_prev);
                std_manager->GenerateSTDescs(current_cloud_world, stds_curr);

                if (!stds_prev.empty()) {
                    visualization_msgs::MarkerArray marker_array;
                    int id = 0;
 
                    for (const auto& desc : stds_curr) {
                        std::vector<float> query;
                        Eigen::Vector3f side_length = desc.side_length_.cast<float>();
                        Eigen::Vector3f angle = desc.angle_.cast<float>();
                        Eigen::Vector3f center = desc.center_.cast<float>();
                        Eigen::Vector3f vertex_A = desc.vertex_A_.cast<float>();
                        Eigen::Vector3f vertex_B = desc.vertex_B_.cast<float>();
                        Eigen::Vector3f vertex_C = desc.vertex_C_.cast<float>();
                        Eigen::Vector3f norms1 = desc.normal1_.cast<float>();
                        Eigen::Vector3f norms2 = desc.normal2_.cast<float>();
                        Eigen::Vector3f norms3 = desc.normal3_.cast<float>();
                        Eigen::Matrix3d axes_f = desc.calculateReferenceFrame();


                        query.insert(query.end(), side_length.data(), side_length.data() + 3);
                        query.insert(query.end(), angle.data(), angle.data() + 3);
                        query.insert(query.end(), center.data(), center.data() + 3);
                        query.insert(query.end(), vertex_A.data(), vertex_A.data() + 3);
                        query.insert(query.end(), vertex_B.data(), vertex_B.data() + 3);
                        query.insert(query.end(), vertex_C.data(), vertex_C.data() + 3);
                        query.insert(query.end(), norms1.data(), norms1.data() + 3);
                        query.insert(query.end(), norms2.data(), norms2.data() + 3);
                        query.insert(query.end(), norms3.data(), norms3.data() + 3);
                        query.insert(query.end(), axes_f.data(), axes_f.data() + axes_f.size());

                        // Search for the nearest descriptor
                        const size_t num_results = 1;
                        std::vector<size_t> ret_indexes(num_results);
                        std::vector<float> out_dists_sqr(num_results);

                        nanoflann::KNNResultSet<float> resultSet(num_results);
                        resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
                        index->index_->findNeighbors(resultSet, query.data());

                        for (size_t i = 0; i < resultSet.size(); i++) {
                           
                            if (ret_indexes[i] < std_local_map.size() && out_dists_sqr[i] < config_setting.kdtree_threshold_) {
                                cont_desc_pairs++;
                                generateArrow(desc, std_local_map[ret_indexes[i]], marker_array, id, msg_point->header);

                                stds_map_pair.push_back(std_local_map[ret_indexes[i]]);
                                stds_curr_pair.push_back(desc);
                            }
                            else{
                                // elements that had no match to be added to std_map to make the map robust:
                                stds_map.push_back(desc);
                            }
                        }
                    }

                    // Publish arrows in RViz
                    pubSTD.publish(marker_array);
                    visualization_msgs::Marker delete_marker_curr;
                    delete_marker_curr.action = visualization_msgs::Marker::DELETEALL;
                    marker_array.markers.clear();
                    marker_array.markers.push_back(delete_marker_curr);
                    pubSTD.publish(marker_array);
                }
            }

            sensor_msgs::PointCloud2 output_cloud;
            pcl::toROSMsg(*current_cloud_world, output_cloud);
            output_cloud.header.frame_id = "map";  
            cloud_pub.publish(output_cloud);
            
            ///////////////// Data visualization ///////

            //// visualizacion de los keypoints current
            visualization_msgs::MarkerArray marker_array_curr;
            Eigen::Vector3f colorVector_curr(0.0f, 0.0f, 1.0f);  // azul

            convertToMarkers(stds_curr, marker_array_curr,colorVector_curr ,0.5,0.05);
            pubkeycurr.publish(marker_array_curr);
            visualization_msgs::Marker delete_marker_curr;
            delete_marker_curr.action = visualization_msgs::Marker::DELETEALL;
            marker_array_curr.markers.clear();  // Asegúrate de que el array de marcadores esté vacío
            marker_array_curr.markers.push_back(delete_marker_curr);
            pubkeycurr.publish(marker_array_curr);
            //////////////////////////////////////////

            ////// publicacion de nube de puntos en los vertices de los stds
            pcl::PointCloud<pcl::PointXYZ>::Ptr std_points(new pcl::PointCloud<pcl::PointXYZ>);
            convertToPointCloud(stds_curr, std_points);
            sensor_msgs::PointCloud2 output_curr;
            pcl::toROSMsg(*std_points, output_curr);
            output_curr.header.frame_id = "map";
            pub_curr_points.publish(output_curr);
            //////////////////////////////////////////
            
            ///////////////////// Previous std
            ////// visualizacion de los keypoints prev
            visualization_msgs::MarkerArray marker_array_prev;
            Eigen::Vector3f colorVector_prev(1.0f, 0.0f, 0.0f);  // rojo
            convertToMarkers(stds_prev, marker_array_prev,colorVector_prev ,0.25);
            pubkeyprev.publish(marker_array_prev);
            visualization_msgs::Marker delete_marker_prev;
            delete_marker_prev.action = visualization_msgs::Marker::DELETEALL;
            marker_array_prev.markers.clear();  // Asegúrate de que el array de marcadores esté vacío
            marker_array_prev.markers.push_back(delete_marker_prev);
            pubkeyprev.publish(marker_array_prev);
            //////////////////////////////////////////

            ////// publicacion de nube de puntos en los vertices de los stds
            pcl::PointCloud<pcl::PointXYZ>::Ptr std_points_prev(new pcl::PointCloud<pcl::PointXYZ>);
            convertToPointCloud(stds_prev, std_points_prev);
            sensor_msgs::PointCloud2 output_prev;
            pcl::toROSMsg(*std_points_prev, output_prev);
            output_prev.header.frame_id = "map";
            pub_prev_points.publish(output_prev);
            //////////////////////////////////////////

            /////////////// plot stds_map
            visualization_msgs::MarkerArray marker_array_map;
            Eigen::Vector3f colorVector_map(0.0f, 0.0f, 0.0f); 
            publishLocalMap(std_local_map, marker_array_map,colorVector_map ,0.5);
            pubkeymap.publish(marker_array_map);

            ////////////////////////////////////////////////////////////////////////////////////////////////

             std::cout << "Matching pairs: " << cont_desc_pairs << std::endl;
            // Add new stds_curr descriptors to std_local_map
            if(!init_std){
                pcl::transformPointCloud(*current_cloud, *current_cloud_world, pose);
                std_manager->GenerateSTDescs(current_cloud_world, stds_curr);
            }
            std_local_map.insert(std_local_map.end(), stds_curr.begin(), stds_curr.end());
            ////////////////////////////Removal of elements per window///////////////
            counts_per_iteration.push_back(stds_curr.size());
            while (counts_per_iteration.size() > config_setting.max_window_size_) {
                int count_to_remove = counts_per_iteration.front();
                counts_per_iteration.pop_front();
                for (int i = 0; i < count_to_remove; ++i) {
                    std_local_map.pop_front();                    
                }
            }
            ////////////////////////////////////////////////////////////////////////////////////

            // Update the matrix with the filtering by vertices

            updateMatrixAndKDTreeWithFiltering(mat, index, std_local_map, config_setting);
            //updateMatrixAndKDTree(mat, index, std_local_map);

            ////// publication of point cloud on the vertices of the filtered MAP stds
            pcl::PointCloud<pcl::PointXYZ>::Ptr std_map_pcl(new pcl::PointCloud<pcl::PointXYZ>);
            MAPconvertToPointCloud(mat, std_map_pcl);
            sensor_msgs::PointCloud2 output_map_point;
            pcl::toROSMsg(*std_map_pcl, output_map_point);
            output_map_point.header.frame_id = "map";
            pub_map_points.publish(output_map_point);

            ////// display of the filtered map triangles
            visualization_msgs::MarkerArray marker_map_filter;
            Eigen::Vector3f colorVector_map_fil(1.0f, 0.0f, 1.0f);  
            MAPconvertToMarkers(mat, marker_map_filter,colorVector_map_fil ,1.0,0.05);
            pubkeymap_filter.publish(marker_map_filter);
            visualization_msgs::Marker delete_map_filter;
            delete_map_filter.action = visualization_msgs::Marker::DELETEALL;
            marker_map_filter.markers.clear();  
            marker_map_filter.markers.push_back(delete_map_filter);
            pubkeymap_filter.publish(marker_map_filter);
            //////////////////////////////////////////

            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end - start;

            ROS_INFO("Extracted %lu ST descriptors in %f seconds", stds_curr.size(), elapsed.count());
            
            std_manager->publishPoses(pose_pub_prev, stds_map_pair, msg_point->header,"map");
            std_manager->publishPoses(pose_pub_curr, stds_curr_pair, msg_point->header,"map");

            // update stds_prev
            stds_prev = stds_curr;
            pose_prev = pose;
            std::cout<<"Iteration: "<<cont_itera++<<std::endl;            
        }
    }

    return 0;
}