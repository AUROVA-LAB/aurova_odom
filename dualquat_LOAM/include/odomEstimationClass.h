// Author of Lilo: Edison Velasco 
// Email evs25@alu.ua.es

#ifndef _ODOM_ESTIMATION_CLASS_H_
#define _ODOM_ESTIMATION_CLASS_H_

//std lib
#include <string>
#include <math.h>
#include <vector>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


// ROS msgs
#include <geometry_msgs/PoseArray.h>

//ceres
#include <ceres/ceres.h>
#include <ceres/rotation.h>

//eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

//LOCAL LIB
#include <ros/ros.h>

// STD LIB
#include "STDesc.h"
#include "KDTree_STD.h"

class OdomEstimationClass 
{

    public:
    	OdomEstimationClass();
    	
        void init(double edge_resolution,double surf_resolution);
		void initMapWithPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr& surf_in);
        void updatePointsToMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr& surf_in,
		                       const std::vector<STDesc>& stdC_pair, const std::vector<STDesc>& stdM_pair,  bool clear_map ,double cropBox_len, double cont_opti);
		void getMap(pcl::PointCloud<pcl::PointXYZ>::Ptr& laserCloudMap);

		Eigen::Isometry3d odom;
		pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudCornerMap;
		pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudSurfMap;
		geometry_msgs::PoseArrayConstPtr std_posesMap;

		pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud; 
		pcl::PointCloud<pcl::PointXYZ>::Ptr surf_cloud; 

	private:
	
		double parameters[8] = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	    Eigen::Map<Eigen::Matrix<double, 8, 1>> dual_quat = Eigen::Map<Eigen::Matrix<double, 8, 1>>(parameters);

		Eigen::Isometry3d last_odom;

		//kd-tree edge
		pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeEdgeMap;
		pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeLocalMap;

		//kd-tree surf
		pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeSurfMap;
		
		//points downsampling before add to map
		pcl::VoxelGrid<pcl::PointXYZ> downSizeFilterEdge;
		pcl::VoxelGrid<pcl::PointXYZ> downSizeFilterSurf;

		//local map
        pcl::CropBox<pcl::PointXYZ> cropBoxFilter;

		//optimization count 
		int optimization_count;

        void addPointsToMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& downsampledEdgeCloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& downsampledSurfCloud, bool clear_map, double cropBox_len);
        void pointAssociateToMap(pcl::PointXYZ const *const pi, pcl::PointXYZ *const po);
        void pointLocalToMap(pcl::PointXYZ const *const pi, pcl::PointXYZ *const po);
	    void downSamplingToMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& edge_pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& edge_pc_out, const pcl::PointCloud<pcl::PointXYZ>::Ptr& surf_pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& surf_pc_out);
        void occlude_pcd(pcl::PointCloud<pcl::PointXYZ>::Ptr & cld_ptr,int dim, double threshA, double threshB);
		void addEdgeDQCostFactor(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function, ceres::Manifold* dq_manifold, double cropBox_len);
		void addSurfDQCostFactor(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function, ceres::Manifold* dq_manifold, double cropBox_len);
		void addSTDCostFactor(std::vector<STDesc> stdC_pair, std::vector<STDesc> stdM_pair, ceres::Problem& problem, ceres::LossFunction *loss_function, ceres::Manifold* dq_manifold);

};

#endif // _ODOM_ESTIMATION_CLASS_H_