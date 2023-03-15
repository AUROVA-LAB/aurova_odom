// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

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

//ceres
#include <ceres/ceres.h>
#include <ceres/rotation.h>

//eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

//LOCAL LIB
#include "lidar.h"
#include "lidarOptimization.h"
#include <ros/ros.h>

class OdomEstimationClass 
{

    public:
    	OdomEstimationClass();
    	
                void init(lidar::Lidar lidar_param, double edge_resolution,double surf_resolution);
		void initMapWithPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr& surf_in);
                void updatePointsToMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr& surf_in, bool clear_map ,double cropBox_len);
		void getMap(pcl::PointCloud<pcl::PointXYZ>::Ptr& laserCloudMap);

		Eigen::Isometry3d odom;
		pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudCornerMap;
		pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudSurfMap;
	private:
		//optimization variable
		double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
		Eigen::Map<Eigen::Quaterniond> q_w_curr = Eigen::Map<Eigen::Quaterniond>(parameters);
		Eigen::Map<Eigen::Vector3d> t_w_curr = Eigen::Map<Eigen::Vector3d>(parameters + 4);

		Eigen::Isometry3d last_odom;

		//kd-tree
		pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeEdgeMap;
		pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeSurfMap;

		//points downsampling before add to map
		pcl::VoxelGrid<pcl::PointXYZ> downSizeFilterEdge;
		pcl::VoxelGrid<pcl::PointXYZ> downSizeFilterSurf;

		//local map
                pcl::CropBox<pcl::PointXYZ> cropBoxFilter;

		//optimization count 
		int optimization_count;

		//function
		void addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function);
		void addSurfCostFactor(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function);
                void addPointsToMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& downsampledEdgeCloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& downsampledSurfCloud, bool clear_map, double cropBox_len);
                void pointAssociateToMap(pcl::PointXYZ const *const pi, pcl::PointXYZ *const po);
                void downSamplingToMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& edge_pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& edge_pc_out, const pcl::PointCloud<pcl::PointXYZ>::Ptr& surf_pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& surf_pc_out);
                void occlude_pcd(pcl::PointCloud<pcl::PointXYZ>::Ptr & cld_ptr,int dim, double threshA, double threshB);
};

#endif // _ODOM_ESTIMATION_CLASS_H_

