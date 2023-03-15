// Author of FLOAM: Wang Han
// Author of Lilo: Edison Velasco
// Email evs25@alu.ua.es

#include "odomEstimationClass.h"

void OdomEstimationClass::init(lidar::Lidar lidar_param, double edge_resolution, double surf_resolution){
    //init local map
    laserCloudCornerMap = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    laserCloudSurfMap = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

    //downsampling size
    downSizeFilterEdge.setLeafSize(edge_resolution, edge_resolution, edge_resolution);
    downSizeFilterSurf.setLeafSize(surf_resolution, surf_resolution, surf_resolution);

    //kd-tree
    kdtreeEdgeMap = pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    kdtreeSurfMap = pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZ>());

    odom = Eigen::Isometry3d::Identity();
    last_odom = Eigen::Isometry3d::Identity();
    optimization_count=2;
}

void OdomEstimationClass::initMapWithPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr& surf_in){
    *laserCloudCornerMap += *edge_in;
    *laserCloudSurfMap += *surf_in;
    optimization_count=4;
    //optimization_count=20;
}


void OdomEstimationClass::updatePointsToMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr& surf_in , bool clear_map, double cropBox_len){

    if(optimization_count>2)
        optimization_count--;

    Eigen::Isometry3d odom_prediction = odom * (last_odom.inverse() * odom);
    last_odom = odom;
    odom = odom_prediction;

    q_w_curr = Eigen::Quaterniond(odom.rotation());
    t_w_curr = odom.translation();

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledEdgeCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledSurfCloud(new pcl::PointCloud<pcl::PointXYZ>());

    downSamplingToMap(edge_in,downsampledEdgeCloud,surf_in,downsampledSurfCloud);


    float limits_surf = 50.0;
    float limits_edge = 10.0;


    if(laserCloudCornerMap->points.size()>limits_edge && laserCloudSurfMap->points.size()>limits_surf){
        kdtreeEdgeMap->setInputCloud(laserCloudCornerMap);
        kdtreeSurfMap->setInputCloud(laserCloudSurfMap);

        for (int iterCount = 0; iterCount < optimization_count; iterCount++){
            float loss_value = 0.25;
            ceres::LossFunction *loss_function = new ceres::HuberLoss(loss_value);
            //ceres::LossFunction *loss_function = new ceres::HuberLoss(1);
            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);

            problem.AddParameterBlock(parameters, 7, new PoseSE3Parameterization());
            
            addEdgeCostFactor(downsampledEdgeCloud,laserCloudCornerMap,problem,loss_function);
            addSurfCostFactor(downsampledSurfCloud,laserCloudSurfMap,problem,loss_function);

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 6;
            options.gradient_check_relative_precision = 1e-4;

            options.minimizer_progress_to_stdout = false;
            options.check_gradients = false;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            
        }
    }else{
        printf("not enough points in map to associate, map error\n");
    }
    odom = Eigen::Isometry3d::Identity();
    odom.linear() = q_w_curr.toRotationMatrix();
    odom.translation() = t_w_curr;
    addPointsToMap(downsampledEdgeCloud,downsampledSurfCloud , clear_map, cropBox_len);

}

void OdomEstimationClass::pointAssociateToMap(pcl::PointXYZ const *const pi, pcl::PointXYZ *const po)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    //po->intensity = pi->intensity;

}

void OdomEstimationClass::downSamplingToMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& edge_pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& edge_pc_out, const pcl::PointCloud<pcl::PointXYZ>::Ptr& surf_pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& surf_pc_out){
    downSizeFilterEdge.setInputCloud(edge_pc_in);
    downSizeFilterEdge.filter(*edge_pc_out);
    downSizeFilterSurf.setInputCloud(surf_pc_in);
    downSizeFilterSurf.filter(*surf_pc_out);
}

void OdomEstimationClass::addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function){
    int corner_num=0;

    float dist = 1.0;
    int nearK = 5;

    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        pcl::PointXYZ point_temp;
        pointAssociateToMap(&(pc_in->points[i]), &point_temp);

        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;       

        kdtreeEdgeMap->nearestKSearch(point_temp, nearK, pointSearchInd, pointSearchSqDis);
        if (pointSearchSqDis[4] < dist) 
        {
            std::vector<Eigen::Vector3d> nearCorners;
            Eigen::Vector3d center(0, 0, 0);
            for (int j = 0; j < nearK; j++)
            {
                Eigen::Vector3d tmp(map_in->points[pointSearchInd[j]].x,
                                    map_in->points[pointSearchInd[j]].y,
                                    map_in->points[pointSearchInd[j]].z);
                center = center + tmp;
                nearCorners.push_back(tmp);
            }
            center = center / (nearK*1.0);

            Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
            for (int j = 0; j < nearK; j++)
            {
                Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

            Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
            {
                Eigen::Vector3d point_on_line = center;
                Eigen::Vector3d point_a, point_b;
                point_a = 0.1 * unit_direction + point_on_line;
                point_b = -0.1 * unit_direction + point_on_line;

                ceres::CostFunction *cost_function = new EdgeAnalyticCostFunction(curr_point, point_a, point_b);
                problem.AddResidualBlock(cost_function, loss_function, parameters);
                corner_num++;
            }
        }
    }
    if(corner_num<20){
        printf("not enough Edge correct points\n");
    }

}

void OdomEstimationClass::addSurfCostFactor(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function){
    int surf_num=0;

    float dist = 1.0;
    const int nearK = 5;

    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        pcl::PointXYZ point_temp;
        pointAssociateToMap(&(pc_in->points[i]), &point_temp);
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeSurfMap->nearestKSearch(point_temp, nearK, pointSearchInd, pointSearchSqDis);

        Eigen::Matrix<double, nearK, 3> matA0;
        Eigen::Matrix<double, nearK, 1> matB0 = -1 * Eigen::Matrix<double, nearK, 1>::Ones();
        if (pointSearchSqDis[4] < dist)
        {

            for (int j = 0; j < nearK; j++)
            {
                matA0(j, 0) = map_in->points[pointSearchInd[j]].x;
                matA0(j, 1) = map_in->points[pointSearchInd[j]].y;
                matA0(j, 2) = map_in->points[pointSearchInd[j]].z;
            }
            // find the norm of plane
            Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
            double negative_OA_dot_norm = 1 / norm.norm();
            norm.normalize();

            bool planeValid = true;
            for (int j = 0; j < nearK; j++)
            {
                // if OX * n > 0.2, then plane is not fit well
                if (fabs(norm(0) * map_in->points[pointSearchInd[j]].x +
                         norm(1) * map_in->points[pointSearchInd[j]].y +
                         norm(2) * map_in->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                {
                    planeValid = false;
                    break;
                }
            }
            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            if (planeValid)
            {
                ceres::CostFunction *cost_function = new SurfNormAnalyticCostFunction(curr_point, norm, negative_OA_dot_norm);
                problem.AddResidualBlock(cost_function, loss_function, parameters);

                surf_num++;
            }
        }

    }
    if(surf_num<20){
        printf("not enough Surf correct points\n");
    }

}

void OdomEstimationClass::addPointsToMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& downsampledEdgeCloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& downsampledSurfCloud, bool clear_map, double cropBox_len){


    if(clear_map){
        occlude_pcd(laserCloudCornerMap,50,0,0);
        //laserCloudCornerMap->clear();
        occlude_pcd(laserCloudSurfMap,50,0,0);
       // laserCloudSurfMap->clear();
       
    }

    for (int i = 0; i < (int)downsampledEdgeCloud->points.size(); i++)
    {
        pcl::PointXYZ point_temp;
        pointAssociateToMap(&downsampledEdgeCloud->points[i], &point_temp);
        laserCloudCornerMap->push_back(point_temp);
    }


    for (int j = 0; j < (int)downsampledSurfCloud->points.size(); j++)
    {
        pcl::PointXYZ point_temp;
        pointAssociateToMap(&downsampledSurfCloud->points[j], &point_temp);
        laserCloudSurfMap->push_back(point_temp);

    }



    int conti = laserCloudCornerMap->size();
    int contj = laserCloudSurfMap->size();
    ROS_INFO("Size: Corner: %d, Surf: %d", conti,contj );

    double x_min = odom.translation().x()-cropBox_len;
    double y_min = odom.translation().y()-cropBox_len;
    double z_min = odom.translation().z()-cropBox_len;
    double x_max = odom.translation().x()+cropBox_len;
    double y_max = odom.translation().y()+cropBox_len;
    double z_max = odom.translation().z()+cropBox_len;
    
    //ROS_INFO("size : %f,%f,%f,%f,%f,%f", x_min, y_min, z_min,x_max, y_max, z_max);
    cropBoxFilter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
    cropBoxFilter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
    cropBoxFilter.setNegative(false);    

    pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCorner(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmpSurf(new pcl::PointCloud<pcl::PointXYZ>());

    cropBoxFilter.setInputCloud(laserCloudSurfMap);
    cropBoxFilter.filter(*tmpSurf);
    cropBoxFilter.setInputCloud(laserCloudCornerMap);
    cropBoxFilter.filter(*tmpCorner);

    downSizeFilterSurf.setInputCloud(tmpSurf);    
    downSizeFilterSurf.filter(*laserCloudSurfMap);
    downSizeFilterEdge.setInputCloud(tmpCorner);
    downSizeFilterEdge.filter(*laserCloudCornerMap);
}



void OdomEstimationClass::occlude_pcd(pcl::PointCloud<pcl::PointXYZ>::Ptr & cld_ptr,int dim, double threshA, double threshB)
{
    for(pcl::PointCloud<pcl::PointXYZ>::iterator it = cld_ptr->begin()+threshA;  it < cld_ptr->end()-threshB;it+=dim)
    {
        cld_ptr->erase(it);

     }
}

OdomEstimationClass::OdomEstimationClass(){

}

