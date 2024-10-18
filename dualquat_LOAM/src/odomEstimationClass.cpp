// Author of LiDAR-Odometry_DQ: Edison Velasco
// Email edison.velasco@ua.es

#include "odomEstimationClass.h"
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <chrono>
#include <pcl/filters/filter.h>
#include "dualquatFunctions.h"

#include <fstream> // Incluir la librería para manejo de archivos
#include <string>  // Incluir la librería para std::string

using namespace DQ_robotics;

//std::string path_estadistic = "/home/ws/src/parque_cientifico/estadisticas.txt";
//std::ofstream org_outputFile(path_estadistic);   

struct EdgeCostFunction {
    EdgeCostFunction(const Eigen::Matrix<double,8,1>& LocalTransformacion, const Eigen::Matrix<double,8,1>& line, const double& factor_line)
        : LocalTransformacion(LocalTransformacion), line(line), factor_line(factor_line) {}

    template <typename T>
    bool operator()(const T* const parameters, T* residuals) const {

        //Eigen::Matrix<T, 8, 1> udq(parameters);
        //Eigen::Matrix<T, 8, 1> dual_quat_last(parameters);// = Eigen::Map<Eigen::Matrix<T, 8, 1>>(parameters[0]);
        Eigen::Matrix<T, 8, 1> dual_quat_last = Eigen::Map<const Eigen::Matrix<T, 8, 1>>(parameters);
        Eigen::Matrix<T, 8, 1> pulck_line = line.template cast<T>();
        Eigen::Matrix<T, 8, 1> localtf = LocalTransformacion.template cast<T>(); 
        Eigen::Matrix<T, 4, 1> local_point = get_translation_dual(localtf);

        ////////////////////////////// funcion costo en duales/////////////////////////////

        Eigen::Matrix<T, 8, 1> Vi;
        Vi << T(1.0), T(0.0), T(0.0), T(0.0), T(0.0), local_point[1], local_point[2], local_point[3];

        Eigen::Matrix<T, 8, 1> Qc = dq_conjugate(dual_quat_last);
        Qc(4) = -Qc(4);
        Qc(5) = -Qc(5);
        Qc(6) = -Qc(6);
        Qc(7) = -Qc(7); 

        Eigen::Matrix<T, 8, 1> Vf = dualquatMult(dual_quat_last,dualquatMult(Vi,Qc));
        Eigen::Matrix<T, 4, 1> transformed_source(T(0.0),Vf[5],Vf[6],Vf[7]);

        Eigen::Matrix<T, 4, 1> l_T(pulck_line[0],pulck_line[1],pulck_line[2],pulck_line[3]);
        Eigen::Matrix<T, 4, 1> m_T(pulck_line[4],pulck_line[5],pulck_line[6],pulck_line[7]);

        /////// Point to Pluckerline distance 

        Eigen::Matrix<T, 4, 1> a = cross_quat(transformed_source, l_T);
        a(0) = a(0)- m_T(0);
        a(1) = a(1)- m_T(1);
        a(2) = a(2)- m_T(2);
        a(3) = a(3)- m_T(3);
              
        T square_norm = a(0)*a(0) + a(1)*a(1) + a(2)*a(2) +a(3)*a(3);
        residuals[0] =  ceres::sqrt(square_norm);
        
        return true;
    }

private:
    const Eigen::Matrix<double,8,1> LocalTransformacion;
    const Eigen::Matrix<double,8,1> line;
    const double factor_line;

};

struct SurfCostFunction {
    SurfCostFunction(const Eigen::Matrix<double,8,1>& LocalTransformacion, const Eigen::Matrix<double,8,1>& plane, const double& averageDistance )
        : LocalTransformacion(LocalTransformacion), plane(plane), averageDistance(averageDistance) {}

    template <typename T>
    bool operator()(const T* const parameters, T* residuals) const {

        //Eigen::Matrix<T, 8, 1> udq(parameters);
        //Eigen::Matrix<T, 8, 1> dual_quat(parameters); //= Eigen::Map<Eigen::Matrix<T, 8, 1>>(parameters[0]);
        Eigen::Map<const Eigen::Matrix<T, 8, 1>> dual_quat(parameters);
        Eigen::Matrix<T, 8, 1> Q = dual_quat;

        Eigen::Matrix<T, 8, 1> plane_T = plane.template cast<T>();

        Eigen::Matrix<T, 8, 1> localtf = LocalTransformacion.template cast<T>(); 
        Eigen::Matrix<T, 4, 1> local_point = get_translation_dual(localtf);

        ////////////////////////////// funcion costo en duales/////////////////////////////

        Eigen::Matrix<T, 8, 1> Vi;
        Vi << T(1.0), T(0.0), T(0.0), T(0.0), T(0.0), local_point[1], local_point[2], local_point[3];

        Eigen::Matrix<T, 8, 1> Qc = dq_conjugate(Q);
        Qc(4) = -Qc(4);
        Qc(5) = -Qc(5);
        Qc(6) = -Qc(6);
        Qc(7) = -Qc(7); 

        Eigen::Matrix<T, 8, 1> Vf = dualquatMult(Q,dualquatMult(Vi,Qc));

        Eigen::Matrix<T, 4, 1> transformed_source(T(0.0),Vf[5],Vf[6],Vf[7]);

        ////// Point to Plane distance 

        Eigen::Matrix<T, 4, 1> n_pi(plane_T[0],plane_T[1],plane_T[2],plane_T[3]);
        T d_pi = plane_T[4]; // le resto de los elementos es 0 porque asi se define el plano;
       
        Eigen::Matrix<T, 4, 1> a = dot_quat(transformed_source, n_pi);
        residuals[0] =  a(0)- d_pi;

        return true;
    }

private:
    const Eigen::Matrix<double,8,1> LocalTransformacion;
    const Eigen::Matrix<double,8,1> plane;
    const double averageDistance;

};

struct STDCostFunction {
    STDCostFunction(const Eigen::Matrix<double,8,1>& stdC_dq, const Eigen::Matrix<double,8,1>& stdM_dq)
        : stdC_dq(stdC_dq), stdM_dq(stdM_dq) {}   

    template <typename T>
    bool operator()(const T* const parameters, T* residuals) const {

        Eigen::Matrix<T, 8, 1> Q_M = stdM_dq.template cast<T>(); 
        Eigen::Matrix<T, 8, 1> Q_C = stdC_dq.template cast<T>(); 

        Eigen::Matrix<T, 8, 1> Q_opti = Eigen::Map<const Eigen::Matrix<T, 8, 1>>(parameters);

        Eigen::Matrix<T, 8, 1> Vf = dualquatMult(dq_conjugate(Q_M),dualquatMult(Q_C,Q_opti));
        Eigen::Matrix<T, 8, 1> Vf_abs = Vf;
        if (Vf[0]<T(0.0))
            Vf_abs = -Vf;
        //Vf_abs[0] = abs(Vf_abs[0]);

        Eigen::Matrix<T, 8, 1> dq_unit;
        dq_unit<<T(1.0),T(0.0),T(0.0),T(0.0),T(0.0),T(0.0),T(0.0),T(0.0);


        dq_unit = dq_unit - Vf_abs;

        T norm_real = dq_unit.head(4).norm();
        T norm_dual = dq_unit.tail(4).norm();
        T norm_dq = ceres::sqrt(norm_real * norm_real + norm_dual * norm_dual);
        
        residuals[0] =  norm_dq;
        return true;
    }


    // template <typename T>
    // bool operator()(const T* const parameters, T* residuals) const {

    //     Eigen::Matrix<T, 8, 1> Q_M = stdM_dq.template cast<T>(); 
    //     Eigen::Matrix<T, 8, 1> Q_C = stdC_dq.template cast<T>(); 

    //     Eigen::Map<const Eigen::Matrix<T, 8, 1>> dual_quat(parameters);
    //     Eigen::Matrix<T, 8, 1> Q = dual_quat;

    //     Eigen::Matrix<T, 4, 1> Q_traslation = get_translation_dual(Q);

    //     Eigen::Matrix<T, 4, 1> M_point = get_translation_dual(Q_M);
    //     Eigen::Matrix<T, 4, 1> C_point = get_translation_dual(Q_C);

    //     Eigen::Quaternion<T> eigen_quaternion;
    //     eigen_quaternion.w() = parameters[0];
    //     eigen_quaternion.x() = parameters[1];
    //     eigen_quaternion.y() = parameters[2]; 
    //     eigen_quaternion.z() = parameters[3];

    //     Eigen::Matrix<T, 4, 1> eigen_translation(Q_traslation[0],Q_traslation[1],Q_traslation[2],Q_traslation[3]);

    //     Eigen::Matrix<T, 4, 1> source_T(C_point[0], C_point[1],C_point[2], C_point[3]);

    //     Eigen::Matrix<T, 4, 1> transformed_source = quaternion_left(eigen_quaternion)* 
    //                                                 quaternion_right(conjugate_quaternion(eigen_quaternion))* 
    //                                                 source_T + eigen_translation;

    //     residuals[0] = transformed_source[1] - M_point[1];
    //     residuals[1] = transformed_source[2] - M_point[2];
    //     residuals[2] = transformed_source[3] - M_point[3];

    //     return true;
    // }

private:
    const Eigen::Matrix<double,8,1> stdC_dq;
    const Eigen::Matrix<double,8,1> stdM_dq;
};

 
struct UDQManifold {

    template<typename T>
    bool Plus(const T* x, const T* delta, T* x_plus_delta) const {

        Eigen::Matrix<T, 8, 1> udq(x);
        Eigen::Matrix<T,8,1> delta_dq;

        delta_dq << T(0.0), delta[0],delta[1],delta[2],T(0.0),delta[3],delta[4],delta[5];

        Eigen::Matrix<T,8,1> udq_plus_delta;

        udq_plus_delta = dualquatMult(exp_dq(delta_dq),udq);

        x_plus_delta[0] = udq_plus_delta(0);
        x_plus_delta[1] = udq_plus_delta(1);
        x_plus_delta[2] = udq_plus_delta(2);
        x_plus_delta[3] = udq_plus_delta(3);
        x_plus_delta[4] = udq_plus_delta(4);
        x_plus_delta[5] = udq_plus_delta(5);
        x_plus_delta[6] = udq_plus_delta(6);
        x_plus_delta[7] = udq_plus_delta(7);


        return true;
    }

    template<typename T>
    bool Minus(const T* y, const T* x, T* y_minus_x) const {


        Eigen::Matrix<T, 8, 1> udq;
        udq << x[0],x[1],x[2],x[3],x[4],x[5],x[6],x[7];

        Eigen::Matrix<T, 8, 1> y_dq;
        y_dq << y[0],y[1],y[2],y[3],y[4],y[5],y[6],y[7];

        Eigen::Matrix<T,8,1> udq_y_minus_x;
        udq_y_minus_x = log_dq(dualquatMult(y_dq,dq_conjugate(udq)));

        y_minus_x[0] = udq_y_minus_x(1);
        y_minus_x[1] = udq_y_minus_x(2);
        y_minus_x[2] = udq_y_minus_x(3);
        y_minus_x[3] = udq_y_minus_x(5);
        y_minus_x[4] = udq_y_minus_x(6);
        y_minus_x[5] = udq_y_minus_x(7);

      
        return true;
    }
};

void OdomEstimationClass::init(double edge_resolution, double surf_resolution){
    //init local map
    laserCloudCornerMap = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    laserCloudSurfMap = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

    //downsampling size
    downSizeFilterEdge.setLeafSize(edge_resolution, edge_resolution, edge_resolution);
    downSizeFilterSurf.setLeafSize(surf_resolution, surf_resolution, surf_resolution);

    //kd-tree Edges
    kdtreeEdgeMap  = pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    kdtreeLocalMap = pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZ>());

    //kd-tree Surfaces
    kdtreeSurfMap  = pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZ>());

    odom = Eigen::Isometry3d::Identity();
    last_odom = Eigen::Isometry3d::Identity();
    optimization_count=2;
}

void OdomEstimationClass::initMapWithPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr& surf_in){
    *laserCloudCornerMap += *edge_in;
    *laserCloudSurfMap += *surf_in;

    //optimization_count=4;
    optimization_count=12;
    std::cout<<"mapa inicializado: "<<std::endl;
    //org_outputFile << "Estadisticas con "<<optimization_count<<" lazos FOR al iniciar"<<std::endl;

  
}

void OdomEstimationClass::updatePointsToMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr& surf_in, 
                                            const std::vector<STDesc>& stdC_pair, const std::vector<STDesc>& stdM_pair,  bool clear_map, double cropBox_len, double cont_opti){

    if(optimization_count>cont_opti)
        optimization_count--;

    Eigen::Isometry3d odom_prediction = odom * (last_odom.inverse() * odom);
    last_odom = odom;
    odom = odom_prediction;

    Eigen::Quaterniond quat_real = Eigen::Quaterniond(odom.rotation());
    Eigen::Vector3d translation = odom.translation();

    ///// Asignacion de los datos anteriores de oemetria al nuevo quaternion dual

    Eigen::Matrix<double, 8, 1> getUDQ;
    getUDQ = ToDQ_T(quat_real,translation);
    dual_quat = Eigen::Map<Eigen::Matrix<double, 8, 1>>(getUDQ.data());

    parameters[0] = dual_quat(0);
    parameters[1] = dual_quat(1);
    parameters[2] = dual_quat(2);
    parameters[3] = dual_quat(3);
    parameters[4] = dual_quat(4);
    parameters[5] = dual_quat(5);
    parameters[6] = dual_quat(6);
    parameters[7] = dual_quat(7);


    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledEdgeCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledSurfCloud(new pcl::PointCloud<pcl::PointXYZ>());

    geometry_msgs::PoseArrayConstPtr std_in;

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledEdgeLocalCloud(new pcl::PointCloud<pcl::PointXYZ>());
    downSizeFilterEdge.setInputCloud(edge_in);
    downSizeFilterEdge.filter(*downsampledEdgeLocalCloud);

    downSamplingToMap(edge_in,downsampledEdgeCloud,surf_in,downsampledSurfCloud);

    long unsigned int limits_edge = 10;
    long unsigned int limits_surf = 50;

    if(laserCloudCornerMap->points.size()>limits_edge && laserCloudSurfMap->points.size()>limits_surf){

        kdtreeEdgeMap->setInputCloud(laserCloudCornerMap);
        kdtreeLocalMap->setInputCloud(downsampledEdgeLocalCloud);

        kdtreeSurfMap->setInputCloud(laserCloudSurfMap);
      
        for (int iterCount = 0; iterCount < optimization_count; iterCount++){
            float loss_value = 0.25; 
            ceres::LossFunction *loss_function = new ceres::HuberLoss(loss_value); //  ceres::SoftLOneLoss(loss_value);
            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);

            ceres::Manifold *dq_manifold = new ceres::AutoDiffManifold<UDQManifold, 8, 6>;
            problem.AddParameterBlock(parameters, 8, dq_manifold);
            problem.SetManifold(parameters,dq_manifold);  

            // Parameterized Cost Functions in dual quaternions 
            addEdgeDQCostFactor(downsampledEdgeCloud,laserCloudCornerMap,problem,loss_function,dq_manifold,cropBox_len);
            addSurfDQCostFactor(downsampledSurfCloud,laserCloudSurfMap,  problem,loss_function,dq_manifold,cropBox_len);
            addSTDCostFactor(stdC_pair, stdM_pair,  problem,loss_function,dq_manifold);

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 100; // 100 para Kitti
            options.gradient_check_relative_precision = 1e-4;  
            options.minimizer_progress_to_stdout = false;
            options.check_gradients = false;
            options.num_threads = 10;


            // // Redirige la salida estándar al archivo
            // std::streambuf *coutbuf = std::cout.rdbuf(); // Guarda el buffer de la salida estándar
            // std::cout.rdbuf(org_outputFile.rdbuf()); // Redirige la salida estándar al archivo

            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);

            // // Restaura la salida estándar
            // std::cout.rdbuf(coutbuf);

            // std::cout << summary.BriefReport() << std::endl;
            // std::cout << summary.FullReport();
            // printf("*********************************************************:\n");
            
            // printf("*********************************************************:\n");

            // std::cout << "Iteraciones exitosas: " << summary.num_successful_steps << std::endl;
            // org_outputFile << "Iteraciones exitosas: " << summary.num_successful_steps << std::endl;
            // org_outputFile << summary.BriefReport() << std::endl;
            // org_outputFile << summary.FullReport() << std::endl;
            // org_outputFile << "*********************************************************" << std::endl;

        }
        // Eigen::Matrix<double, 8, 1> dq_data = Eigen::Map<Eigen::Matrix<double, 8, 1>>(parameters);
        // org_outputFile<<"Resuelto: "<<dq_data.transpose()<<std::endl;
        // org_outputFile<< "______________________________________________________________________" << std::endl;
        
    }else{
        printf("not enough points in map to associate, map error\n");
    }
      
                  
    //Converitr los datos de quaternion dual a datos de q y t para enviar a odometria
    //Eigen::Matrix<double, 8, 1> dual_quat(parameters);
    quat_real.w() = dual_quat(0);
    quat_real.x() = dual_quat(1);
    quat_real.y() = dual_quat(2);
    quat_real.z() = dual_quat(3);

    Eigen::Matrix<double, 8, 1> getTraslation = dual_quat;
    Eigen::Matrix<double,3,1> param_tr=get_translation(getTraslation);
    translation.x() = param_tr(0);
    translation.y() = param_tr(1);
    translation.z() = param_tr(2);

    odom = Eigen::Isometry3d::Identity();
    odom.linear() = quat_real.toRotationMatrix();
    odom.translation() = translation;
    
    addPointsToMap(downsampledEdgeCloud,downsampledSurfCloud , clear_map, cropBox_len);

}

void OdomEstimationClass::pointAssociateToMap(pcl::PointXYZ const *const pi, pcl::PointXYZ *const po)
{
    
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Quaterniond quat_real;
    Eigen::Vector3d translation;

    quat_real.w() = dual_quat(0);
    quat_real.x() = dual_quat(1);
    quat_real.y() = dual_quat(2);
    quat_real.z() = dual_quat(3);
    
    Eigen::Matrix<double, 8, 1> getTraslation = dual_quat;
    Eigen::Matrix<double,3,1> param_tr=get_translation(getTraslation);
    translation.x() = param_tr(0);
    translation.y() = param_tr(1);
    translation.z() = param_tr(2);

    Eigen::Vector3d point_w = quat_real * point_curr + translation;
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();

}


void OdomEstimationClass::pointLocalToMap(pcl::PointXYZ const *const pi, pcl::PointXYZ *const po)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    po->x = point_curr.x();
    po->y = point_curr.y();
    po->z = point_curr.z();

}

void OdomEstimationClass::downSamplingToMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& edge_pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& edge_pc_out, const pcl::PointCloud<pcl::PointXYZ>::Ptr& surf_pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& surf_pc_out){
    downSizeFilterEdge.setInputCloud(edge_pc_in);
    downSizeFilterEdge.filter(*edge_pc_out);
    downSizeFilterSurf.setInputCloud(surf_pc_in);
    downSizeFilterSurf.filter(*surf_pc_out);    
}

void OdomEstimationClass::addEdgeDQCostFactor(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function, ceres::Manifold* dq_manifold, double cropBox_len){
    int corner_num = 0  ;
    int no_corner  = 0 ;    
    int min_edges  = 5.0; 
    float dist     = 1.0;
    int nearK      = 5.0;

   // Eigen::Affine3d LocalTransformacion_line = Eigen::Affine3d::Identity();
    Eigen::Matrix<double,8,1> Local_line_Q;
    Local_line_Q<<1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;

   // Eigen::Affine3d MapTransformacion_line = Eigen::Affine3d::Identity();
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
            if (saes.eigenvalues()[2] >  5*saes.eigenvalues()[1])
            {

                ////////////////////// Factor linea optimzation
                const auto& eigenvalues = saes.eigenvalues();
                double maxEigenvalue = eigenvalues(2);  
                double sumEigenvalues = eigenvalues.sum();
                double factor_line = maxEigenvalue / sumEigenvalues;
                //////////////////////////////////////////////////////////////////////

                Eigen::Vector3d point_on_line = center;
                Eigen::Vector3d point_a, point_b;
                point_a = 0.1 * unit_direction + point_on_line;
                point_b = -0.1 * unit_direction + point_on_line;

                //////////////////////////////////////////////////////////////////////////

                Eigen::Quaterniond cuaternion(1.0,0.0,0.0,0.0);

                // Crear la traslación
                Eigen::Matrix<double,3,1> punto_local_m(curr_point.x(),curr_point.y(),curr_point.z());
                Local_line_Q = ToDQ_T(cuaternion,punto_local_m);
                Eigen::Matrix<double,8,1> line_map = calculatePluckerLine (point_a,point_b);

                //////////////////////////////////////////Cost Function///////////////
                ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<EdgeCostFunction, 1, 8>( new EdgeCostFunction(Local_line_Q, line_map, factor_line));
                problem.AddResidualBlock(cost_function, loss_function, parameters); 
     
                corner_num++;
            }
            else{
                no_corner++;
            }            
        }

       
        if (corner_num == cropBox_len)
            break;
     }
  //  org_outputFile << "OK Edges: " << corner_num<<std::endl;
  //  org_outputFile << "No Edges: " << no_corner<<std::endl;
   // std::cout<<"Linea: "<<corner_num<<", No linea:"<<no_corner<<std::endl;
    if(corner_num<min_edges){
        std::cout<<"At least "<< min_edges <<" points are required for edge matching."<<std::endl;
    }

}

void OdomEstimationClass::addSurfDQCostFactor(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_in, const pcl::PointCloud<pcl::PointXYZ>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function, ceres::Manifold* dq_manifold, double cropBox_len)
{
    int surf_num=0;
    int no_plane =0;
    float dist = 1.0;
    const int nearK = 5.0;

    Eigen::Matrix<double,8,1> Local_plane_Q;
    Local_plane_Q<<1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;

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
                // if OX * n > 0.1, then plane is not fit well
                if (fabs(norm(0) * map_in->points[pointSearchInd[j]].x +
                         norm(1) * map_in->points[pointSearchInd[j]].y +
                         norm(2) * map_in->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.1)
                {
                    planeValid = false;
                    no_plane++;
                    break;
                }

            }

            /////////////// Factor para superficies
            double sumDistances = 0.0;
            int numPoints = nearK;  // Número de puntos considerados
            for (int j = 0; j < numPoints; j++) {
                double distance = fabs(norm(0) * map_in->points[pointSearchInd[j]].x +
                                    norm(1) * map_in->points[pointSearchInd[j]].y +
                                    norm(2) * map_in->points[pointSearchInd[j]].z + negative_OA_dot_norm);
                sumDistances += distance;
            }
            double averageDistance = 1.0-sumDistances / numPoints;            
            ////////////////////

            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            Eigen::Vector3d p = matA0.colwise().mean();
            if (planeValid)
            {
                Eigen::Matrix<double, 8, 1> plane_Q = calculatePlane(norm, p); // plano del mapa

                Eigen::Matrix<double,3,1> curr_surf_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);           
                Eigen::Quaterniond cuaternion(1.0,0.0,0.0,0.0);
                Local_plane_Q = ToDQ_T(cuaternion,curr_surf_point);


                    /////////////////////////////////////////////////////////////////////////////////
                ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<SurfCostFunction, 1, 8>(
                new SurfCostFunction(Local_plane_Q, plane_Q,averageDistance));
                problem.AddResidualBlock(cost_function, loss_function, parameters); 
                // problem.SetManifold(parameters,dq_manifold);
               surf_num++;
            }
            else{
                no_plane++;
            }
        }
                if (surf_num == cropBox_len)
                break;           
        }

   // org_outputFile << "OK Surf: " << surf_num<<std::endl;
   // org_outputFile << "No Surf: " << no_plane<<std::endl;

    if(surf_num<20){
        printf("not enough correct points to plane");
    }

}

void OdomEstimationClass::addSTDCostFactor(std::vector<STDesc> stdC_pair, std::vector<STDesc> stdM_pair, ceres::Problem& problem, ceres::LossFunction *loss_function, ceres::Manifold* dq_manifold)
{
    // odoemtria calculada:
    Eigen::Matrix<double, 8, 1> dq_optimizado(dual_quat);

    for (size_t i = 0; i < stdC_pair.size(); ++i) {

        const auto& descC = stdC_pair[i];
        const auto& descM = stdM_pair[i];

        // Extraer los datos de descC
        Eigen::Matrix3d axesC = descC.calculateReferenceFrame();
        Eigen::Quaterniond quatC(axesC);
        Eigen::Matrix<double,3,1> centerC(descC.center_.x(),descC.center_.y(),descC.center_.z());

        // Extraer los datos de descM
        Eigen::Matrix3d axesM = descM.calculateReferenceFrame();
        Eigen::Quaterniond quatM(axesM);
        Eigen::Matrix<double,3,1> centerM(descM.center_.x(),descM.center_.y(),descM.center_.z());

        // STD descritpor to DQ
        Eigen::Matrix<double,8,1> stdC_dq;
        stdC_dq = ToDQ_T(quatC,centerC);
        Eigen::Matrix<double,8,1> stdM_dq;
        stdM_dq = ToDQ_T(quatM,centerM);    

        ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<STDCostFunction, 1, 8>(new STDCostFunction(stdC_dq, stdM_dq));
        problem.AddResidualBlock(cost_function, loss_function, parameters); 
    } 
   // org_outputFile << "OK stds: " << stdC_pair.size()<<std::endl;

}

void OdomEstimationClass::addPointsToMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& downsampledEdgeCloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& downsampledSurfCloud, bool clear_map, double cropBox_len){

    if(clear_map){
        occlude_pcd(laserCloudCornerMap,50,0,0);
        laserCloudCornerMap->clear();
        occlude_pcd(laserCloudSurfMap,50,0,0);
        laserCloudSurfMap->clear();
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

    double x_min = odom.translation().x()-50;
    double y_min = odom.translation().y()-50;
    double z_min = odom.translation().z()-50;
    double x_max = odom.translation().x()+50;
    double y_max = odom.translation().y()+50;
    double z_max = odom.translation().z()+50;
    
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


