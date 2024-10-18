// Author of DualQuatFunctios: Edison Velasco 
// Email epvelasco1912@gmail.com, edison.velasco@ua.es

#ifndef _DUALQUAT_FUNCTIONS_H_
#define _DUALQUAT_FUNCTIONS_H_

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <dqrobotics/utils/DQ_Constants.h>
#include <dqrobotics/utils/DQ_Geometry.h>
#include <dqrobotics/utils/DQ_Constants.h>
#include <dqrobotics/robot_modeling/DQ_Kinematics.h>

using namespace DQ_robotics;

template<typename T>
Eigen::Quaternion<T> quatMult( Eigen::Quaternion<T> q1,  Eigen::Quaternion<T> q2) {
    Eigen::Quaternion<T> resultQ;


    resultQ.w() = q1.w() * q2.w() - q1.x() * q2.x() - q1.y() * q2.y() - q1.z() * q2.z();
    resultQ.x() = q1.w() * q2.x() + q1.x() * q2.w() + q1.y() * q2.z() - q1.z() * q2.y();
    resultQ.y() = q1.w() * q2.y() + q1.y() * q2.w() + q1.z() * q2.x() - q1.x() * q2.z();
    resultQ.z() = q1.w() * q2.z() + q1.z() * q2.w() + q1.x() * q2.y() - q1.y() * q2.x();

    return resultQ;
}

template<typename T>
Eigen::Matrix<T,4,1> quatMult_mT( const Eigen::Matrix<T,4,1>& q1, const Eigen::Matrix<T,4,1>& q2) {
    Eigen::Matrix<T,4,1> resultQ;


    resultQ(0) = q1(0) * q2(0) - q1(1) * q2(1) - q1(2) * q2(2) - q1(3) * q2(3);
    resultQ(1) = q1(0) * q2(1) + q1(1) * q2(0) + q1(2) * q2(3) - q1(3) * q2(2);
    resultQ(2) = q1(0) * q2(2) + q1(2) * q2(0) + q1(3) * q2(1) - q1(1) * q2(3);
    resultQ(3) = q1(0) * q2(3) + q1(3) * q2(0) + q1(1) * q2(2) - q1(2) * q2(1);

    return resultQ;
}

template<typename T>
Eigen::Matrix<T, 8, 1> ToDQ_T( Eigen::Quaternion<T> rotacion,  Eigen::Matrix<T, 3, 1> translation) {
    
    Eigen::Quaternion<T> r;
    r.w() = rotacion.w();
    r.x() = rotacion.x();
    r.y() = rotacion.y();
    r.z() = rotacion.z();
    
    Eigen::Quaternion<T> t;
    t.w() = T(0.0);
    t.x() = translation(0);
    t.y() = translation(1);
    t.z() = translation(2);


    Eigen::Quaternion<T> d_aux = quatMult(t,r);
    Eigen::Matrix<T, 4, 1> d(d_aux.w(),d_aux.x(),d_aux.y(),d_aux.z());
    d = d*T(0.5);

    Eigen::Matrix<T, 8, 1> res_udq;
    
    res_udq << r.w(),r.x(),r.y(),r.z(),d(0),d(1),d(2),d(3); 

    return res_udq;
}

// template<typename T>
// Eigen::Matrix<T,3,3> skew(const Eigen::Matrix<T,3,1>* mat_in){
//     Eigen::Matrix<T,3,3> skew_mat;
//     skew_mat.setZero();
//     skew_mat(0,1) = -mat_in(2);
//     skew_mat(0,2) =  mat_in(1);
//     skew_mat(1,2) = -mat_in(0);
//     skew_mat(1,0) =  mat_in(2);
//     skew_mat(2,0) = -mat_in(1);
//     skew_mat(2,1) =  mat_in(0);
//     return skew_mat;
// }

template<typename T>
Eigen::Matrix<T,8,1>  getTransformFromSe3(const Eigen::Matrix<T,6,1>& se3){

    Eigen::Matrix<T,3,1> omega(se3(0),se3(1),se3(2));
    Eigen::Matrix<T,3,1> upsilon(se3(3),se3(4),se3(5));
    Eigen::Matrix<T,3,3> Omega;

    Eigen::Quaternion<T> q;
            q.w() = T(1.0);
            q.x() = T(0.0);
            q.y() = T(0.0);
            q.z() = T(0.0);
        
    Eigen::Matrix<T,3,1> t(T(0.0),T(0.0),T(0.0));


    // skew omega
    Omega(0,0) =    T(0.0);
    Omega(0,1) = -omega(2);
    Omega(0,2) =  omega(1);
    Omega(1,1) =    T(0.0);
    Omega(1,2) = -omega(0);
    Omega(1,0) =  omega(2);
    Omega(2,0) = -omega(1);
    Omega(2,1) =  omega(0); 
    Omega(2,2) =    T(0.0);

    //T theta = omega.norm();

    T squared_norm_delta =  omega(0)*omega(0) + omega(1)*omega(1) + omega(2)*omega(2);

   // std::cout<<"squared_norm_delta: "<<squared_norm_delta<<std::endl;

    if (squared_norm_delta == T(0.0)){
        Eigen::Matrix<T,8,1> udq_id;
        udq_id<<T(1.0),T(0.0),T(0.0),T(0.0),T(0.0),T(0.0),T(0.0),T(0.0);

        return udq_id;
    }

    T theta = sqrt(squared_norm_delta);

   // std::cout<<"theta: "<<theta<<std::endl;

    T half_theta = T(0.5)*theta;

    T imag_factor;
    T real_factor = cos(half_theta);
    if(theta<T(1e-10))
    {
        T theta_sq = theta*theta;
        T theta_po4 = theta_sq*theta_sq;
        imag_factor = T(0.5)-T(0.0208333)*theta_sq+T(0.000260417)*theta_po4;
    }
    else
    {
        T sin_half_theta = sin(half_theta);
        imag_factor = sin_half_theta/theta;
    }

    q = Eigen::Quaternion<T>(real_factor, imag_factor*omega(0), imag_factor*omega(1), imag_factor*omega(2));
    

    Eigen::Matrix<T,3,3> J;
    if (theta<T(1e-10))
    {
        J = q.matrix();
    }
    else
    {
        Eigen::Matrix<T,3,3> Omega2 = Omega*Omega;
        Eigen::Matrix<T,3,3> identity_matrix;
        identity_matrix << T(1), T(0), T(0),
                           T(0), T(1), T(0),
                           T(0), T(0), T(1);

        J = (identity_matrix + (T(1)-cos(theta))/(theta*theta)*Omega + (theta-sin(theta))/(pow(theta,3))*Omega2);
    }

    t = J*upsilon;


    Eigen::Matrix<T,8,1> udq = ToDQ_T(q,upsilon);

    return udq;
   
}

Eigen::Matrix<double,8,1> calculatePluckerLine(const Vector3d& p1, const Vector3d& p2) {
    // Calculate the direction and the dual of the Plücker line
    //Eigen::Vector3d direction = p1.cross(p2);
    Eigen::Vector3d direction = p1-p2;

    // Normalize the direction
    direction.normalize();
    DQ l1(0,direction.x(),direction.y(),direction.z(),0,0,0,0);
    DQ p1_dq(0,p1.x(),p1.y(),p1.z());
    DQ m1 = DQ_robotics::cross(p1_dq,l1);
   
    DQ Pl_dq = l1 + E_*m1; 
    Eigen::Matrix<double,8,1> Pl_dq_eigen = vec8(Pl_dq);

    return Pl_dq_eigen;
}

// Function to fit a plane to a set of sample points
void fit_plane(const MatrixXd& X, Vector3d& n, Vector3d& p) {
    // Calculate the midpoint of the samples, which belongs to the plane
    p = X.colwise().mean();

    // Center the samples by subtracting the midpoint
    MatrixXd R = X.rowwise() - p.transpose();

    // Calculate the principal directions of the sample cloud using eigenvalue decomposition
    SelfAdjointEigenSolver<MatrixXd> eigensolver(R.transpose() * R);
    VectorXd eigenvalues = eigensolver.eigenvalues();
    MatrixXd eigenvectors = eigensolver.eigenvectors();

    // Extract the eigenvector corresponding to the plane's normal direction (n)
    n = eigenvectors.col(0);

}


Eigen::Matrix<double,8,1> calculatePlane(Vector3d& n, Vector3d& p) {
    
    // Normalize the normal vector
    n.normalize();
    DQ n1(0,n.x(),n.y(),n.z(),0,0,0,0);

    DQ p1_dq(0,p.x(),p.y(),p.z());
    DQ d_pi = DQ_robotics::dot(p1_dq,n1);
   
    DQ plane = n1 + E_*d_pi; 
    Eigen::Matrix<double,8,1> plane_eigen = vec8(plane);
    
    return plane_eigen;
}

// Function to check if points are close enough to the fitted plane
bool isPlane(const MatrixXd& X, const Vector3d& n, const Vector3d& p, double umbral) {
    for (int i = 0; i < X.rows(); ++i) {
        // Calculate the distance of each point to the plane using the plane equation
        double distancia = fabs(n.dot(X.row(i)) - n.dot(p));
        // Check if the distance exceeds the threshold
        if (distancia > umbral) {
            return false; // The point is too far from the plane
        }
    }
    return true; // All points are close enough to the plane
}


template<typename T>
Eigen::Matrix<T,8,1> dualquatMult_2q(Eigen::Quaternion<T> qp1, Eigen::Quaternion<T> qd1, Eigen::Matrix<T,8,1> Q2) {

    Eigen::Matrix<T,8,1> res_mul;

    Eigen::Quaternion<T> q1_r;
            q1_r.w() = qp1.w();
            q1_r.x() = qp1.x();
            q1_r.y() = qp1.y();
            q1_r.z() = qp1.z();
    Eigen::Quaternion<T> q1_d;
            q1_d.w() = qd1.w();
            q1_d.x() = qd1.x();
            q1_d.y() = qd1.y();
            q1_d.z() = qd1.z();

    Eigen::Quaternion<T> q2_r;
            q2_r.w() = Q2(0);
            q2_r.x() = Q2(1);
            q2_r.y() = Q2(2);
            q2_r.z() = Q2(3);            
    Eigen::Quaternion<T> q2_d;
            q2_d.w() = Q2(4);
            q2_d.x() = Q2(5);
            q2_d.y() = Q2(6);
            q2_d.z() = Q2(7);

    res_mul(0) = q1_r.w()*q2_r.w() - q1_r.x()*q2_r.x() - q1_r.y()*q2_r.y() - q1_r.z()*q2_r.z();
    res_mul(1) = q1_r.w()*q2_r.x() + q1_r.x()*q2_r.w() + q1_r.y()*q2_r.z() - q1_r.z()*q2_r.y();
    res_mul(2) = q1_r.w()*q2_r.y() - q1_r.x()*q2_r.z() + q1_r.y()*q2_r.w() + q1_r.z()*q2_r.x();
    res_mul(3) = q1_r.w()*q2_r.z() + q1_r.x()*q2_r.y() - q1_r.y()*q2_r.x() + q1_r.z()*q2_r.w();
    res_mul(4) = q1_r.w()*q2_d.w() - q1_r.x()*q2_d.x() - q1_r.y()*q2_d.y() - q1_r.z()*q2_d.z() + q1_d.w()*q2_r.w() - q1_d.x()*q2_r.x() - q1_d.y()*q2_r.y() - q1_d.z()*q2_r.z();
    res_mul(5) = q1_r.w()*q2_d.x() + q1_r.x()*q2_d.w() + q1_r.y()*q2_d.z() - q1_r.z()*q2_d.y() + q1_d.w()*q2_r.x() + q1_d.x()*q2_r.w() + q1_d.y()*q2_r.z() - q1_d.z()*q2_r.y();
    res_mul(6) = q1_r.w()*q2_d.y() - q1_r.x()*q2_d.z() + q1_r.y()*q2_d.w() + q1_r.z()*q2_d.x() + q1_d.w()*q2_r.y() - q1_d.x()*q2_r.z() + q1_d.y()*q2_r.w() + q1_d.z()*q2_r.x();
    res_mul(7) = q1_r.w()*q2_d.z() + q1_r.x()*q2_d.y() - q1_r.y()*q2_d.x() + q1_r.z()*q2_d.w() + q1_d.w()*q2_r.z() + q1_d.x()*q2_r.y() - q1_d.y()*q2_r.x() + q1_d.z()*q2_r.w();

    return res_mul;
}

template<typename T>
Eigen::Matrix<T,8,1> dualquatMult(const Eigen::Matrix<T,8,1>& q1, const Eigen::Matrix<T,8,1>& q2) {
    Eigen::Matrix<T,8,1> res_mul;


    res_mul << T(q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3]),
               T(q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2]),
               T(q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1]),
               T(q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]),
               T(q1[0] * q2[4] - q1[1] * q2[5] - q1[2] * q2[6] - q1[3] * q2[7] + q1[4] * q2[0] - q1[5] * q2[1] - q1[6] * q2[2] - q1[7] * q2[3]),
               T(q1[0] * q2[5] + q1[1] * q2[4] + q1[2] * q2[7] - q1[3] * q2[6] + q1[4] * q2[1] + q1[5] * q2[0] + q1[6] * q2[3] - q1[7] * q2[2]),
               T(q1[0] * q2[6] - q1[1] * q2[7] + q1[2] * q2[4] + q1[3] * q2[5] + q1[4] * q2[2] - q1[5] * q2[3] + q1[6] * q2[0] + q1[7] * q2[1]),
               T(q1[0] * q2[7] + q1[1] * q2[6] - q1[2] * q2[5] + q1[3] * q2[4] + q1[4] * q2[3] + q1[5] * q2[2] - q1[6] * q2[1] + q1[7] * q2[0]);
    
    return res_mul;
}

// Quaternion left and right multiplication
template<typename T>
Eigen::Matrix<T, 4, 4> quaternion_left(Eigen::Quaternion<T> q) {
    Eigen::Matrix<T, 4, 4> Q1;

    Q1 << q.w(), -q.x(), -q.y(), -q.z(),
          q.x(),  q.w(), -q.z(),  q.y(),
          q.y(),  q.z(),  q.w(), -q.x(),
          q.z(), -q.y(),  q.x(),  q.w();

    return Q1;
}

template<typename T>
Eigen::Matrix<T, 4, 4> quaternion_right(Eigen::Quaternion<T> q) {
    Eigen::Matrix<T, 4, 4> Q1;

    Q1 << q.w(), -q.x(), -q.y(), -q.z(),
          q.x(),  q.w(),  q.z(), -q.y(),
          q.y(), -q.z(),  q.w(),  q.x(),
          q.z(),  q.y(), -q.x(),  q.w();

    return Q1;
}

template<typename T>
Eigen::Quaternion<T> conjugate_quaternion(Eigen::Quaternion<T> q) {
    Eigen::Quaternion<T> conj_q;
    conj_q.w() = q.w();
    conj_q.x() = -q.x();
    conj_q.y() = -q.y();
    conj_q.z() = -q.z();

    return conj_q;
}

template<typename T>
Eigen::Matrix<T, 8, 1> dq_conjugate(Eigen::Matrix<T, 8, 1> dq)
{
    Eigen::Matrix<T, 8, 1> res_conj;
    res_conj << T(dq(0)), T(-dq(1)), T(-dq(2)), T(-dq(3)), T(dq(4)), T(-dq(5)), T(-dq(6)), T(-dq(7));
    return res_conj;
}

template<typename T>
Eigen::Matrix<T, 8, 1> exp_dq(Eigen::Matrix<T, 8, 1> q)
{

    Eigen::Matrix<T, 4, 1> prim_exp;
    Eigen::Matrix<T, 8, 1> res_exp;
    res_exp << T(1.0),T(0.0),T(0.0),T(0.0),T(0.0),T(0.0),T(0.0),T(0.0);
 
    Eigen::Quaternion<T> q_d;
            q_d.w()=q(4);
            q_d.x()=q(5);
            q_d.y()=q(6);
            q_d.z()=q(7); 

   // Eigen::Matrix<T,4,1> prim_dq(q(0),q(1),q(2),q(3));

    T squared_norm_delta =  q(1)*q(1) + q(2)*q(2) + q(3)*q(3);
    Eigen::Quaternion<T> dual_exp;
    
    if(squared_norm_delta > T(1e-10)){

        T phi = sqrt(squared_norm_delta);
        prim_exp(0) = cos(phi);
        prim_exp(1) = (sin(phi)/phi) * q(1);
        prim_exp(2) = (sin(phi)/phi) * q(2);
        prim_exp(3) = (sin(phi)/phi) * q(3);

        Eigen::Quaternion<T> prim_exp_quat;      /// convert from 4x1 matrix to quaternion object
        prim_exp_quat.w()=prim_exp(0);
        prim_exp_quat.x()=prim_exp(1);
        prim_exp_quat.y()=prim_exp(2);
        prim_exp_quat.z()=prim_exp(3);
        dual_exp = quatMult(q_d,prim_exp_quat);

    }

    else{

        prim_exp << T(1.0) ,q(1) ,q(2) , q(3);
        dual_exp.w() = q(4);
        dual_exp.x() = q(5);
        dual_exp.y() = q(6);
        dual_exp.z() = q(7);

    }
  
    res_exp << prim_exp(0) ,prim_exp(1) ,prim_exp(2) ,prim_exp(3), 
               dual_exp.w(),dual_exp.x(),dual_exp.y(),dual_exp.z();

    return res_exp;

}

template<typename T>
Eigen::Matrix<T, 4, 1> cross_quat(Eigen::Matrix<T, 4, 1> q1,  Eigen::Matrix<T, 4, 1> q2) {
    
    
    Eigen::Matrix<T, 4, 1> m1 = quatMult_mT(q1,q2);
    Eigen::Matrix<T, 4, 1> m2 = quatMult_mT(q2,q1);

    Eigen::Matrix<T, 4, 1>  diff_quat(m1(0)-m2(0),
                                      m1(1)-m2(1),
                                      m1(2)-m2(2),
                                      m1(3)-m2(3));
    diff_quat = diff_quat * 0.5;
    Eigen::Matrix<T, 4, 1> res_quat;
    res_quat(0) = diff_quat(0);
    res_quat(1) = diff_quat(1);
    res_quat(2) = diff_quat(2);
    res_quat(3) = diff_quat(3);
    
    return diff_quat;
}

template<typename T>
Eigen::Matrix<T, 4, 1> dot_quat(const Eigen::Matrix<T, 4, 1> q1,  const Eigen::Matrix<T, 4, 1> q2) {


    Eigen::Matrix<T, 4, 1> res_dot = - (quatMult_mT(q1,q2)+quatMult_mT(q2,q1))*0.5;

    //res_dot = q1(0) * q2(0) + q1(1) * q2(1) + q1(2) * q2(2) + q1(3) * q2(3);
    return res_dot;
}

template<typename T>
Eigen::Matrix<T, 4, 1> get_translation_dual( Eigen::Matrix<T, 8, 1> udq) {
    
    Eigen::Quaternion<T> quat;
    quat.w() = udq(0);
    quat.x() = udq(1);
    quat.y() = udq(2);
    quat.z() = udq(3);


    Eigen::Quaternion<T> dual;
    dual.w() = udq(4);
    dual.x() = udq(5);
    dual.y() = udq(6);
    dual.z() = udq(7);

    Eigen::Quaternion<T> t_aux = quatMult(dual,conjugate_quaternion(quat));
    Eigen::Matrix<T, 4, 1> t(t_aux.w(),t_aux.x(),t_aux.y(),t_aux.z());

    t = t * T(2.0);

    Eigen::Matrix<T, 4, 1> res_traslation(T(0.0),t(1),t(2),t(3)); 
    
    return res_traslation;
}

Eigen::Matrix<double,3,1> get_translation( Eigen::Matrix<double, 8, 1> udq) {
    
    Eigen::Quaternion<double> quat;
    quat.w() = udq(0);
    quat.x() = udq(1);
    quat.y() = udq(2);
    quat.z() = udq(3);


    Eigen::Quaternion<double> dual;
    dual.w() = udq(4);
    dual.x() = udq(5);
    dual.y() = udq(6);
    dual.z() = udq(7);

    Eigen::Quaternion<double> t_aux = quatMult(dual,conjugate_quaternion(quat));
    Eigen::Matrix<double, 4, 1> t(t_aux.w(),t_aux.x(),t_aux.y(),t_aux.z());

    t = t * 2.0;

    Eigen::Matrix<double,3,1> res_traslation(t(1),t(2),t(3)); 
    
    return res_traslation;
}

template<typename T>
Eigen::Matrix<T, 8, 1> log_dq(Eigen::Matrix<T, 8, 1> dq)
{

    T squared_norm_delta =  dq(1)*dq(1) + dq(2)*dq(2) + dq(3)*dq(3);
    Eigen::Matrix<T,4,1> aux;
    Eigen::Matrix<T, 8, 1> res_log;
    Eigen::Matrix<T, 4, 1> q;
    Eigen::Matrix<T, 4, 1> d;

    if (squared_norm_delta > T(1e-10)){

        T norm_delta = sqrt(squared_norm_delta);
        T theta = atan2(norm_delta,dq(0));
        aux << theta, dq(1)/norm_delta, dq(2)/norm_delta, dq(3)/norm_delta;

        Eigen::Matrix<T,4,1> aux_2;
        aux_2 << T(0.0), aux(0)*aux(1), aux(0)*aux(2),aux(0)*aux(3);
        q = T(0.5) * aux_2; //primary
        d = T(0.5) * get_translation_dual(dq); //dual

    }
    else
    {
        q << T(0.0),dq(1),dq(2),dq(3);
        d << T(0.0),dq(5),dq(6),dq(7);
    }

    res_log << q(0),q(1),q(2),q(3),d(0),d(1),d(2),d(3);

    return res_log;
}

#endif // _DUALQUAT_FUNCTIONS_H_

