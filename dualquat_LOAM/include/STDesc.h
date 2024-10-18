#pragma once

#include "omp.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <fstream>
#include <mutex>
#include <pcl/common/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sstream>
#include <stdio.h>
#include <string>
#include <unordered_map>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Header.h>

#include <nanoflann.hpp> // nanoflann library

// ROS Pose
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>

#define HASH_P 116101
#define MAX_N 10000000000
#define MAX_FRAME_N 20000

typedef struct ConfigSetting {
  /* for point cloud pre-preocess*/
  int stop_skip_enable_ = 0;
  double ds_size_ = 0.5;
  int maximum_corner_num_ = 30;

  /* for key points*/
  double plane_merge_normal_thre_;
  double plane_merge_dis_thre_;
  double plane_detection_thre_ = 0.01;
  double voxel_size_ = 1.0;
  int voxel_init_num_ = 10;
  double proj_image_resolution_ = 0.5;
  double proj_dis_min_ = 0.2;
  double proj_dis_max_ = 5;
  double corner_thre_ = 10;

  /* for STD */
  int descriptor_near_num_ = 10;
  double descriptor_min_len_ = 1;
  double descriptor_max_len_ = 10;
  double non_max_suppression_radius_ = 3.0;
  double std_side_resolution_ = 0.2;

  /* for place recognition*/
  int skip_near_num_ = 50;
  int candidate_num_ = 50;
  int sub_frame_num_ = 10;
  double rough_dis_threshold_ = 0.03;
  double vertex_diff_threshold_ = 0.7;
  double icp_threshold_ = 0.5;
  double normal_threshold_ = 0.1;
  double dis_threshold_ = 0.3;
  int max_window_size_ = 10;
  double kdtree_threshold_ = 50.0;
  double epsilon_ = 100.0;

} ConfigSetting;

// Structure for Stabel Triangle Descriptor
typedef struct STDesc {
  // the side lengths of STDesc, arranged from short to long
  Eigen::Vector3d side_length_;

  // projection angle between vertices
  Eigen::Vector3d angle_;

  Eigen::Vector3d center_;
  unsigned int frame_id_;

  // three vertexs
  Eigen::Vector3d vertex_A_;
  Eigen::Vector3d vertex_B_;
  Eigen::Vector3d vertex_C_;

  Eigen::Vector3d normal1_, normal2_, normal3_;


  Eigen::Matrix3d calculateReferenceFrame() const {
    // Eje X: Normalizado P3 - centroid
    Eigen::Vector3d x_axis = (vertex_C_ - center_).normalized();

    // Eje Y: Normalizado P2 - P1
    Eigen::Vector3d y_axis = (vertex_B_ - vertex_A_).normalized();

    // Eje Z: Producto cruzado de X y Y
    Eigen::Vector3d z_axis = x_axis.cross(y_axis).normalized();

    // Corregir el axis Y para asegurar que sea ortogonal
    y_axis = z_axis.cross(x_axis).normalized();

    Eigen::Matrix3d axes;
    axes.col(0) = x_axis;
    axes.col(1) = y_axis;
    axes.col(2) = z_axis;

    return axes;
  }

  // Convert the descriptor to a flat vector
    std::vector<double> toVector() const {
        std::vector<double> vec(15);
        vec[0] = side_length_[0]; vec[1] = side_length_[1]; vec[2] = side_length_[2];
        vec[3] = angle_[0]; vec[4] = angle_[1]; vec[5] = angle_[2];
        vec[6] = center_[0]; vec[7] = center_[1]; vec[8] = center_[2];
        vec[9] = vertex_A_[0]; vec[10] = vertex_A_[1]; vec[11] = vertex_A_[2];
        vec[12] = vertex_B_[0]; vec[13] = vertex_B_[1]; vec[14] = vertex_B_[2];
        vec[15] = vertex_C_[0]; vec[16] = vertex_C_[1]; vec[17] = vertex_C_[2];
        return vec;
    }

  // some other inform attached to each vertex,e.g., intensity
  Eigen::Vector3d vertex_attached_;

   void setFromMatrixRow(const Eigen::VectorXf &row) {
        side_length_ = row.segment<3>(0).cast<double>();
        angle_ = row.segment<3>(3).cast<double>();
        center_ = row.segment<3>(6).cast<double>();
        vertex_A_ = row.segment<3>(9).cast<double>();
        vertex_B_ = row.segment<3>(12).cast<double>();
        vertex_C_ = row.segment<3>(15).cast<double>();
        Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::RowMajor>> axes_f(row.segment<9>(18).data());
        // Assuming axes_f can be set directly
        // Note: Adjust if STDesc stores axes differently
    }


} STDesc;

/////////////////////// Descriptor STD BBDD to nanoflann
struct DescriptorCloud {
    std::vector<STDesc> descriptors;

    // Returns the number of data points
    inline size_t kdtree_get_point_count() const { return descriptors.size(); }

    // Returns the dim'th component of the idx'th point in the class
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        std::vector<double> vec = descriptors[idx].toVector();
        return vec[dim];
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop
    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const { return false; }
};

// plane structure for corner point extraction
typedef struct Plane {
  pcl::PointXYZINormal p_center_;
  Eigen::Vector3d center_;
  Eigen::Vector3d normal_;
  Eigen::Matrix3d covariance_;
  float radius_ = 0;
  float min_eigen_value_ = 1;
  float intercept_ = 0;
  int id_ = 0;
  int sub_plane_num_ = 0;
  int points_size_ = 0;
  bool is_plane_ = false;
} Plane;

typedef struct STDMatchList {
  std::vector<std::pair<STDesc, STDesc>> match_list_;
  std::pair<int, int> match_id_;
  double mean_dis_;
} STDMatchList;

class VOXEL_LOC {
public:
  int64_t x, y, z;

  VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
      : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOC &other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

// for down sample function
struct M_POINT {
  float xyz[3];
  float intensity;
  int count = 0;
};

// Hash value

template <> struct std::hash<VOXEL_LOC> {
  int64_t operator()(const VOXEL_LOC &s) const {
    using std::hash;
    using std::size_t;
    return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
  }
};

class STDesc_LOC {
public:
  int64_t x, y, z, a, b, c;

  STDesc_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0, int64_t va = 0,
             int64_t vb = 0, int64_t vc = 0)
      : x(vx), y(vy), z(vz), a(va), b(vb), c(vc) {}

  bool operator==(const STDesc_LOC &other) const {
    // use three attributes
    return (x == other.x && y == other.y && z == other.z);
    // use six attributes
    // return (x == other.x && y == other.y && z == other.z && a == other.a &&
    //         b == other.b && c == other.c);
  }
};

template <> struct std::hash<STDesc_LOC> {
  int64_t operator()(const STDesc_LOC &s) const {
    using std::hash;
    using std::size_t;
    return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
    // return ((((((((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N +
    //              (s.x)) *
    //             HASH_P) %
    //                MAX_N +
    //            s.a) *
    //           HASH_P) %
    //              MAX_N +
    //          s.b) *
    //         HASH_P) %
    //            MAX_N +
    //        s.c;
  }
};

// OctoTree structure for plane detection
class OctoTree {
public:
  ConfigSetting config_setting_;
  std::vector<Eigen::Vector3d> voxel_points_;
  Plane *plane_ptr_;
  int layer_;
  int octo_state_; // 0 is end of tree, 1 is not
  int merge_num_ = 0;
  bool is_project_ = false;
  std::vector<Eigen::Vector3d> proj_normal_vec_;

  // check 6 direction: x,y,z,-x,-y,-z
  bool is_check_connect_[6];
  bool connect_[6];
  OctoTree *connect_tree_[6];

  bool is_publish_ = false;
  OctoTree *leaves_[8];
  double voxel_center_[3]; // x, y, z
  float quater_length_;
  bool init_octo_;
  OctoTree(const ConfigSetting &config_setting)
      : config_setting_(config_setting) {
    voxel_points_.clear();
    octo_state_ = 0;
    layer_ = 0;
    init_octo_ = false;
    for (int i = 0; i < 8; i++) {
      leaves_[i] = nullptr;
    }
    for (int i = 0; i < 6; i++) {
      is_check_connect_[i] = false;
      connect_[i] = false;
      connect_tree_[i] = nullptr;
    }
    plane_ptr_ = new Plane;
  }
  // Destructor: Liberar toda la memoria asignada dinámicamente
  ~OctoTree() {
    // Liberar la memoria
    delete plane_ptr_;
  }
  void init_plane();
  void init_octo_tree();
};

void down_sampling_voxel(pcl::PointCloud<pcl::PointXYZI> &pl_feat,
                         double voxel_size);

void load_pose_with_time(
    const std::string &pose_file,
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> &poses_vec,
    std::vector<double> &times_vec);

void read_parameters(ros::NodeHandle &nh, ConfigSetting &config_setting);

double time_inc(std::chrono::_V2::system_clock::time_point &t_end,
                std::chrono::_V2::system_clock::time_point &t_begin);

pcl::PointXYZI vec2point(const Eigen::Vector3d &vec);
Eigen::Vector3d point2vec(const pcl::PointXYZI &pi);

void publish_std_pairs(
    const std::vector<std::pair<STDesc, STDesc>> &match_std_pairs,
    const ros::Publisher &std_publisher);

bool attach_greater_sort(std::pair<double, int> a, std::pair<double, int> b);

struct PlaneSolver {
  PlaneSolver(Eigen::Vector3d curr_point_, Eigen::Vector3d curr_normal_,
              Eigen::Vector3d target_point_, Eigen::Vector3d target_normal_)
      : curr_point(curr_point_), curr_normal(curr_normal_),
        target_point(target_point_), target_normal(target_normal_){};
  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const {
    Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
    Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
    Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()),
                              T(curr_point.z())};
    Eigen::Matrix<T, 3, 1> point_w;
    point_w = q_w_curr * cp + t_w_curr;
    Eigen::Matrix<T, 3, 1> point_target(
        T(target_point.x()), T(target_point.y()), T(target_point.z()));
    Eigen::Matrix<T, 3, 1> norm(T(target_normal.x()), T(target_normal.y()),
                                T(target_normal.z()));
    residual[0] = norm.dot(point_w - point_target);
    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_,
                                     const Eigen::Vector3d curr_normal_,
                                     Eigen::Vector3d target_point_,
                                     Eigen::Vector3d target_normal_) {
    return (
        new ceres::AutoDiffCostFunction<PlaneSolver, 1, 4, 3>(new PlaneSolver(
            curr_point_, curr_normal_, target_point_, target_normal_)));
  }

  Eigen::Vector3d curr_point;
  Eigen::Vector3d curr_normal;
  Eigen::Vector3d target_point;
  Eigen::Vector3d target_normal;
};

class STDescManager {
public:
  STDescManager() = default;

  ConfigSetting config_setting_;

  unsigned int current_frame_id_;

  STDescManager(ConfigSetting &config_setting)
      : config_setting_(config_setting) {
    current_frame_id_ = 0;
  };

  // hash table, save all descriptors
  std::unordered_map<STDesc_LOC, std::vector<STDesc>> data_base_;

  // save all key clouds, optional
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> key_cloud_vec_;

  // save all corner points, optional
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> corner_cloud_vec_;

  // save all planes of key frame, required
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> plane_cloud_vec_;

  /*Three main processing functions*/

  // generate STDescs from a point cloud
  void GenerateSTDescs(pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
                       std::vector<STDesc> &stds_vec);

  // search result <candidate_id, plane icp score>. -1 for no loop
  void SearchLoop(const std::vector<STDesc> &stds_vec,
                  std::pair<int, double> &loop_result,
                  std::pair<Eigen::Vector3d, Eigen::Matrix3d> &loop_transform,
                  std::vector<std::pair<STDesc, STDesc>> &loop_std_pair);

  // add descriptors to database
  void AddSTDescs(const std::vector<STDesc> &stds_vec);

  // Geometrical optimization by plane-to-plane ico
  void PlaneGeomrtricIcp(
      const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &source_cloud,
      const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &target_cloud,
      std::pair<Eigen::Vector3d, Eigen::Matrix3d> &transform);

  void MatchConsecutiveFrames(const std::vector<STDesc>& prev_descs, const std::vector<STDesc>& curr_descs, std::vector<std::pair<STDesc, STDesc>>& matched_pairs);
  void publish_matched_pairs(const std::vector<std::pair<STDesc, STDesc>>& matched_pairs, const ros::Publisher& pub);
  void publishAxes(const ros::Publisher& marker_pub, const std::vector<STDesc>&, const std_msgs::Header& header);
  void publishPoses(const ros::Publisher& pose_pub, const std::vector<STDesc>& descs, const std_msgs::Header& header, const std::string frame_id);

  //void publishAxes(const ros::Publisher& marker_pub, const std::vector<STDesc>&, const std_msgs::Header& header);

  void clearDatabase();

private:
  /*Following are sub-processing functions*/

  // voxelization and plane detection
  void init_voxel_map(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
                      std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map);

  // build connection for planes
  void build_connection(std::unordered_map<VOXEL_LOC, OctoTree *> &feat_map);

  // acquire planes from voxel_map
  void getPlane(const std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
                pcl::PointCloud<pcl::PointXYZINormal>::Ptr &plane_cloud);

  // extract corner points from pre-build voxel map and clouds
  void
  corner_extractor(std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
                   pcl::PointCloud<pcl::PointXYZINormal>::Ptr &corner_points);

  void
  extract_corner(const Eigen::Vector3d &proj_center,
                 const Eigen::Vector3d proj_normal,
                 const std::vector<Eigen::Vector3d> proj_points,
                 pcl::PointCloud<pcl::PointXYZINormal>::Ptr &corner_points);

  // non maximum suppression, to control the number of corners
  void non_maxi_suppression(
      pcl::PointCloud<pcl::PointXYZINormal>::Ptr &corner_points);

  // build STDescs from corner points.
  void
  build_stdesc(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &corner_points,
               std::vector<STDesc> &stds_vec);

  // Select a specified number of candidate frames according to the number of
  // STDesc rough matches
  void candidate_selector(const std::vector<STDesc> &stds_vec,
                          std::vector<STDMatchList> &candidate_matcher_vec);

  // Get the best candidate frame by geometry check
  void
  candidate_verify(const STDMatchList &candidate_matcher, double &verify_score,
                   std::pair<Eigen::Vector3d, Eigen::Matrix3d> &relative_pose,
                   std::vector<std::pair<STDesc, STDesc>> &sucess_match_vec);

  // Get the transform between a matched std pair
  void triangle_solver(std::pair<STDesc, STDesc> &std_pair, Eigen::Vector3d &t,
                       Eigen::Matrix3d &rot);

  // Geometrical verification by plane-to-plane icp threshold
  double plane_geometric_verify(
      const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &source_cloud,
      const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &target_cloud,
      const std::pair<Eigen::Vector3d, Eigen::Matrix3d> &transform);
};