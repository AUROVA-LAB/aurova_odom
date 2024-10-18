
# LiDAR Odometry with Dual Quaternion Optimization

This repository contains an approach for LiDAR odometry based on edge and surface features, as well as Stable Triangle Descriptors (STD). These features are optimized using Dual Quaternion (DQ) optimization.

This method introduces a novel approach for LiDAR odometry estimation, fully parameterized with dual quaternions. The features derived from the point cloud, including edges, surfaces, and Stable Triangle Descriptors (STD), along with the optimization process, are all expressed in the dual quaternion space. This enables a direct combination of translation and orientation errors using dual quaternion operations, significantly improving pose estimation accuracy. Comparative experiments against other state-of-the-art methods show that this approach achieves enhanced performance.

An overview of our LiDAR-Only odometry method can be found in [DualQuat_LOAM](https://aurova-projects.github.io/dualquat_loam/) of AUROVA PROJECTS

## Requirements

- **ROS 1** (tested in ROS-Noetic)
- **PCL (Point Cloud Library)**
- **Eigen** for linear algebra operations
- **[Ceres Solver](http://ceres-solver.org/)** (tested with 2.2.0 library version)
- Point cloud preprocessing node ([aurova_preprocessed](https://github.com/AUROVA-LAB/aurova_preprocessed)). More specifically [pc_feature](https://github.com/AUROVA-LAB/aurova_preprocessed/tree/master/pc_features) node.

## Installation

### 1. Clone the repository

```bash
cd ~/catkin_ws/src
git clone 
```

### 2. Build the package

Navigate to your catkin workspace and build the package:

```bash
cd ~/catkin_ws
catkin_make --only-pkg-with-deps dualquat_loam
```

## Usage

### 1. Launch the pc_feature node

We have separated the point cloud preprocessing for edge and surface feature extraction, allowing the user to implement this package with their own code or use the one provided in [pc_feature](https://github.com/AUROVA-LAB/aurova_preprocessed/tree/master/pc_features) node.

```bash
roslaunch pc_feature_extraction Kitti_extraction.launch
```

### 2. Launch the odometry node

The package has been tested with the [KITTI](https://www.cvlibs.net/datasets/kitti/raw_data.php) (00 - 010 sequences), [HeliPR](https://sites.google.com/view/heliprdataset) (Roundabout02, Bridge02, Town03), [conSLAM](https://github.com/mac137/ConSLAM/tree/main) (sequence02) and [NTU-VIRAL](https://ntu-aris.github.io/ntu_viral_dataset/)(eee03) datasets.

```bash
roslaunch dualquat_loam odomEstimation_KITTI_dataset.launch
```
Make sure the parameters and topics are correctly set according to your LiDAR sensor and point cloud input.

## Acknowledgements

We would like to acknowledge the following repositories for their contributions to this project:

- [F-LOAM](https://github.com/wh200720041/floam)
- [STD Descriptors](https://github.com/hku-mars/STD)
- [DQ Robotics](https://github.com/dqrobotics)
