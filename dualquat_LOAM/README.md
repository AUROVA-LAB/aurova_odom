
# LiDAR Odometry with Dual Quaternion Optimization

This repository contains an approach for LiDAR odometry based on edge and surface features, as well as Stable Triangle Descriptors (STD). These features are optimized using Dual Quaternion (DQ) optimization.

This method introduces a novel approach for LiDAR odometry estimation, fully parameterized with dual quaternions. The features derived from the point cloud, including edges, surfaces, and Stable Triangle Descriptors (STD), along with the optimization process, are all expressed in the dual quaternion space. This enables a direct combination of translation and orientation errors using dual quaternion operations, significantly improving pose estimation accuracy. Comparative experiments against other state-of-the-art methods show that this approach achieves enhanced performance.

An overview of our LiDAR-Only odometry method can be found in [DualQuat_LOAM](https://aurova-projects.github.io/dualquat_loam/) of AUROVA PROJECTS

## Docker is all you need

You can also use only Docker to run DualQuat-LOAM. To do so, follow these simple steps.

### Clone the repository

```bash
git clone https://github.com/AUROVA-LAB/aurova_odom.git
```

### Build the Docker image

```bash
cd /path/to/your/directory/aurova_odom/dualquat_LOAM/
sudo docker build -t dualquat_loam .
```

### Run the container

```bash
sudo docker run --shm-size=1g \
--privileged \
--ulimit memlock=-1 \
--ulimit stack=67108864 \
--rm -it --net=host -e DISPLAY=:0 \
--user=root \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
--name dualquatloam_container \
--gpus all \
--cpuset-cpus=0-3 \
dualquat_loam
```

### Inside the container

You will need at least two terminals.

In one terminal, run the odometry estimation node:

```bash
source devel/setup.bash
roslaunch dualquat_loam odomEstimation_KITTI_display_STD.launch
```

In the second terminal, run the feature extraction:

```bash
source devel/setup.bash
roslaunch pc_feature_extraction Kitti_extraction.launch
```

Now you can test the pose estimation with our dualquat-loam using the [KITTI](https://www.cvlibs.net/datasets/kitti/raw_data.php) dataset.

### Note (outside the container):

If you need to display any graphical interface started inside the container, first run this outside the container:

```bash
xhost +local:
```

## Requirements (without Docker)

- **ROS 1** (tested in ROS-Noetic)
- **PCL (Point Cloud Library)**
- **Eigen** for linear algebra operations
- **[Ceres Solver](http://ceres-solver.org/)** (tested with 2.2.0 library version)
- **[Nanoflann library](https://github.com/jlblancoc/nanoflann)**

    ### Nanoflann install Instructions:
    ```bash
    git clone https://github.com/jlblancoc/nanoflann.git ~/your-directory/nanoflann
    cd ~/your-directory/nanoflann
    mkdir build 
    cd build
    cmake .. 
    sudo make install
    ```
        
- Point cloud preprocessing node ([aurova_preprocessed](https://github.com/AUROVA-LAB/aurova_preprocessed)). More specifically [pc_feature](https://github.com/AUROVA-LAB/aurova_preprocessed/tree/master/pc_features) node.

And also it is necessary the dependencies of the [STD]((https://github.com/hku-mars/STD)) descriptors:

```bash
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update 
sudo apt install -y libgtsam-dev libgtsam-unstable-dev
```
## Installation (without Docker)

### 1. Clone the repository

```bash
cd ~/catkin_ws/src
git clone https://github.com/AUROVA-LAB/aurova_odom.git

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
- [Nanoflann](https://github.com/jlblancoc/nanoflann)
