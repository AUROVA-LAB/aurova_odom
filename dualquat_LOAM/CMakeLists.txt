cmake_minimum_required(VERSION 3.0.2)
project(dualquat_loam)

## Compile as C++14
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 5)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
    message("core for MP:  4")
  elseif(N GREATER 3)
    math(EXPR PROC_NUM "${N} - 2")
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM="${PROC_NUM}")
    message("core for MP:  ${PROC_NUM}")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

# Find nanoflannConfig.cmake:
find_package(nanoflann)
include_directories("/home/ws/src/nanoflann/include")

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  nav_msgs
  rosbag
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  pcl_conversions
  pcl_ros
  tf_conversions
  
)

include_directories( "/usr/include/dqrobotics" )

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

## System dependencies are found with CMake's conventions
find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED QUIET)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)


include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
  include
  ${PCL_LIBRARY_DIRS}
  ${CERES_LIBRARY_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL Ceres 
  INCLUDE_DIRS include
)

add_executable(dq_pose_estimation_node src/odomEstimationNode.cpp src/odomEstimationClass.cpp src/STDesc.cpp src/KDTree_STD.cpp)
target_link_libraries(dq_pose_estimation_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} dqrobotics)

add_executable(laser_mapping src/laserMappingNode.cpp src/laserMappingClass.cpp)
target_link_libraries(laser_mapping ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} dqrobotics)

add_executable(std_extractor src/std_extractor.cpp src/STDesc.cpp src/KDTree_STD.cpp)
        target_link_libraries(std_extractor
                ${catkin_LIBRARIES}
                ${PCL_LIBRARIES}
                ${OpenCV_LIBS}
                ${CERES_LIBRARIES})

