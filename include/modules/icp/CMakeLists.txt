cmake_minimum_required(VERSION 3.0.2)
project(test_icp)

SET(CMAKE_BUILD_TYPE "Release")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
# SET(CMAKE_BUILD_TYPE "Debug")
add_compile_options(-std=c++17 -O3)
add_definitions(-std=c++17)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(/usr/local/include/eigen3)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        sensor_msgs
        )
include_directories(${catkin_INCLUDE_DIRS})

find_package(Glog)
find_package(gflags REQUIRED)
find_package(OpenMP REQUIRED)
find_package( TBB REQUIRED)
find_package(Sophus REQUIRED)
# //?：什么鬼bug https://blog.csdn.net/qq_39779233/article/details/127973918
set(Sophus_LIBRARIES libSophus.so)

find_package(PCL 1.9 REQUIRED QUIET)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  /home/robot-nuc12/catkin_ws/src/slam/SLAM_Box/include
)

# ndt_lo_node
add_executable(test_icp test_icp.cpp)
target_link_libraries(
  test_icp 
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Sophus_LIBRARIES}
  OpenMP::OpenMP_CXX
  glog::glog
  TBB::tbb
)