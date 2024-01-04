cmake_minimum_required(VERSION 3.0.2)
project(slam_box)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++17)
add_definitions(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  tf
  eigen_conversions
)

find_package( TBB REQUIRED)
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(/usr/local/include/eigen3)
# //?：什么鬼bug https://blog.csdn.net/qq_39779233/article/details/127973918
set(Sophus_LIBRARIES libSophus.so)

# GNSS
find_package (GeographicLib REQUIRED)
include_directories(${GeographicLib_INCLUDE_DIRS})

find_package(Glog)

find_package(PCL 1.9 REQUIRED QUIET)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# yaml-cpp
find_package(yaml-cpp REQUIRED)
include_directories(${yaml-cpp_INCLUDE_DIRS})

catkin_package(
)

include(cmake/geographic.cmake)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 新的包含写法
add_library(${PROJECT_NAME}.utils
          src/utils/cloud_publisher.cpp
          src/utils/cloud_subscriber.cpp
          src/utils/gnss_data.cpp
          src/utils/gnss_subscriber.cpp
          src/utils/imu_subscriber.cpp
          src/utils/odometry_publisher.cpp
)
target_link_libraries(${PROJECT_NAME}.utils ${PCL_LIBRARIES})

add_library(${PROJECT_NAME}.ndt_lo
          src/modules/ndt_lo/direct_ndt_lo.cpp
          src/modules/ndt_lo/ndt_3d.cpp
)
file(GLOB_RECURSE ALL_MODULES "*src/modules/*.cpp")

# ndt_lo_node
add_executable(ndt_3d_odometry_node src/nodes/ndt_3d_odometry_node.cpp 
# ${ALL_SRCS}
${ALL_MODULES}
)
target_link_libraries(ndt_3d_odometry_node 
              ${PROJECT_NAME}.utils
              ${catkin_LIBRARIES}
              ${Sophus_LIBRARIES}
              ${GeographicLib_LIBRARIES}
              glog::glog
              TBB::tbb
              yaml-cpp
              )

# loosely_lio_node
add_executable(loosely_lio_node 
              src/nodes/loosely_lio_node.cpp 
              src/modules/ESKF/loosely_lio.cpp
              src/modules/ESKF/static_imu_init.cpp
)
target_link_libraries(loosely_lio_node 
              ${PROJECT_NAME}.utils
              ${PROJECT_NAME}.ndt_lo
              ${catkin_LIBRARIES}
              ${Sophus_LIBRARIES}
              ${GeographicLib_LIBRARIES}
              glog::glog
              TBB::tbb
              ${yaml-cpp_LIBRARIES}
              yaml-cpp
              )

# add_executable(gnss_data_node src/nodes/gnss_data_node.cpp 
#               src/utils/gnss_data.cpp
#               src/utils/gnss_subscriber.cpp
#               src/utils/odometry_publisher.cpp
#               src/utils/imu_subscriber.cpp
# )
# target_link_libraries(gnss_data_node 
#             ${catkin_LIBRARIES}
#             ${GeographicLib_LIBRARIES}
#             glog::glog
#             )