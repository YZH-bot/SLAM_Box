// #pragma once

#ifndef CLOUD_UTILS_H
#define CLOUD_UTILS_H

#include "utils/common_type.hpp"

struct VelodynePointXYZIRT
{
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(float, time, time))

using SelfPointType = VelodynePointXYZIRT;
using SelfPointCloudType = pcl::PointCloud<SelfPointType>;
using SelfCloudPtr = pcl::PointCloud<SelfPointType>::Ptr; // 自定义类型只能使用指针，使用PointCloud类型编译不通过

namespace Odometry
{
  class CloudData
  {
  public:
    CloudData()
    {
      cloud_ptr.reset(new SelfPointCloudType());
    }
    double time = 0.0;
    SelfCloudPtr cloud_ptr;
  };

  inline CloudPtr PointCloud2ToCloudPtr(sensor_msgs::PointCloud2::Ptr msg)
  {
    CloudPtr cloud(new PointCloudType);
    pcl::fromROSMsg(*msg, *cloud);
    return cloud;
  }

  // todo: inline可以避免函数重复定义
  inline CloudPtr ConvertToCloud(SelfCloudPtr input)
  {
    CloudPtr cloud(new PointCloudType);
    for (auto &pt : input->points)
    {
      PointType p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = pt.z;
      p.intensity = pt.intensity;
      cloud->points.template emplace_back(p);
    }
    cloud->width = cloud->size();
    cloud->height = 1;
    return cloud;
  }
}

#endif