#pragma once

#include "utils/common_type.hpp"

namespace Odometry {
struct IMU{
  IMU() = default;
  IMU(const double t, const Vec3d& gyro, const Vec3d& acce): timestamp_(t), gyro_(gyro), acce_(acce){};

  double timestamp_ = 0.0;
  Vec3d gyro_ = Vec3d::Zero();
  Vec3d acce_ = Vec3d::Zero();
};
}

using IMUPtr = std::shared_ptr<Odometry::IMU>;
