#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <sophus/so3.h>
#include <sophus/se3.h>

using Vec3i = Eigen::Vector3i;
using Vec3d = Eigen::Vector3d;
using Vec6d = Eigen::Matrix<double, 6, 1>;
using Mat3d = Eigen::Matrix3d;
using Mat6d = Eigen::Matrix<double, 6, 6>;
using Quatd = Eigen::Quaterniond;
using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = pcl::PointCloud<PointType>::Ptr;
using SO3 = Sophus::SO3;
using SE3 = Sophus::SE3;

// 常量定义
constexpr double kDEG2RAD = M_PI / 180.0;

inline Vec3d ToVec3d(const PointType &pt) { return pt.getArray3fMap().cast<double>(); }
// 矢量比较
template <int N>
struct less_vec
{
  inline bool operator()(const Eigen::Matrix<int, N, 1> &v1, const Eigen::Matrix<int, N, 1> &v2) const;
};
// 矢量哈希
template <int N>
struct hash_vec
{
  inline size_t operator()(const Eigen::Matrix<int, N, 1> &v) const;
};
// 模板实例化
template <>
inline bool less_vec<3>::operator()(const Eigen::Matrix<int, 3, 1> &v1, const Eigen::Matrix<int, 3, 1> &v2) const
{
  return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]) || (v1[0] == v2[0] && v1[1] == v2[1] && v1[2] < v2[2]);
}
template <>
inline size_t hash_vec<3>::operator()(const Eigen::Matrix<int, 3, 1> &v) const
{
  return size_t(((v[0] * 73856093) ^ (v[1] * 471943) ^ (v[2] * 83492791)) % 10000000);
}

template <typename S, int n>
inline Eigen::Matrix<int, n, 1> CastToInt(const Eigen::Matrix<S, n, 1> &value)
{
  return value.array().template round().template cast<int>();
}

template <typename S>
inline SE3 Mat4ToSE3(const Eigen::Matrix<S, 4, 4> &m)
{
  // 对R做归一化防止sophus的检查不过
  Quatd q(m.template block<3, 3>(0, 0).template cast<double>());
  q.normalize();
  return SE3(q, m.template block<3, 1>(0, 3).template cast<double>());
}

/**
 * 计算一个容器内数据的均值与矩阵形式协方差
 * @tparam C    容器类型
 * @tparam int 　数据维度
 * @tparam Getter   获取数据函数, 接收一个容器内数据类型，返回一个Eigen::Matrix<double, dim,1> 矢量类型
 */
template <typename C, int dim, typename Getter>
void ComputeMeanAndCov(const C &data, Eigen::Matrix<double, dim, 1> &mean, Eigen::Matrix<double, dim, dim> &cov,
                       Getter &&getter)
{
  using D = Eigen::Matrix<double, dim, 1>;
  using E = Eigen::Matrix<double, dim, dim>;
  size_t len = data.size();
  assert(len > 1);

  // clang-format off
    mean = std::accumulate(data.begin(), data.end(), Eigen::Matrix<double, dim, 1>::Zero().eval(),
                           [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); }) / len;
    cov = std::accumulate(data.begin(), data.end(), E::Zero().eval(),
                          [&mean, &getter](const E& sum, const auto& data) -> E {
                              D v = getter(data) - mean;
                              return sum + v * v.transpose();
                          }) / (len - 1);
  // clang-format on
}

/**
 * 计算一个容器内数据的均值与对角形式协方差
 * @tparam C    容器类型
 * @tparam D    结果类型
 * @tparam Getter   获取数据函数, 接收一个容器内数据类型，返回一个D类型
 */
template <typename C, typename D, typename Getter>
void ComputeMeanAndCovDiag(const C &data, D &mean, D &cov_diag, Getter &&getter)
{
  size_t len = data.size();
  assert(len > 1);
  // clang-format off
    mean = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                           [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); }) / len;
    cov_diag = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                               [&mean, &getter](const D& sum, const auto& data) -> D {
                                   return sum + (getter(data) - mean).cwiseAbs2().eval();
                               }) / (len - 1);
  // clang-format on
}

template <typename S>
inline Eigen::Matrix<S, 3, 1> VecFromArray(const std::vector<S> &v)
{
  return Eigen::Matrix<S, 3, 1>(v[0], v[1], v[2]);
}

template <typename S>
inline Eigen::Matrix<S, 3, 3> MatFromArray(const std::vector<S> &v)
{
  Eigen::Matrix<S, 3, 3> m;
  m << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
  return m;
}

inline void SaveMap(const std::string &map_path, CloudPtr global_map)
{
  global_map->width = global_map->size();
  global_map->height = 1;
  global_map->resize(global_map->width * global_map->height);
  pcl::io::savePCDFile(map_path, *global_map);
}

template <typename CloudType>
void SaveCloudToFile(const std::string &path, CloudType& cloud){
  cloud.height = 1;
  cloud.width = cloud.size();
  pcl::io::savePCDFile(path, cloud);
}