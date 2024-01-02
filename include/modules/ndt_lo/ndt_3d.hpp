#pragma once

#include <vector>
#include <numeric>
#include <bits/stdc++.h>
#include <unordered_map>


#include "utils/common_type.hpp"

namespace Odometry
{

    /**
     * 3D 形式的NDT
     */
    class Ndt3d
    {
    public:
        enum class NearbyType
        {
            CENTER,  // 只考虑中心
            NEARBY6, // 上下左右前后
        };

        struct Options
        {
            int max_iteration_ = 50;       // 最大迭代次数
            double voxel_size_ = 0.5;      // 体素大小
            double inv_voxel_size_ = 1.0;  //
            int min_effective_pts_ = 10;   // 最近邻点数阈值
            int min_pts_in_voxel_ = 3;     // 每个栅格中最小点数
            double eps_ = 1e-3;            // 收敛判定条件
            double res_outlier_th_ = 20.0; // 异常值拒绝阈值
            bool remove_centroid_ = false; // 是否计算两个点云中心并移除中心？

            NearbyType nearby_type_ = NearbyType::NEARBY6;
        };

        using KeyType = Eigen::Matrix<int, 3, 1>; // 体素的索引
        struct VoxelData
        {
            VoxelData() {}
            VoxelData(size_t id) { idx_.emplace_back(id); }

            std::vector<size_t> idx_;     // 点云中点的索引
            Vec3d mu_ = Vec3d::Zero();    // 均值
            Mat3d sigma_ = Mat3d::Zero(); // 协方差
            Mat3d info_ = Mat3d::Zero();  // 协方差之逆
        };

        Ndt3d()
        {
            options_.inv_voxel_size_ = 1.0 / options_.voxel_size_;
            GenerateNearbyGrids();
        }

        Ndt3d(Options options) : options_(options)
        {
            options_.inv_voxel_size_ = 1.0 / options_.voxel_size_;
            GenerateNearbyGrids();
        }

        /// 设置目标的Scan
        void SetTarget(CloudPtr target)
        {
            target_ = target;
            BuildVoxels();

            // 计算点云中心
            target_center_ = std::accumulate(target->points.begin(), target_->points.end(), Vec3d::Zero().eval(),
                                             [](const Vec3d &c, const PointType &pt) -> Vec3d
                                             { return c + ToVec3d(pt); }) /
                             target_->size();
        }

        /// 设置被配准的Scan
        void SetSource(CloudPtr source)
        {
            source_ = source;

            source_center_ = std::accumulate(source_->points.begin(), source_->points.end(), Vec3d::Zero().eval(),
                                             [](const Vec3d &c, const PointType &pt) -> Vec3d
                                             { return c + ToVec3d(pt); }) /
                             source_->size();
        }

        void SetGtPose(const SE3 &gt_pose)
        {
            gt_pose_ = gt_pose;
            gt_set_ = true;
        }

        /// 使用gauss-newton方法进行ndt配准
        bool AlignNdt(SE3 &init_pose);

    private:
        void BuildVoxels();

        /// 根据最近邻的类型，生成附近网格
        void GenerateNearbyGrids();

        CloudPtr target_ = nullptr;
        CloudPtr source_ = nullptr;

        Vec3d target_center_ = Vec3d::Zero();
        Vec3d source_center_ = Vec3d::Zero();

        SE3 gt_pose_;
        bool gt_set_ = false;

        Options options_;

        std::unordered_map<KeyType, VoxelData, hash_vec<3>> grids_; // 栅格数据
        std::vector<KeyType> nearby_grids_;                         // 附近的栅格
    };

} // namespace sad
