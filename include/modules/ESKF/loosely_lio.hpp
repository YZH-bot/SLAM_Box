#pragma once

#include <vector>
#include "utils/imu_data.hpp"
#include "utils/cloud_data.hpp"

#include "modules/ESKF/ESKF.hpp"
#include "modules/ESKF/static_imu_init.hpp"
#include "modules/ndt_lo/direct_ndt_lo.hpp"

#include <pcl_ros/transforms.h>
#include <yaml-cpp/yaml.h>

namespace Odometry
{
    class loosely_lio
    {
    public:
        struct Options
        {
            Options() {}
            bool save_motion_undistortion_pcd_ = false; // 是否保存去畸变前后的点云
        };

        // constructor
        loosely_lio(Options options);

        // init param
        bool Init(const std::string &config_yaml);

        // load yaml file
        bool LoadFromYAML(const std::string &yaml_file);

        // main function
        void ProcessMeasurements(std::vector<IMUPtr> &imu_input, CloudData &cloud_input);

        // 尝试让IMU初始化
        void TryInitIMU();

        // 利用IMU预测状态信息
        // 这段时间的预测数据会放入imu_states_里
        void Predict();

        // 对measures_中的点云去畸变
        void Undistort();

        // 执行一次配准和观测
        void Align();

        // 
        CloudData get_aligned_cloud();
        
    private:
        bool imu_need_init_ = true;         // 是否需要估计IMU初始零偏
        StaticIMUInit imu_init_;            // imu 静止初始化
        std::vector<NavStated> imu_states_; // 利用imu进行预测的状态
        ESKFD eskf_;                        // ESKF
        SE3 TIL_;                           // Lidar与IMU之间外参
        std::shared_ptr<Odometry::DirectNDTLO> ndt_lo = nullptr;
        SE3 pose_of_lo_;

        std::vector<IMUPtr> imu_measurments_;
        CloudData lidar_frame;
        CloudData world_frame;

        Options options_;
    };
}