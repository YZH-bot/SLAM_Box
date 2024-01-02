#include "modules/ESKF/loosely_lio.hpp"

namespace Odometry
{
    // 构造函数
    loosely_lio::loosely_lio(Options options) : options_(options)
    {
        StaticIMUInit::Options imu_init_options;
        imu_init_options.use_speed_for_static_checking_ = false; // todo: withou odom
        imu_init_ = StaticIMUInit(imu_init_options);
    }

    // init loosely lio
    bool loosely_lio::Init(const std::string &config_yaml)
    {
        if (!LoadFromYAML(config_yaml))
        {
            return false;
        }

        // 初始化 lidar 匹配器的参数
        Odometry::DirectNDTLO::Options ndt_options;
        ndt_lo = std::make_shared<Odometry::DirectNDTLO>(ndt_options);

        return true;
    }

    // load yaml file
    bool loosely_lio::LoadFromYAML(const std::string &yaml_file)
    {
        auto yaml = YAML::LoadFile(yaml_file);
        std::vector<double> ext_t = yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
        std::vector<double> ext_r = yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();

        Vec3d lidar_T_wrt_IMU = VecFromArray(ext_t);
        Mat3d lidar_R_wrt_IMU = MatFromArray(ext_r);
        TIL_ = SE3(lidar_R_wrt_IMU, lidar_T_wrt_IMU);
        return true;
    }

    // main function
    void loosely_lio::ProcessMeasurements(std::vector<IMUPtr> &imu_input, CloudData &cloud_input)
    {
        LOG(INFO) << "Loosely LIO's ProcessMeasurements, imu: " << imu_input.size() << ", lidar pts: " << cloud_input.cloud_ptr->size();
        imu_measurments_ = imu_input;
        lidar_frame = cloud_input;

        if (imu_need_init_)
        {
            // 初始化IMU系统
            TryInitIMU();
            return;
        }

        // use imu data for predict motion
        Predict();

        // 配准
        Align();
    }

    void loosely_lio::TryInitIMU()
    {
        for (auto imu : imu_measurments_)
        {
            imu_init_.AddIMU(*imu);
        }

        if (imu_init_.InitSuccess())
        {
            // 读取初始零偏，设置ESKF
            ESKFD::Options options;
            // 噪声由初始化器估计
            options.gyro_var_ = sqrt(imu_init_.GetCovGyro()[0]);
            options.acce_var_ = sqrt(imu_init_.GetCovAcce()[0]);
            eskf_.SetInitialConditions(options, imu_init_.GetInitBg(), imu_init_.GetInitBa(), imu_init_.GetGravity());
            imu_need_init_ = false;

            LOG(INFO) << "IMU初始化成功";
        }
    }

    void loosely_lio::Predict()
    {
        imu_states_.clear();
        imu_states_.emplace_back(eskf_.GetNominalState());
        // 对imu状态进行预测
        for (auto imu : imu_measurments_)
        {
            eskf_.Predict(*imu);
            imu_states_.emplace_back(eskf_.GetNominalState());
        }
    }

    void loosely_lio::Align()
    {
        SelfCloudPtr scan_undistort_trans(new SelfPointCloudType);
        pcl::transformPointCloud(*lidar_frame.cloud_ptr, *scan_undistort_trans, TIL_.matrix());

        auto curren_scan_pcl = ConvertToCloud(scan_undistort_trans);
        // 从EKF中获取预测pose，放入LO，获取LO位姿，最后合入EKF
        SE3 pose_predict = eskf_.GetNominalSE3();
        ndt_lo->AddCloud(curren_scan_pcl, pose_predict);
        pose_of_lo_ = pose_predict;
        eskf_.ObserveSE3(pose_of_lo_, 1e-2, 1e-2);

        pcl::transformPointCloud(*scan_undistort_trans, *world_frame.cloud_ptr, eskf_.GetNominalSE3().matrix());
    }

    CloudData loosely_lio::get_aligned_cloud()
    {
        return world_frame;
    }

}
