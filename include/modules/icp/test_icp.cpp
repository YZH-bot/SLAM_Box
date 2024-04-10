#include <glog/logging.h>
#include <gflags/gflags.h>

#include "icp.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

DEFINE_string(source, "/home/robot-nuc12/catkin_ws/src/slam/SLAM_Box/include/modules/icp/data/kneeling_lady_source.pcd", "source 点云path");
DEFINE_string(target, "/home/robot-nuc12/catkin_ws/src/slam/SLAM_Box/include/modules/icp/data/kneeling_lady_target.pcd", "target 点云path");
DEFINE_string(ground_truth_file, "./data/ch7/EPFL/kneeling_lady_pose.txt", "真值Pose");

// doc: 模板偏特化
template void SaveCloudToFile<ICP::CloudType>(const std::string &path, ICP::CloudType &cloud);

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    ICP::CloudPtr source(new ICP::CloudType), target(new ICP::CloudType);
    pcl::io::loadPCDFile(fLS::FLAGS_source, *source);
    pcl::io::loadPCDFile(fLS::FLAGS_target, *target);

    ICP icp;
    icp.SetTarget(target);
    icp.SetSource(source);
    SE3 pose;
    auto t_s = std::chrono::high_resolution_clock::now();
    bool success = icp.AglinP2P(pose);
    auto t_e = std::chrono::high_resolution_clock::now();
    auto t_c = std::chrono::duration_cast<std::chrono::milliseconds>((t_e - t_s)).count();
    std::cout << std::setw(28) << std::left << "\t icp_p2p time: " << t_c << " ms\n";
    if (success)
    {
        LOG(INFO) << "icp p2plane align success, pose: " << pose.so3().unit_quaternion().coeffs().transpose()
                  << ", " << pose.translation().transpose();
        ICP::CloudPtr source_trans(new ICP::CloudType);
        pcl::transformPointCloud(*source, *source_trans, pose.matrix().cast<float>());
        SaveCloudToFile("../data/icp_p2p_trans.pcd", *source_trans);
    }
    else
    {
        LOG(ERROR) << "align failed.";
    }
    return 0;
}
