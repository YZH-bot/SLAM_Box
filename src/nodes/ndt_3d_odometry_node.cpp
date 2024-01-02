#include <ros/ros.h>

#include "modules/ndt_lo/ndt_3d.hpp"
#include "modules/ndt_lo/direct_ndt_lo.hpp"

#include "utils/cloud_subscriber.hpp"
#include "utils/cloud_publisher.hpp"
#include "utils/odometry_publisher.hpp"

#include <pcl/common/transforms.h>

using namespace Odometry;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ndt_3d_odometry");
    ros::NodeHandle nh("~");

    std::string cloud_topic;
    bool save_global_map;
    std::string global_map_save_path;
    nh.param<std::string>("cloud_topic", cloud_topic, "/kitti/velo/pointcloud");
    nh.param<bool>("save_global_map", save_global_map, false);
    nh.param<std::string>("global_map_save_path", global_map_save_path, "/home/robot-nuc12/catkin_ws/src/slam/simple_icp/icp_odometry/data/map.pcd");

    std::shared_ptr<CloudSubscriber> cloud_sub_ptr = std::make_shared<CloudSubscriber>(nh, cloud_topic, 100000);
    std::shared_ptr<CloudPublisher> cloud_pub_ptr = std::make_shared<CloudPublisher>(nh, "current_scan", 100, "map");
    std::shared_ptr<CloudPublisher> local_map_pub_ptr = std::make_shared<CloudPublisher>(nh, "local_map", 100, "map");
    std::shared_ptr<CloudPublisher> global_map_pub_ptr = std::make_shared<CloudPublisher>(nh, "global_map", 100, "map");
    std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr = std::make_shared<OdometryPublisher>(nh, "laser_odom", "map", "lidar", 100);

    Odometry::DirectNDTLO::Options options;
    options.ndt3d_options_.nearby_type_ = Odometry::Ndt3d::NearbyType::NEARBY6;
    Odometry::DirectNDTLO ndt_lo(options);

    std::deque<CloudData> cloud_data_buff;
    bool has_global_map_published = false;
    CloudPtr local_map_ptr(new PointCloudType());
    CloudPtr global_map_ptr(new PointCloudType());
    CloudPtr current_scan_ptr_pcl(new PointCloudType());

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();

        cloud_sub_ptr->ParseData(cloud_data_buff);
        while (cloud_data_buff.size() > 0)
        {
            CloudData cloud_data = cloud_data_buff.front();

            cloud_data_buff.pop_front();
            Eigen::Matrix4f odometry_matrix = Eigen::Matrix4f::Identity();
            SE3 pose;

            auto t1_s = std::chrono::steady_clock::now();
            // 转化为pcl的点云类型才可以使用pcl的算法
            current_scan_ptr_pcl = Odometry::ConvertToCloud(cloud_data.cloud_ptr);
            ndt_lo.AddCloud(current_scan_ptr_pcl, pose);
            auto t1_e = std::chrono::steady_clock::now();
            auto durations = std::chrono::duration_cast<std::chrono::milliseconds>(t1_e - t1_s);
            ROS_INFO_STREAM("ndt time consuming: " << durations.count() << " ms");

            auto laser_matrix = pose.matrix().cast<float>();
            laser_odom_pub_ptr->Publish(laser_matrix);

            CloudPtr scan_world(new PointCloudType);
            pcl::transformPointCloud(*current_scan_ptr_pcl, *scan_world, pose.matrix().cast<float>());
            cloud_pub_ptr->Publish(scan_world);
            local_map_pub_ptr->Publish(ndt_lo.getLocalMap());
            *global_map_ptr += *scan_world;

            // 全局地图
            pcl::VoxelGrid<PointType> cloud_filter_;
            cloud_filter_.setLeafSize(0.5, 0.5, 0.5);
            cloud_filter_.setInputCloud(global_map_ptr);
            cloud_filter_.filter(*global_map_ptr);
            global_map_pub_ptr->Publish(global_map_ptr);
        }
        rate.sleep();
    }

    std::cout << "Saving the pose..." << std::endl;
    ndt_lo.SavePose();

    if (save_global_map)
    {
        std::cout << "Saving the global map..." << std::endl;
        SaveMap(global_map_save_path, global_map_ptr);
    }
    std::cout << "Finish!" << std::endl;

    return 0;
}
