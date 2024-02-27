#include <vector>
#include <glog/logging.h>

#include "utils/cloud_subscriber.hpp"
#include "utils/imu_subscriber.hpp"
#include "utils/cloud_publisher.hpp"

#include "modules/ESKF/loosely_lio.hpp"

int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = "/home/robot-nuc12/catkin_ws/src/slam/simple_icp/icp_odometry/log";
    FLAGS_alsologtostderr = true;

    ros::init(argc, argv, "loosely_lio_node");
    ros::NodeHandle nh("~");

    std::string imu_topic;
    std::string cloud_topic;
    bool save_global_map;
    std::string global_map_save_path;
    nh.param<std::string>("imu_topic", imu_topic, "/imu_correct");
    nh.param<std::string>("cloud_topic", cloud_topic, "/velodyne_points_0");
    nh.param<bool>("save_global_map", save_global_map, false);
    nh.param<std::string>("global_map_save_path", global_map_save_path, "/home/robot-nuc12/catkin_ws/src/slam/simple_icp/icp_odometry/data/ulhk/map.pcd");

    std::shared_ptr<Odometry::CloudSubscriber> cloud_sub_ptr = std::make_shared<Odometry::CloudSubscriber>(nh, cloud_topic, 100000);
    std::shared_ptr<Odometry::IMUSubscriber> imu_sub_ptr = std::make_shared<Odometry::IMUSubscriber>(nh, imu_topic, 1000000);
    // todo: gnss subscriber

    std::shared_ptr<Odometry::CloudPublisher> cloud_pub_ptr = std::make_shared<Odometry::CloudPublisher>(nh, "current_scan", 100, "map");
    std::shared_ptr<Odometry::CloudPublisher> global_map_pub_ptr = std::make_shared<Odometry::CloudPublisher>(nh, "global_map", 100, "map");

    std::deque<Odometry::CloudData> cloud_data_buffer;
    std::deque<IMUPtr> imu_data_buffer;
    Eigen::Matrix4d lidar_to_imu = Eigen::Matrix4d::Identity();
    // todo: pose initialization(using gnss)
    bool init_lio = false;
    std::vector<IMUPtr> imu_measurements;
    CloudPtr global_map_ptr(new PointCloudType());
    CloudPtr current_scan_ptr_pcl(new PointCloudType());

    // 初始化 lio
    Odometry::loosely_lio::Options options;
    Odometry::loosely_lio lio(options);
    lio.Init("/home/robot-nuc12/catkin_ws/src/slam/simple_icp/slam_box/config/velodyne_ulhk.yaml");

    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();

        cloud_sub_ptr->ParseData(cloud_data_buffer);
        imu_sub_ptr->ParseData(imu_data_buffer);

        if (cloud_data_buffer.empty() || imu_data_buffer.empty())
        {
            LOG(INFO) << "Imu msg coming~: " << imu_data_buffer.size();
            LOG(INFO) << "Lidar msg coming~: " << cloud_data_buffer.size();
            // LOG(INFO) << "No data coming~";
            continue;
        }

        // initialize (对齐imu和lidar的初始数据)
        if (!init_lio)
        {
            for (auto imu : imu_data_buffer)
            {
                if (imu->timestamp_ < cloud_data_buffer.front().time)
                {
                    imu_data_buffer.pop_front();
                }
            }
            init_lio = true;
            LOG(INFO) << "initialized~";
        }

        // parse date and process
        Odometry::CloudData lidar_cloud = cloud_data_buffer.front();
        cloud_data_buffer.pop_front();
        double lidar_begin_time = lidar_cloud.time;
        double lidar_end_time = lidar_begin_time + lidar_cloud.cloud_ptr->points.back().time;

        // get imu data between one lidar frame
        double imu_time = imu_data_buffer.front()->timestamp_;
        imu_measurements.clear();
        while (!(imu_data_buffer.empty()) && (imu_time < lidar_end_time))
        {
            imu_time = imu_data_buffer.front()->timestamp_;
            if (imu_time > lidar_end_time)
            {
                break;
            }
            imu_measurements.push_back(imu_data_buffer.front());
            imu_data_buffer.pop_front();
        }

        // processing imu and lidar measurements
        LOG(INFO) << "processing imu and lidar measurements: " << imu_measurements.size() << ", lidar pts: " << lidar_cloud.cloud_ptr->size();
        lio.ProcessMeasurements(imu_measurements, lidar_cloud);
        auto scan_world = lio.get_aligned_cloud();

        // publisher
        current_scan_ptr_pcl = Odometry::ConvertToCloud(scan_world.cloud_ptr);
        cloud_pub_ptr->Publish(current_scan_ptr_pcl);
        *global_map_ptr += *current_scan_ptr_pcl;

        // 全局地图
        pcl::VoxelGrid<PointType> cloud_filter_;
        cloud_filter_.setLeafSize(0.5, 0.5, 0.5);
        cloud_filter_.setInputCloud(global_map_ptr);
        cloud_filter_.filter(*global_map_ptr);
        global_map_pub_ptr->Publish(global_map_ptr);

        rate.sleep();
    }

    if (save_global_map)
    {
        std::cout << "Saving the global map..." << std::endl;
        SaveMap(global_map_save_path, global_map_ptr);
    }
    std::cout << "Finish!" << std::endl;

    return 0;
}