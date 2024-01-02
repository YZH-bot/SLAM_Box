#include <ros/ros.h>

#include "utils/gnss_data.hpp"
#include "utils/imu_data.hpp"
#include "utils/gnss_subscriber.hpp"
#include "utils/odometry_publisher.hpp"
#include "utils/imu_subscriber.hpp"
#include <tf/transform_listener.h>

using namespace Odometry;

tf::TransformListener *listener_;
bool TransformToMatrix(const tf::StampedTransform &transform, Eigen::Matrix4f &transform_matrix)
{
    Eigen::Translation3f tl_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());

    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
    Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());

    // 此矩阵为 child_frame_id 到 base_frame_id 的转换矩阵
    transform_matrix = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

    return true;
}
bool LookupData(Eigen::Matrix4f &transform_matrix)
{
    try
    {
        tf::StampedTransform transform;
        listener_->lookupTransform("velo_link", "imu_link", ros::Time(0), transform);
        TransformToMatrix(transform, transform_matrix);
        return true;
    }
    catch (tf::TransformException &ex)
    {
        return false;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gnss_data_node");
    ros::NodeHandle nh;
    listener_ = new tf::TransformListener();
    std::shared_ptr<IMUSubscriber> imu_sub_ptr = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1);
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1);
    std::shared_ptr<OdometryPublisher> gnss_pub_ptr = std::make_shared<OdometryPublisher>(nh, "gnss", "map", "lidar", 1);

    std::deque<IMU> imu_data_buff;
    std::deque<GNSSData> gnss_data_buff;
    bool transform_received = false;
    bool gnss_origin_position_inited = false;
    Eigen::Matrix4f lidar_to_imu = Eigen::Matrix4f::Identity();

    ros::Rate rate(5);
    while (ros::ok())
    {
        ros::spinOnce();
        imu_sub_ptr->ParseData(imu_data_buff);
        gnss_sub_ptr->ParseData(gnss_data_buff);

        if (!transform_received)
        {
            if (LookupData(lidar_to_imu))
            {
                transform_received = true;
            }
        }
        else
        {
            IMU imu_data = imu_data_buff.back();
            GNSSData gnss_data = gnss_data_buff.back();
            if(imu_data_buff.size() == 0 || gnss_data_buff.size() == 0)
                continue;
            imu_data_buff.pop_front();
            gnss_data_buff.pop_front();
            gnss_data.UpdateXYZ();

            if (!gnss_origin_position_inited)
            {
                gnss_data.InitOriginPosition();
                gnss_origin_position_inited = true;
            }
            Eigen::Matrix4f odometry_matrix = Eigen::Matrix4f::Identity();
            odometry_matrix(0, 3) = gnss_data.local_E;
            odometry_matrix(1, 3) = gnss_data.local_N;
            odometry_matrix(2, 3) = gnss_data.local_U;
            std::cout << gnss_data.local_E << " " << gnss_data.local_N << " " << gnss_data.local_U << std::endl;
            odometry_matrix.block<3, 3>(0, 0) = imu_data.GetOrientationMatrix();
            odometry_matrix *= lidar_to_imu.inverse();
            gnss_pub_ptr->Publish(odometry_matrix);
        }
        rate.sleep();
    }

    return 0;
}
