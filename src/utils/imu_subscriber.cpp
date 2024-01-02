#include "utils/imu_subscriber.hpp"

#include "glog/logging.h"
#include <eigen_conversions/eigen_msg.h>

namespace Odometry {
IMUSubscriber::IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &IMUSubscriber::msg_callback, this);
}

void IMUSubscriber::msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {
    IMUPtr imu_data = std::make_shared<IMU>();
    
    imu_data->timestamp_ = imu_msg_ptr->header.stamp.toSec();
    tf::vectorMsgToEigen(imu_msg_ptr->angular_velocity, imu_data->gyro_);
    tf::vectorMsgToEigen(imu_msg_ptr->linear_acceleration, imu_data->acce_);

    new_imu_data_.push_back(imu_data);
}

void IMUSubscriber::ParseData(std::deque<IMUPtr>& imu_data_buff) {
    if (new_imu_data_.size() > 0) {
        imu_data_buff.insert(imu_data_buff.end(), new_imu_data_.begin(), new_imu_data_.end());
        new_imu_data_.clear();
    }
}
}