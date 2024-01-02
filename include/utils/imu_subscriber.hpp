#pragma once

#include <deque>
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "utils/imu_data.hpp"

namespace Odometry {
class IMUSubscriber {
  public:
    IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    IMUSubscriber() = default;
    void ParseData(std::deque<IMUPtr>& deque_imu_data);

  private:
    void msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<IMUPtr> new_imu_data_; 
};
}
