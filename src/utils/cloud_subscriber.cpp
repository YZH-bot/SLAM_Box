#include "utils/cloud_subscriber.hpp"

namespace Odometry {
CloudSubscriber::CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &CloudSubscriber::msg_callback, this);
}

void CloudSubscriber::msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr) {
    CloudData cloud_data;
    cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
    sensor_msgs::PointCloud2 cloud_msg = *cloud_msg_ptr;
    pcl::moveFromROSMsg(cloud_msg, *(cloud_data.cloud_ptr));
    // ROS_INFO_STREAM("Cloud Data size: " << cloud_data.cloud_ptr->size());
    new_cloud_data_.push_back(cloud_data);
}

void CloudSubscriber::ParseData(std::deque<CloudData>& cloud_data_buff) {
    if (new_cloud_data_.size() > 0) {
        cloud_data_buff.insert(cloud_data_buff.end(), new_cloud_data_.begin(), new_cloud_data_.end());
        new_cloud_data_.clear();
    }
}
} // namespace data_input