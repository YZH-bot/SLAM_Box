#ifndef LIDAR_LOCALIZATION_PUBLISHER_CLOUD_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_CLOUD_PUBLISHER_HPP_

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "utils/cloud_data.hpp"

namespace Odometry {
class CloudPublisher {
  public:
    CloudPublisher(ros::NodeHandle& nh,
                   std::string topic_name,
                   size_t buff_size,
                   std::string frame_id);
    CloudPublisher() = default;
    void Publish(CloudPtr cloud_ptr_input);
  
  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;
};
} 
#endif