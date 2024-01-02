/*
 * @Description: 通过ros发布点云
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 */

#include "utils/cloud_publisher.hpp"
#include <ros/ros.h>
namespace Odometry
{
    CloudPublisher::CloudPublisher(ros::NodeHandle &nh,
                                   std::string topic_name,
                                   size_t buff_size,
                                   std::string frame_id)
        : nh_(nh), frame_id_(frame_id)
    {
        publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
    }

    void CloudPublisher::Publish(CloudPtr cloud_ptr_input)
    {
        sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output);
        cloud_ptr_output->header.stamp = ros::Time::now();
        cloud_ptr_output->header.frame_id = frame_id_;
        // ROS_INFO_STREAM("Cloud publish: " << cloud_ptr_output->data.size());
        publisher_.publish(*cloud_ptr_output);
    }

} // namespace data_output