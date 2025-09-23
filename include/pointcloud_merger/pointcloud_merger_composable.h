/*
 * Copyright [2020] Author: Thomas SIMON
 */

#ifndef POINTCLOUD_MERGER_POINTCLOUD_MERGER_COMPOSABLE_H
#define POINTCLOUD_MERGER_POINTCLOUD_MERGER_COMPOSABLE_H

#include <string>
#include <memory>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace pointcloud_merger
{
  class PointCloudMergerComposable : public rclcpp::Node
  {
  public:
    explicit PointCloudMergerComposable(const rclcpp::NodeOptions &options);

  private:
    void callbackSync(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg1,
                      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg2);

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    std::shared_ptr<tf2_ros::Buffer> tf2_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub1_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub2_;

    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>>>
        sync_;

    // ROS Parameters
    unsigned int input_queue_size_;
    double max_interval_duration_;
    bool gResize = false;
    std::string target_frame_;
    std::string cloud1_;
    std::string cloud2_;
    std::string cloud_out_;

    // Internal clouds
    sensor_msgs::msg::PointCloud2::SharedPtr cloud1RW_;
    sensor_msgs::msg::PointCloud2::SharedPtr cloud2RW_;
    sensor_msgs::msg::PointCloud2 output_;
    int iSizeCloud1Max_;
    int iSizeCloud2Max_;
  };

} // namespace pointcloud_merger

#endif // POINTCLOUD_MERGER_POINTCLOUD_MERGER_COMPOSABLE_H
