/*
 * Software License Agreement (BSD License)
 *
 *  Copyright [2020] ENSTA
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 *  Author: Thomas SIMON
 */

#include <limits>

#include <rclcpp_components/register_node_macro.hpp>
#include <pointcloud_merger/pointcloud_merger_composable.h>

namespace pointcloud_merger
{
  PointCloudMergerComposable::PointCloudMergerComposable(const rclcpp::NodeOptions &options)
      : Node("pointcloud_merger", options)
  {
    iSizeCloud1Max_ = 0;
    iSizeCloud2Max_ = 0;
    gResize = false;

    this->declare_parameter<std::string>("target_frame", "base_link");
    this->declare_parameter<std::string>("cloud1", "sensor1/depth/points");
    this->declare_parameter<std::string>("cloud2", "sensor2/depth/points");
    this->declare_parameter<std::string>("cloud_out", "cloud_out");
    this->declare_parameter<int>("input_queue_size", 10);
    this->declare_parameter<double>("max_interval_duration", 0.2);

    // Init internal pointcloud
    cloud1RW_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    cloud2RW_ = std::make_shared<sensor_msgs::msg::PointCloud2>();

    target_frame_ = this->get_parameter("target_frame").as_string();
    cloud1_ = this->get_parameter("cloud1").as_string();
    cloud2_ = this->get_parameter("cloud2").as_string();
    cloud_out_ = this->get_parameter("cloud_out").as_string();
    input_queue_size_ = this->get_parameter("input_queue_size").as_int();
    max_interval_duration_ = this->get_parameter("max_interval_duration").as_double();

    rclcpp::QoS qos = rclcpp::QoS(10);

    RCLCPP_INFO(this->get_logger(), "Got a subscriber to scan, starting subscriber to pointcloud 1");
    sub1_.subscribe(this, cloud1_, qos.get_rmw_qos_profile());

    RCLCPP_INFO(this->get_logger(), "Got a subscriber to scan, starting subscriber to pointcloud 2");
    sub2_.subscribe(this, cloud2_, qos.get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>>>(
        message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,
                                                        sensor_msgs::msg::PointCloud2>(input_queue_size_),
        sub1_, sub2_);

    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(max_interval_duration_));

    sync_->registerCallback(std::bind(&PointCloudMergerComposable::callbackSync, this, std::placeholders::_1, std::placeholders::_2));

    tf2_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_);

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_out_, 1);
  }

  void PointCloudMergerComposable::callbackSync(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg1,
                                                const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg2)
  {
    if (!(target_frame_ == cloud_msg1->header.frame_id))
    {
      try
      {
        auto transform1 = tf2_->lookupTransform(
            target_frame_, cloud_msg1->header.frame_id, tf2::TimePointZero);
        tf2::doTransform(*cloud_msg1, *cloud1RW_, transform1);
        cloud1RW_->header.frame_id = target_frame_;
      }
      catch (tf2::TransformException &ex)
      {
        RCLCPP_ERROR(this->get_logger(), "Transform cloud 1 failure: %s", ex.what());
      }
    }
    else
    {
      *cloud1RW_ = *cloud_msg1;
    }

    if (!(target_frame_ == cloud_msg2->header.frame_id))
    {
      try
      {
        auto transform2 = tf2_->lookupTransform(
            target_frame_, cloud_msg2->header.frame_id, tf2::TimePointZero);
        tf2::doTransform(*cloud_msg2, *cloud2RW_, transform2);
        cloud2RW_->header.frame_id = target_frame_;
      }
      catch (tf2::TransformException &ex)
      {
        RCLCPP_ERROR(this->get_logger(), "Transform cloud2 failure: %s", ex.what());
      }
    }
    else
    {
      *cloud2RW_ = *cloud_msg2;
    }

    if (cloud1RW_ != 0 && cloud2RW_ != 0)
    {
      if ((cloud1RW_->width != 0) && (cloud2RW_->width != 0))
      {
        output_.header.frame_id = target_frame_;
        output_.header.stamp = this->now();

        int iSizeCloud1 = cloud1RW_->width * cloud1RW_->height;

        if (iSizeCloud1Max_ != iSizeCloud1) // Keep the max
        {
          RCLCPP_INFO(this->get_logger(), "Cloud 1 change size %d --> %d", iSizeCloud1Max_,
                      iSizeCloud1);
          iSizeCloud1Max_ = iSizeCloud1;
          gResize = false;
        }

        int iSizeCloud2 = cloud2RW_->width * cloud2RW_->height;

        if (iSizeCloud2Max_ != iSizeCloud2) // Keep the max
        {
          RCLCPP_INFO(this->get_logger(), "Cloud 2 change size %d --> %d", iSizeCloud2Max_,
                      iSizeCloud2);
          iSizeCloud2Max_ = iSizeCloud2;
          gResize = false;
        }

        output_.width = iSizeCloud1Max_ + iSizeCloud2Max_;
        output_.height = 1;
        output_.is_bigendian = cloud1RW_->is_bigendian;
        output_.is_dense = cloud1RW_->is_dense; // there may be invalid points

        output_.fields = cloud1RW_->fields;
        output_.point_step = cloud1RW_->point_step;
        output_.row_step = output_.width * output_.point_step;

        if (gResize == false)
        {
          output_.data.resize(output_.row_step);
          gResize = true;
          RCLCPP_INFO(this->get_logger(), "Output cloud resized : %d", output_.width * output_.height);
        }

        size_t cloud1_data_size = iSizeCloud1 * cloud1RW_->point_step;
        size_t cloud2_data_size = iSizeCloud2 * cloud2RW_->point_step;

        std::memcpy(output_.data.data(),
                    cloud1RW_->data.data(),
                    cloud1_data_size);

        std::memcpy(output_.data.data() + cloud1_data_size,
                    cloud2RW_->data.data(),
                    cloud2_data_size);

        pub_->publish(output_);
      }
    }
  }

} // namespace pointcloud_merger

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_merger::PointCloudMergerComposable)
