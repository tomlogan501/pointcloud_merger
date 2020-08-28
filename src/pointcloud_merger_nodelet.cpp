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
#include <pluginlib/class_list_macros.h>
#include <pointcloud_merger/pointcloud_merger_nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

bool gResize = false;

namespace pointcloud_merger
{
PointCloudMergerNodelet::PointCloudMergerNodelet() {}

void PointCloudMergerNodelet::onInit()
{
  private_nh_ = getPrivateNodeHandle();
  input_queue_size_ = 10;
  rateFreq_ = 30;  // Hz (max output frequency)
  iSizeCloud1Max_ = 0;
  iSizeCloud2Max_ = 0;
  gResize = false;

  // Init internal pointcloud
  cloud1RW_.reset(new sensor_msgs::PointCloud2);
  cloud2RW_.reset(new sensor_msgs::PointCloud2);

  private_nh_.param<std::string>("target_frame", target_frame_, "base_link");
  private_nh_.param<double>("transform_tolerance", tolerance_, 1);
  private_nh_.param<std::string>("cloud1", cloud1_, "sensor1/depth/points");
  private_nh_.param<std::string>("cloud2", cloud2_, "sensor2/depth/points");
  private_nh_.param<std::string>("cloud_out", cloud_out_, "cloud_out");
  int concurrency_level;
  private_nh_.param<int>("concurrency_level", concurrency_level, 1);

  // Check if explicitly single threaded, otherwise, let nodelet manager dictate
  // thread pool size
  if (concurrency_level == 1)
  {
    nh_ = getNodeHandle();
  }
  else
  {
    nh_ = getMTNodeHandle();
  }

  // Only queue one pointcloud per running thread
  if (concurrency_level > 0)
  {
    input_queue_size_ = concurrency_level;
  }
  else
  {
    input_queue_size_ = boost::thread::hardware_concurrency();
  }

  tf2_.reset(new tf2_ros::Buffer());
  tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));

  pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
           cloud_out_, 1, boost::bind(&PointCloudMergerNodelet::connectCb, this),
           boost::bind(&PointCloudMergerNodelet::disconnectCb, this));
}

void PointCloudMergerNodelet::connectCb()
{
  if (pub_.getNumSubscribers() > 0 &&
      sub1_.getSubscriber().getNumPublishers() == 0)
  {
    NODELET_INFO(
      "Got a subscriber to scan, starting subscriber to pointcloud 1");
    sub1_.subscribe(nh_, cloud1_, input_queue_size_);
  }

  if (pub_.getNumSubscribers() > 0 &&
      sub2_.getSubscriber().getNumPublishers() == 0)
  {
    NODELET_INFO(
      "Got a subscriber to scan, starting subscriber to pointcloud 2");
    sub2_.subscribe(nh_, cloud2_, input_queue_size_);
  }

  sync_ = new message_filters::TimeSynchronizer<PointCloud2, PointCloud2>(sub1_, sub2_, 2);
  sync_->registerCallback(boost::bind(&PointCloudMergerNodelet::callbackSync, this, _1, _2));
}

void PointCloudMergerNodelet::disconnectCb()
{
  if (pub_.getNumSubscribers() == 0)
  {
    NODELET_INFO("No subscribers , shutting down subscriber to pointclouds");
    sub1_.unsubscribe();
    sub2_.unsubscribe();
  }
}

void PointCloudMergerNodelet::callbackSync(const PointCloud2ConstPtr &cloud_msg1,
              const PointCloud2ConstPtr &cloud_msg2)
{
    if (!(target_frame_ == cloud_msg1->header.frame_id))
    {
      try
      {
        tf2_->transform(*cloud_msg1, *cloud1RW_, target_frame_,
                        ros::Duration(tolerance_));
        cloud1RW_->header.frame_id = target_frame_;
      }
      catch (tf2::TransformException &ex)
      {
        NODELET_ERROR_STREAM("Transform cloud 1 failure: " << ex.what());
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
        tf2_->transform(*cloud_msg2, *cloud2RW_, target_frame_,
                        ros::Duration(tolerance_));
        cloud2RW_->header.frame_id = target_frame_;
      }
      catch (tf2::TransformException &ex)
      {
        NODELET_ERROR_STREAM("Transform cloud2 failure: " << ex.what());
      }
    }
    else
    {
      *cloud2RW_ = *cloud_msg2;
    }

    // for fields setup
    sensor_msgs::PointCloud2Modifier modifier(output_);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    if (cloud1RW_ != 0 && cloud2RW_ != 0)
    {
      if ((cloud1RW_->width != 0) && (cloud2RW_->width != 0))
      {
        output_.header.frame_id = target_frame_;
        output_.header.stamp = ros::Time::now();

        int iSizeCloud1 = cloud1RW_->width * cloud1RW_->height;

        if (iSizeCloud1Max_ != iSizeCloud1 )  // Keep the max
        {
          NODELET_INFO("Cloud 1 change size %d --> %d", iSizeCloud1Max_,
                       iSizeCloud1);
          iSizeCloud1Max_ = iSizeCloud1;
          gResize = false;
        }

        int iSizeCloud2 = cloud2RW_->width *  cloud2RW_->height;

        if (iSizeCloud2Max_ != iSizeCloud2)  // Keep the max
        {
          NODELET_INFO("Cloud 2 change size %d --> %d", iSizeCloud2Max_,
                       iSizeCloud2);
          iSizeCloud2Max_ = iSizeCloud2;
          gResize = false;
        }

        output_.width = iSizeCloud1Max_ + iSizeCloud2Max_;
        output_.height = 1;
        output_.is_bigendian = cloud1RW_->is_bigendian;
        output_.is_dense = cloud1RW_->is_dense;  // there may be invalid points

        if (gResize == false)
        {
          modifier.resize(output_.width);
          gResize = true;
          NODELET_INFO("Output cloud resized : %d", output_.width * output_.height);
        }

        // Lets copy every XYZ
        sensor_msgs::PointCloud2Iterator<float> out_x(output_, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(output_, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(output_, "z");

        sensor_msgs::PointCloud2Iterator<float> c1_x(*cloud1RW_, "x");
        sensor_msgs::PointCloud2Iterator<float> c1_y(*cloud1RW_, "y");
        sensor_msgs::PointCloud2Iterator<float> c1_z(*cloud1RW_, "z");

        sensor_msgs::PointCloud2Iterator<float> c2_x(*cloud2RW_, "x");
        sensor_msgs::PointCloud2Iterator<float> c2_y(*cloud2RW_, "y");
        sensor_msgs::PointCloud2Iterator<float> c2_z(*cloud2RW_, "z");

        // Cloud1
        for (int i = 0; i < iSizeCloud1; ++i)
        {
          *out_x = *c1_x;
          *out_y = *c1_y;
          *out_z = *c1_z;
          ++out_x;
          ++out_y;
          ++out_z;
          ++c1_x;
          ++c1_y;
          ++c1_z;
        }

        // Cloud2
        for (int i = 0; i < iSizeCloud2; ++i)
        {
          *out_x = *c2_x;
          *out_y = *c2_y;
          *out_z = *c2_z;
          ++out_x;
          ++out_y;
          ++out_z;
          ++c2_x;
          ++c2_y;
          ++c2_z;
        }
        pub_.publish(output_);
      }
    }
}

}  // namespace pointcloud_merger

PLUGINLIB_EXPORT_CLASS(pointcloud_merger::PointCloudMergerNodelet,
                       nodelet::Nodelet)
