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


#define TIMEOUT 1000
bool bSwitch_ = false;

namespace pointcloud_merger
{
PointCloudMergerNodelet::PointCloudMergerNodelet() {}

void PointCloudMergerNodelet::onInit()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  private_nh_ = getPrivateNodeHandle();
  input_queue_size_ = 10;
  rateFreq_ = 20;  // Hz
  iSizeCloud1_ = 0;
  iSizeCloud2_ = 0;
  bSwitch_ = false;

  // Init internal pointcloud
  cloud1RW_.reset(new sensor_msgs::PointCloud2);
  cloud2RW_.reset(new sensor_msgs::PointCloud2);

  private_nh_.param<std::string>("target_frame", target_frame_, "base_link");
  private_nh_.param<double>("transform_tolerance", tolerance_, 1);
  private_nh_.param<std::string>("cloud1", cloud1_, "cloud1");
  private_nh_.param<std::string>("cloud2", cloud2_, "cloud2");
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
  message_filter1_.reset(
    new MessageFilter(sub1_, *tf2_, target_frame_, input_queue_size_, nh_));
  message_filter2_.reset(
    new MessageFilter(sub2_, *tf2_, target_frame_, input_queue_size_, nh_));

  message_filter1_->registerCallback(
    boost::bind(&PointCloudMergerNodelet::cloudCb1, this, _1));
  message_filter1_->registerFailureCallback(
    boost::bind(&PointCloudMergerNodelet::failureCb1, this, _1, _2));

  message_filter2_->registerCallback(
    boost::bind(&PointCloudMergerNodelet::cloudCb2, this, _1));
  message_filter2_->registerFailureCallback(
    boost::bind(&PointCloudMergerNodelet::failureCb2, this, _1, _2));

  pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
           cloud_out_, 10, boost::bind(&PointCloudMergerNodelet::connectCb, this),
           boost::bind(&PointCloudMergerNodelet::disconnectCb, this));
}

void PointCloudMergerNodelet::sumPointCloud()
{
  ros::Rate Rate(rateFreq_);
  sensor_msgs::PointCloud2 output;

  // for fields setup
  sensor_msgs::PointCloud2Modifier modifier(output);
  modifier.setPointCloud2FieldsByString(1, "xyz");

  bool bResize = false;
  boost::chrono::high_resolution_clock::time_point t1;
  boost::chrono::high_resolution_clock::time_point t2;
  boost::chrono::milliseconds sumGlobal;
  int iCycle = 0;

  while (pub_.getNumSubscribers() > 0)
  {
    if (cloud1RW_ != 0 && cloud2RW_ != 0)
    {
      if ((cloud1RW_->width != 0) && (cloud2RW_->width != 0))
      {
        output.header.frame_id = target_frame_;
        output.header.stamp = ros::Time::now();

        int iTempSize = cloud1RW_->width * cloud1RW_->height;

        if (iSizeCloud1_ < iTempSize )  // Keep the max
        {
          NODELET_INFO("Cloud 1 change size %d --> %d", iSizeCloud1_,
                       iTempSize);
          iSizeCloud1_ = iTempSize;
          bResize = false;
        }

        iTempSize = cloud2RW_->width *  cloud2RW_->height;

        if (iSizeCloud2_ < iTempSize)  // Keep the max
        {
          NODELET_INFO("Cloud 2 change size %d --> %d", iSizeCloud2_,
                       iTempSize);
          iSizeCloud2_ = iTempSize;
          bResize = false;
        }

        output.width = iSizeCloud1_ + iSizeCloud2_;
        output.height = 1;
        output.is_bigendian = cloud1RW_->is_bigendian;
        output.is_dense = cloud1RW_->is_dense;  // there may be invalid points

        if (bResize == false)
        {
          modifier.resize(output.width);
          bResize = true;
          NODELET_INFO("Output cloud resized");
        }

        t1 = boost::chrono::high_resolution_clock::now();

        // Lets copy every XYZ
        sensor_msgs::PointCloud2Iterator<float> out_x(output, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(output, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(output, "z");

        sensor_msgs::PointCloud2Iterator<float> c1_x(*cloud1RW_, "x");
        sensor_msgs::PointCloud2Iterator<float> c1_y(*cloud1RW_, "y");
        sensor_msgs::PointCloud2Iterator<float> c1_z(*cloud1RW_, "z");

        sensor_msgs::PointCloud2Iterator<float> c2_x(*cloud2RW_, "x");
        sensor_msgs::PointCloud2Iterator<float> c2_y(*cloud2RW_, "y");
        sensor_msgs::PointCloud2Iterator<float> c2_z(*cloud2RW_, "z");

        // Cloud1
        for (int i = 0; i < iSizeCloud1_; ++i)
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
        for (int i = 0; i < iSizeCloud2_; ++i)
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
        t2 = boost::chrono::high_resolution_clock::now();
        pub_.publish(output);
      }
    }

    Rate.sleep();
    iCycle++;
    sumGlobal += (boost::chrono::duration_cast<boost::chrono::milliseconds>(t2-t1));

      if (iCycle == 25)
      {
          iCycle = 0;
          sumGlobal /= 25;

          NODELET_INFO("Temps de traitement en milliseconde %d", (int)sumGlobal.count());
          sumGlobal -= sumGlobal;  //  reset
      }
  }
  NODELET_INFO("Leaving Publisher");
}

void PointCloudMergerNodelet::connectCb()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
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

  // Launching addition thread
  addition_thread_ =
    boost::thread(boost::bind(&PointCloudMergerNodelet::sumPointCloud, this));
}

void PointCloudMergerNodelet::disconnectCb()
{
  boost::mutex::scoped_lock lock(connect_mutex_);

  addition_thread_.join();

  if (pub_.getNumSubscribers() == 0)
  {
    NODELET_INFO("No subscribers , shutting down subscriber to pointclouds");
    sub1_.unsubscribe();
    sub2_.unsubscribe();
    cloud1_mutex_.unlock();
    cloud2_mutex_.unlock();
    bSwitch_ = true;
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    bSwitch_ = false;
  }
}

void PointCloudMergerNodelet::failureCb1(
  const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
  tf2_ros::filter_failure_reasons::FilterFailureReason reason)
{
  NODELET_WARN_STREAM_THROTTLE(
    1.0, "Can't transform pointcloud from frame "
    << cloud_msg->header.frame_id << " to "
    << message_filter1_->getTargetFramesString() << " at time "
    << cloud_msg->header.stamp << ", reason: " << reason);
}

void PointCloudMergerNodelet::failureCb2(
  const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
  tf2_ros::filter_failure_reasons::FilterFailureReason reason)
{
  NODELET_WARN_STREAM_THROTTLE(
    1.0, "Can't transform pointcloud from frame "
    << cloud_msg->header.frame_id << " to "
    << message_filter2_->getTargetFramesString() << " at time "
    << cloud_msg->header.stamp << ", reason: " << reason);
}
void PointCloudMergerNodelet::cloudCb1(
  const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    if (bSwitch_ != false ) return;
      boost::mutex::scoped_try_lock lock(cloud1_mutex_);

  if (!(target_frame_ == cloud_msg->header.frame_id))
  {
    try
    {
      tf2_->transform(*cloud_msg, *cloud1RW_, target_frame_,
                      ros::Duration(tolerance_));
      cloud1RW_->header.frame_id = target_frame_;
    }
    catch (tf2::TransformException &ex)
    {
      NODELET_ERROR_STREAM("Transform failure: " << ex.what());
    }
  }
  else
  {
    *cloud1RW_ = *cloud_msg;
  }


  bSwitch_ = true;
}

void PointCloudMergerNodelet::cloudCb2(
  const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  if (bSwitch_ != true ) return;
  boost::mutex::scoped_try_lock lock(cloud2_mutex_);

  if (!(target_frame_ == cloud_msg->header.frame_id))
  {
    try
    {
      tf2_->transform(*cloud_msg, *cloud2RW_, target_frame_,
                      ros::Duration(tolerance_));
      cloud2RW_->header.frame_id = target_frame_;
    }
    catch (tf2::TransformException &ex)
    {
      NODELET_ERROR_STREAM("Transform failure: " << ex.what());
    }
  }
  else
  {
    *cloud2RW_ = *cloud_msg;
  }
  bSwitch_ = false;
}

}  // namespace pointcloud_merger

PLUGINLIB_EXPORT_CLASS(pointcloud_merger::PointCloudMergerNodelet,
                       nodelet::Nodelet)
