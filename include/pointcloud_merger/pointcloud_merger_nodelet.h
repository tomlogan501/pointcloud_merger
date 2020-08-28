/*
 * Copyright [2020] Author: Thomas SIMON
 */

#ifndef POINTCLOUD_MERGER_POINTCLOUD_MERGER_NODELET_H
#define POINTCLOUD_MERGER_POINTCLOUD_MERGER_NODELET_H

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/exception/all.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

using namespace sensor_msgs;
using namespace message_filters;
using namespace boost;

namespace pointcloud_merger
{
typedef tf2_ros::MessageFilter<sensor_msgs::PointCloud2> MessageFilter;

class PointCloudMergerNodelet : public nodelet::Nodelet
{
public:
  PointCloudMergerNodelet();

private:
  virtual void onInit();

  void cloudCb1(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
  void cloudCb2(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

  void failureCb1(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
                  tf2_ros::filter_failure_reasons::FilterFailureReason reason);
  void failureCb2(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
                  tf2_ros::filter_failure_reasons::FilterFailureReason reason);
  void callbackSync(const sensor_msgs::PointCloud2ConstPtr &cloud_msg1,
                const sensor_msgs::PointCloud2ConstPtr &cloud_msg2);

  void connectCb();
  void disconnectCb();

  void sumPointCloud();

  ros::NodeHandle nh_, private_nh_;
  ros::Publisher pub_;

  boost::shared_ptr<tf2_ros::Buffer> tf2_;
  boost::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub1_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub2_;
  TimeSynchronizer<sensor_msgs::PointCloud2,sensor_msgs::PointCloud2>* sync_;

  // ROS Parameters
  unsigned int input_queue_size_;
  unsigned int rateFreq_;
  double tolerance_;
  std::string target_frame_;
  std::string cloud1_;
  std::string cloud2_;
  std::string cloud_out_;

  // Internal clouds
  sensor_msgs::PointCloud2Ptr cloud1RW_;
  sensor_msgs::PointCloud2Ptr cloud2RW_;
  sensor_msgs::PointCloud2 output_;
  int iSizeCloud1Max_;
  int iSizeCloud2Max_;
};

}  // namespace pointcloud_merger

#endif  // POINTCLOUD_MERGER_POINTCLOUD_MERGER_NODELET_H
