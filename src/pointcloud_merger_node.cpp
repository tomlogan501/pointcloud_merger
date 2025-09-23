/*
 *  Copyright [2020] Author: Thomas SIMON
 */

#include "pointcloud_merger/pointcloud_merger_composable.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<pointcloud_merger::PointCloudMergerComposable>(rclcpp::NodeOptions());

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
