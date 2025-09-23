cmake_minimum_required(VERSION 3.5)
project(pointcloud_merger)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

include_directories(include)

add_library(pointcloud_merger SHARED
  src/pointcloud_merger_composable.cpp
)

ament_target_dependencies(pointcloud_merger
  rclcpp
  sensor_msgs
  message_filters
  tf2
  tf2_ros
  tf2_sensor_msgs
  rclcpp_components
)

rclcpp_components_register_node(
    pointcloud_merger
    PLUGIN "pointcloud_merger::PointCloudMergerComposable"
    EXECUTABLE pointcloud_merger_plugin
)

ament_export_targets(export_pointcloud_merger)

add_executable(pointcloud_merger_node
  src/pointcloud_merger_node.cpp
)

ament_target_dependencies(pointcloud_merger_node
  rclcpp
  sensor_msgs
  message_filters
  tf2
  tf2_ros
  tf2_sensor_msgs
)

target_link_libraries(pointcloud_merger_node
  pointcloud_merger
)

install(TARGETS pointcloud_merger
  EXPORT export_pointcloud_merger
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS pointcloud_merger_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES pointcloud_merger.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_export_dependencies(rclcpp)

ament_package()