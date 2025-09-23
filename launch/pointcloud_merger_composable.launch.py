#!usr/bin/python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    param_file = LaunchConfiguration("params_file")

    pkg_name = "pointcloud_merger"
    config_path = "config/merge_params.yaml"

    config_file = os.path.join(get_package_share_directory(pkg_name), config_path)

    declare_config_file = DeclareLaunchArgument(
        "params_file",
        default_value=[config_file],
        description="Path to param file.",
    )

    container = ComposableNodeContainer(
        name="pointcloud_merger_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="pointcloud_merger",
                plugin="pointcloud_merger::PointCloudMergerComposable",
                name="pointcloud_merger",
                parameters=[param_file],
            ),
        ],
        output="screen",
    )

    return LaunchDescription([declare_config_file, container])
