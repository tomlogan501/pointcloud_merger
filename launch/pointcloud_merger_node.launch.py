#!usr/bin/python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    param_file = LaunchConfiguration("params_file")
    namespace = LaunchConfiguration("namespace")
    topic_remap = LaunchConfiguration("topic_remap", default="/merged_pointcloud")

    pkg_name = "pointcloud_merger"
    config_path = "config/merge_params.yaml"

    config_file = os.path.join(get_package_share_directory(pkg_name), config_path)

    declare_config_file = DeclareLaunchArgument(
        "params_file",
        default_value=[config_file],
        description="Path to param file.",
    )

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="livox",
        description="Namespace of the merger node.",
    )

    pointcloud_merger_node = Node(
        package="pointcloud_merger",
        executable="pointcloud_merger_node",
        name="pointcloud_merger",
        namespace=namespace,
        parameters=[param_file],
        remappings=[("/merged_pointcloud", topic_remap)],
        output="screen",
    )

    return LaunchDescription([declare_config_file, declare_namespace, pointcloud_merger_node])
