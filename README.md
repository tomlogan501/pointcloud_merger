# Pointcloud merger
Package to combine 2 PointCloud2 in one with a target frame.

![Docker Image CI](https://github.com/tomlogan501/pointcloud_merger/workflows/Docker%20Image%20CI/badge.svg)

![rviz screenshot](img/rviz.png)

# Parameters
* ```target_frame``` Frame to merge the 2 clouds. Default ```base_link```
* ```cloud_out``` output topic name. Default ```cloud_out```
* ```cloud1``` input topic name. Default ```cloud1```
* ```cloud2``` input topic name. Default ```cloud2```
* ```input_queue_size``` Qos settings. Default ```10```
* ```max_interval_duration``` Max interval (s) between 2 pointclouds. Default ```0.2```

# Features
* Composable node support

# Example minimal launch
Node:

```ros2 launch pointcloud_merger pointcloud_merger_node.launch.py```

Composable:

```ros2 launch pointcloud_merger pointcloud_merger_composable.launch.py```
