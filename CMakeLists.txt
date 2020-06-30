cmake_minimum_required(VERSION 3.0)
project(pointcloud_merger)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  nodelet
  roscpp
  rospy
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
  tf2_sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pointcloud_merger
  CATKIN_DEPENDS message_filters nodelet roscpp rospy sensor_msgs tf2 tf2_ros tf2_sensor_msgs pcl_conversions pcl_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(pointcloud_merger src/pointcloud_merger_nodelet.cpp)
target_link_libraries(pointcloud_merger ${catkin_LIBRARIES})

add_executable(pointcloud_merger_node src/pointcloud_merger_node.cpp)
target_link_libraries(pointcloud_merger_node pointcloud_merger ${catkin_LIBRARIES})

install(TARGETS
  pointcloud_merger
  pointcloud_merger_node
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
    find_package(roslint REQUIRED)
    roslint_cpp(src/pointcloud_merger_nodelet.cpp)
    roslint_cpp(src/pointcloud_merger_node.cpp)
    roslint_add_test()

    if (CMAKE_BUILD_TYPE STREQUAL "Debug" )
        add_compile_definitions(DEBUG)
    endif ()

    #PCL REQUIREMENT
    find_package(PCL 1.2 REQUIRED)
    find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})

    catkin_add_gtest(${PROJECT_NAME}_test
        test/test_pc_merger.cpp
        test/test_pc_merger.h
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)

    target_link_libraries(  ${PROJECT_NAME}_test
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES})
endif()
