#include <gtest/gtest.h>
#include <iostream>
#include <string>

#include <unistd.h>
#include <stdio.h>

// ROS
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

// PCL
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include "test_pc_merger.h"

#define NB_ITER 50
#define MAX_DISTANCE 5

TEST(UnitTest, checkSumClouds) {
  // Loading pointclouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(
      new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>("data/cloud1.pcd", *cloud1) ==
      -1) //* load the file
  {
    PCL_ERROR("Couldn't read file data/cloud1.pcd \n");
    ASSERT_FALSE(true);
  }
#ifdef DEBUG
  std::cout << "Loaded " << cloud1->width * cloud1->height
            << " data points from data/cloud1.pcd with the following fields: "
            << std::endl;
#endif

  if (pcl::io::loadPCDFile<pcl::PointXYZ>("data/cloud2.pcd", *cloud2) ==
      -1) //* load the file
  {
    PCL_ERROR("Couldn't read file data/cloud2.pcd \n");
    ASSERT_FALSE(true);
  }

#ifdef DEBUG
  std::cout << "Loaded " << cloud2->width * cloud2->height
            << " data points from data/cloud2.pcd with the following fields: "
            << std::endl;
#endif

  if (pcl::io::loadPCDFile<pcl::PointXYZ>("data/cloud_out.pcd", *cloud3) ==
      -1) //* load the file
  {
    PCL_ERROR("Couldn't read file data/cloud_out.pcd \n");
    ASSERT_FALSE(true);
  }

#ifdef DEBUG
  std::cout
      << "Loaded " << cloud3->width * cloud3->height
      << " data points from data/cloud_out.pcd with the following fields: "
      << std::endl;
#endif

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud1, *cloud1, indices);
  pcl::removeNaNFromPointCloud(*cloud2, *cloud2, indices);
  pcl::removeNaNFromPointCloud(*cloud3, *cloud3, indices);

  // Help ICP by transform into the same frame
  Eigen::Affine3f transform_cloud1, transform_cloud2, transform_cloud3;
  transform_cloud3 = Eigen::Affine3f::Identity();

//  transform_cloud1 = Eigen::Affine3f::Identity();
//  transform_cloud2 = Eigen::Affine3f::Identity();
//  transform_cloud1.translation() << 0.948, 0.566, 0.065;
//  transform_cloud1.rotate(Eigen::Quaternionf(0.249, -0.420, 0.750, 0.445));
//  transform_cloud2.translation() << -0.948, 0.566, 0.065;
//  transform_cloud2.rotate(Eigen::Quaternionf(0.420, -0.249, 0.445, 0.763));

  // To match frames with cloud 1 and 2, getting close.
  transform_cloud3.rotate(
      Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitY()));
  transform_cloud3.rotate(
      Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitX()));

  pcl::transformPointCloud(*cloud3, *cloud3, transform_cloud3);
  pcl::io::savePCDFile<pcl::PointXYZ>("data/cloud_out_updated.pcd", *cloud3);

#ifdef DEBUG
  // Print the transformation
  std::cout << transform_cloud1.matrix() << std::endl;
  std::cout << transform_cloud2.matrix() << std::endl;
  std::cout << transform_cloud3.matrix() << std::endl;
#endif

  // Registration cloud1 in cloud3
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setEuclideanFitnessEpsilon(0.000000000001);
  icp.setTransformationEpsilon(0.0000001);
  icp.setMaximumIterations(2);
  icp.setMaxCorrespondenceDistance(MAX_DISTANCE);
  icp.setInputTarget(cloud3);

  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*cloud1, *cloud_copy);

  pcl::PointCloud<pcl::PointXYZ>::Ptr reg_result = cloud_copy;

#ifdef DEBUG
    PCL_INFO("Press q for cloud 1\n");
    p->spin();
#endif

  for (int i = 0; i < NB_ITER; ++i) {

#ifdef DEBUG
    PCL_INFO("Iteration Nr. %d.\n", i);
#endif

    // save cloud for visualization purpose
    cloud_copy = reg_result;

    // Estimate
    icp.setInputSource(cloud_copy);
    icp.align(*reg_result);

    // accumulate transformation between each Iteration
    Ti = icp.getFinalTransformation() * Ti;

    // if the difference between this transformation and the previous one
    // is smaller than the threshold, refine the process by reducing
    // the maximal correspondence distance
    if (std::abs((icp.getLastIncrementalTransformation() - prev).sum()) <
        icp.getTransformationEpsilon())
      icp.setMaxCorrespondenceDistance(icp.getMaxCorrespondenceDistance() *
                                       0.8);

    prev = icp.getLastIncrementalTransformation();

#ifdef DEBUG
    // visualize current state
    showCloudsRight(cloud3, cloud_copy);
    std::cout << "has converged:" << icp.hasConverged()
              << " score: " << icp.getFitnessScore() << std::endl;
#endif

    if (icp.hasConverged() == 0) {
      icp.setMaxCorrespondenceDistance(icp.getMaxCorrespondenceDistance() * 2);
    }
  }

  EXPECT_TRUE(icp.getFitnessScore() < 0.0001);

  //CLOUD2
#ifdef DEBUG
    PCL_INFO("Press q for cloud 2\n");
    p->spin();
#endif

  pcl::copyPointCloud(*cloud2, *cloud_copy);
  icp.setMaxCorrespondenceDistance(MAX_DISTANCE); // reset to MAX_DISTANCE

  reg_result = cloud_copy;

  for (int i = 0; i < NB_ITER; ++i) {

#ifdef DEBUG
    PCL_INFO("Iteration Nr. %d.\n", i);
#endif

    cloud_copy = reg_result;

    // Estimate
    icp.setInputSource(cloud_copy);
    icp.align(*reg_result);

    // accumulate transformation between each Iteration
    Ti = icp.getFinalTransformation() * Ti;

    // if the difference between this transformation and the previous one
    // is smaller than the threshold, refine the process by reducing
    // the maximal correspondence distance
    if (std::abs((icp.getLastIncrementalTransformation() - prev).sum()) <
        icp.getTransformationEpsilon())
      icp.setMaxCorrespondenceDistance(icp.getMaxCorrespondenceDistance() *
                                       0.8);

    prev = icp.getLastIncrementalTransformation();

#ifdef DEBUG
    // visualize current state
    showCloudsRight(cloud3, cloud_copy);
    std::cout << "has converged:" << icp.hasConverged()
              << " score: " << icp.getFitnessScore() << std::endl;
#endif

    if (icp.hasConverged() == 0) {
      icp.setMaxCorrespondenceDistance(icp.getMaxCorrespondenceDistance() * 2);
    }
  }

  EXPECT_TRUE(icp.getFitnessScore() < 0.0001);

#ifdef DEBUG
    PCL_INFO("Press q to quit\n");
    p->spin();
#endif
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {

#ifdef DEBUG
  // Create a PCLVisualizer object
  p = new pcl::visualization::PCLVisualizer(argc, argv, "Check ICP ");
  p->addCoordinateSystem(10.0, "global");
  p->createViewPort(0.0, 0, 0.5, 1.0, vp_1);
  p->createViewPort(0.5, 0, 1.0, 1.0, vp_2);
#endif

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
