#ifndef TEST_PC_MERGER_H
#define TEST_PC_MERGER_H

#ifdef DEBUG
#include <pcl/visualization/pcl_visualizer.h>
using pcl::visualization::PointCloudColorHandlerCustom;

pcl::visualization::PCLVisualizer *p;
int vp_1, vp_2;

////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the first viewport of the visualizer
 *
 */
void showCloudsLeft(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target,
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source) {
  p->removePointCloud("vp1_target");
  p->removePointCloud("vp1_source");

  PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(cloud_target, 0, 255, 0);
  PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(cloud_source, 255, 0, 0);
  p->addPointCloud(cloud_target, tgt_h, "vp1_target", vp_1);
  p->addPointCloud(cloud_source, src_h, "vp1_source", vp_1);

  PCL_INFO("Press q to begin the registration.\n");
  p->spin();
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the second viewport of the visualizer
 *
 */
void showCloudsRight(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source) {
  p->removePointCloud("vp2_target");
  p->removePointCloud("vp2_source");

  PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(cloud_target, 0, 255, 0);
  PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(cloud_source, 255, 0, 0);
  p->addPointCloud(cloud_target, tgt_h, "vp2_target", vp_2);
  p->addPointCloud(cloud_source, src_h, "vp2_source", vp_2);

  p->spinOnce();
}

#endif
#endif // TEST_PC_MERGER_H
