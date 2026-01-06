#ifndef OBSTACLE_DETECTION_IFM_PCL_CLIP_H
#define OBSTACLE_DETECTION_IFM_PCL_CLIP_H

#include <ros/ros.h>
#include <types.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>

namespace pcl_clip
{
class Pcl_clip
{
public:
  Pcl_clip(){};
  ~Pcl_clip(){};
  void init(ros::NodeHandle node_handle);
  PointCloudXYZI::Ptr GetPcl(const PointCloudXYZI::ConstPtr cloud_in);
  PointCloudXYZI::Ptr Clip_hull(const PointCloudXYZI::ConstPtr in);
  PointCloudXYZI::Ptr PclR(const PointCloudXYZI::ConstPtr cloud_in);
  // void en_cluster(const pcl::PointCloud<PointType>::ConstPtr points_in, std::vector<pcl::PointIndices> &cluster_indices);
  
private:
  double roi_x_min_;
  double roi_x_max_;
  double roi_y_min_;
  double roi_y_max_;
  double roi_z_min_;
  double roi_z_max_;

  double hroi_1_x_;
  double hroi_1_y_;
  double hroi_2_x_;
  double hroi_2_y_;
  double hroi_3_x_;
  double hroi_3_y_;
  double hroi_4_x_;
  double hroi_4_y_;
  float leftSize;
  float radius;
  bool down_flag;

  double Tolerance_;
  int mincluster_;
  int maxcluster_;
 


};
}//namespace pcl_clip
#endif