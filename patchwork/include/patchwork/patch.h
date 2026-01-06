#ifndef PATCH_H
#define PATCH_H
#include <iostream>
#include <ros/ros.h>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <memory>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/convex_hull.h>




namespace patch_ground{
 /*   class gps_class {
    public:
    void ins_callback(const gps_ins_read::gps_dataConstPtr& imu_msg)
    ros::Subscriber ins_subscriber_;
};*/
class patch_ground{
    

  

public:
    patch_ground(ros::NodeHandle nh);
    ~patch_ground() {};
private:
    ros::Subscriber cloud_time;
    ros::Publisher cloud_no_ground , cloud_ground;


    void PCLCallback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud_in);
    void downsample(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &in_cloud_ptr,
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr &out_cloud_ptr);
    void Clip(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud_in , pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_out);
                                 
    

};
}


#endif