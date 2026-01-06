#include <ros/ros.h>
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <iostream>
#include <csignal>
#include <unistd.h>
#include <Python.h>

#include <Eigen/Core>
#include <types.h>
#include <m-detector/DynObjFilter.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>

#include <deque>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <m-detector/pcl_clip/pcl_clip.h>

//时间同步
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// #include "preprocess.h"

using namespace std;

shared_ptr<DynObjFilter> DynObjFilt(new DynObjFilter());
M3D cur_rot = Eigen::Matrix3d::Identity();
V3D cur_pos = Eigen::Vector3d::Zero();
pcl_clip::Pcl_clip Clip;

int     QUAD_LAYER_MAX  = 1;
int     occlude_windows = 3;
int     point_index = 0;
float   VER_RESOLUTION_MAX  = 0.01;
float   HOR_RESOLUTION_MAX  = 0.01;
float   angle_noise     = 0.001;
float   angle_occlude     = 0.02;
float   dyn_windows_dur = 0.5;
bool    dyn_filter_en = true, dyn_filter_dbg_en = true;
string  points_topic, odom_topic;
string  out_folder, out_folder_origin;
double  lidar_end_time = 0;
int     dataset = 0;
int     cur_frame = 0;
int     odom_i = 0;
int     point_i = 0;
bool    evaluate = false;

deque<M3D> buffer_rots;
deque<V3D> buffer_poss;
deque<double> buffer_times;
deque<boost::shared_ptr<PointCloudXYZI>> buffer_pcs;


ros::Publisher pub_pcl_dyn, pub_pcl_dyn_extend, pub_pcl_std, pub_pcl_cluster_, pub_pcl_all, pcl_cloud, pcl_boundary; 
ros::Publisher marker_pub_box_, marker_pub_tracinfo_, marker_pub_track_, cluster_vis_high;//cluster_vis_high在dynobjcluster内部发布


void AllCallback(const sensor_msgs::PointCloud2ConstPtr& msg_in, const nav_msgs::OdometryConstPtr &cur_odom)
{
    //odom

    Eigen::Quaterniond cur_q;
    geometry_msgs::Quaternion tmp_q;
    tmp_q = cur_odom -> pose.pose.orientation;
    tf::quaternionMsgToEigen(tmp_q, cur_q);
    cur_rot = cur_q.matrix();//四元数转旋转矩阵
    cur_pos << cur_odom -> pose.pose.position.x, cur_odom -> pose.pose.position.y, cur_odom -> pose.pose.position.z;//平移矩阵

    
    
    auto cur_time = cur_odom -> header.stamp.toSec();

    boost::shared_ptr<PointCloudXYZI> cloud_in(new PointCloudXYZI());
    boost::shared_ptr<PointCloudXYZI> cloud_filtered(new PointCloudXYZI());
    boost::shared_ptr<PointCloudXYZI> feats_undistort(new PointCloudXYZI());
    pcl::fromROSMsg(*msg_in, *cloud_in);
    feats_undistort = Clip.PclR(cloud_in);

    //处理
    boost::shared_ptr<PointCloudXYZI> cur_pc = feats_undistort;//point
    // auto cur_time = (msg_in->header.stamp).toSec();

    ros::Time begin_time1 = ros::Time::now ();
    DynObjFilt->filter(cur_pc, cur_rot, cur_pos, cur_time, cluster_vis_high, pub_pcl_cluster_);
    double clustering_time = (ros::Time::now () - begin_time1).toSec ();  
    ROS_INFO ("time: %f ", clustering_time);

    DynObjFilt->publish_markers(cur_time, marker_pub_box_, marker_pub_tracinfo_, marker_pub_track_, cur_rot, cur_pos, pcl_boundary);
    DynObjFilt->publish_dyn(pub_pcl_all, pub_pcl_dyn, pub_pcl_dyn_extend, pub_pcl_std, cur_time);
    cur_frame ++;

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynfilter_odom");
    ros::NodeHandle nh;
    nh.param<string>("dyn_obj/points_topic", points_topic, "");
    nh.param<string>("dyn_obj/odom_topic", odom_topic, "");
    nh.param<string>("dyn_obj/out_file", out_folder,"");
    nh.param<string>("dyn_obj/out_file_origin", out_folder_origin, "");
    nh.param<bool>("evaluate", evaluate, false);

    DynObjFilt->init(nh);
    //roi选取和降采样
    Clip.init(nh);   


    /*** ROS subscribe and publisher initialization ***/
    pub_pcl_dyn_extend = nh.advertise<sensor_msgs::PointCloud2>("/m_detector/frame_out", 10000);  //聚类后 *laserCloudDynObj_clus
    pub_pcl_dyn = nh.advertise<sensor_msgs::PointCloud2> ("/m_detector/point_out", 100000);  //聚类前 *laserCloudDynObj_world
    pub_pcl_std  = nh.advertise<sensor_msgs::PointCloud2> ("/m_detector/std_points", 100000);   //静止
    pub_pcl_all = nh.advertise<sensor_msgs::PointCloud2> ("/m_detector/all_point", 100000);

    pcl_boundary = nh.advertise<sensor_msgs::PointCloud2> ("/m_detector/boundary", 100000);

    pub_pcl_cluster_  = nh.advertise<sensor_msgs::PointCloud2> ("/m_detector/cluster_points", 100000); //追踪聚类点 

    //发布跟踪框
    marker_pub_box_ = nh.advertise<visualization_msgs::MarkerArray>("/m_detector/lidar_perception/marker_box",1);
    marker_pub_tracinfo_ = nh.advertise<visualization_msgs::MarkerArray>("/m_detector/lidar_perception/marker_info",1);
    marker_pub_track_ = nh.advertise<visualization_msgs::MarkerArray>("/m_detector/lidar_perception/marker_track",1);
    cluster_vis_high = nh.advertise<visualization_msgs::MarkerArray>("/m_detector/lidar_perception/marker_box_cluster",1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,nav_msgs::Odometry> SyncPolicy;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pcl(nh, points_topic, 200000);
    message_filters::Subscriber<nav_msgs::Odometry> sub_odom(nh, odom_topic, 200000);
    message_filters::Synchronizer<SyncPolicy> *sync; // 时间同步器
    sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), sub_pcl, sub_odom);
    sync->registerCallback(boost::bind(&AllCallback, _1, _2));            



    ros::spin();
    return 0;
}
