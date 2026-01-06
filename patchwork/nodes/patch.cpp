
//随机样本一致性算法（RANSAC）进行地面分割
#define PCL_NO_PRECOMPILE
#include <visualization_msgs/Marker.h>
#include <sstream>
#include "patchwork/patch.h"


using namespace std;
namespace patch_ground{

patch_ground::patch_ground(ros::NodeHandle nh) 
{
    cloud_time = nh.subscribe("/time_point", 1, &patch_ground::PCLCallback, this);///patchwork/non_ground /filtered_points_no_ground
    cloud_no_ground = nh.advertise<sensor_msgs::PointCloud2>("/patchwork/non_ground",1);
    cloud_ground = nh.advertise<sensor_msgs::PointCloud2>("/patchwork/ground",1);
    cout << "topic success" << endl;

}

void patch_ground::PCLCallback(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud_time)//回调
{
    // sensor_msgs::PointCloud2 cloud_out;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pr(new pcl::PointCloud<pcl::PointXYZI>);
    // *cloud_temp=*cloud_time;
    patch_ground::downsample(cloud_time,cloud_temp);
    patch_ground:: Clip(cloud_temp,pr);

    cloud_temp->header.stamp = pcl_conversions::toPCL(ros::Time::now());
    cloud_temp->header.frame_id = "velodyne";
    cloud_no_ground.publish(cloud_temp);

    pr->header.stamp = pcl_conversions::toPCL(ros::Time::now());
    pr->header.frame_id = "velodyne";
    cloud_ground.publish(pr);

    cout << "success" << endl;


}


void patch_ground::downsample(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &in_cloud_ptr, //下采样
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr &out_cloud_ptr)
{
    // //去除离群点
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    // sor.setInputCloud(in_cloud_ptr);
    // sor.setMeanK(50);
    // sor.setStddevMulThresh(1.0);
    // sor.filter(*cloud);

    //下采样
    float leafSize = 0.2;
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setInputCloud(in_cloud_ptr);
    voxel_grid_filter.setLeafSize(leafSize, leafSize, leafSize);
    voxel_grid_filter.filter(*out_cloud_ptr);
    // cout << "topic success" << endl;

}

void patch_ground:: Clip(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud_in , pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_out){

    // pcl::PointCloud<pcl::PointXYZI>::Ptr pr;

    // patch_ground::downsample(cloud_in,pr);
    


    // // 计算法线向量
    // pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
    // ne.setInputCloud(cloud_in);
    // pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
    // ne.setSearchMethod(kdtree);
    // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    // ne.setRadiusSearch(0.1);  // 设置邻域搜索半径
    // ne.compute(*cloud_normals);
    
    // 使用随机样本一致性算法（RANSAC）进行地面分割
    pcl::SACSegmentationFromNormals<pcl::PointXYZI, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05);  // 设置距离阈值
    seg.setInputCloud(cloud_in);
    // seg.setInputNormals(cloud_normals);
    seg.setOptimizeCoefficients(true);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.segment(*inliers, *coefficients);

    // Copy the points of the plane to a new cloud.
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    pcl::PointCloud<pcl::PointXYZI>::Ptr plane(new pcl::PointCloud<pcl::PointXYZI>);
    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers);
    extract.filter(*plane);

     // 生成凸包
    pcl::ConvexHull<pcl::PointXYZI> hull;
    pcl::PointCloud<pcl::PointXYZI>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZI>);
    hull.setInputCloud(plane);
    hull.setDimension(2);
    hull.reconstruct(*convexHull);

        // 冗余检查.
    // if (hull.getDimension() == 2)
    // {
        // 该类用于分割出棱柱模型内部的点集
        pcl::ExtractPolygonalPrismData<pcl::PointXYZI> prism;
        prism.setInputCloud(cloud_in);
        // 设置平面模型的点集
        prism.setInputPlanarHull(convexHull);

        // 设置高度范围
        prism.setHeightLimits(0.5, 2.0);//0.01f,0.4f
        pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);

        prism.segment(*objectIndices);

        // Get and show all points retrieved by the hull.
        extract.setIndices(objectIndices);
        extract.filter(*cloud_out);
        // pcl::visualization::CloudViewer viewerObjects("Objects on table");
        // viewerObjects.showCloud(objects);
        // pcl::io::savePCDFile("objects.pcd", *objects);
        // while (!viewerObjects.wasStopped())
        // {
        //         // Do nothing but wait.
        // }
    // }



    cout << "clip success" << endl;    
    // // 提取点云中位于地面上的区域
    // pcl::ExtractPolygonalPrismData<pcl::PointXYZI> prism;
    // prism.setInputCloud(cloud_in);
    // prism.setInputPlanarHull(inliers);
    // prism.setHeightLimits(-0.2, 0.2);  // 设置高度范围
    // pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices);
    // prism.segment(*ground_indices);

    // // 计算地面中心点
    // Eigen::Vector4f centroid;
    // pcl::compute3DCentroid(*cloud_in, *ground_indices, centroid);
    
    // // 将点云分成"patchwork"样式
    // int patch_size = 50;
    // int overlap_size = 10;
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // for (int y = 0; y < 100; y += patch_size - overlap_size)
    // {
    //     for (int x = 0; x < 100; x += patch_size - overlap_size)
    //     {
    //         pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    //         for (size_t i = 0; i < cloud->size(); ++i)
    //         {
    //             if (cloud->points[i].x >= centroid[0] - 50 + x &&
    //                 cloud->points[i].x < centroid[0] - 50 + x + patch_size &&
    //                 cloud->points[i].y >= centroid[1] - 50 + y &&
    //                 cloud->points[i].y < centroid[1] - 50 + y + patch_size &&
    //                 ground_indices->indices.end() == std::find(ground_indices->indices.begin(), ground_indices->indices.end(), static_cast<int>(i)))
    //             {
    //                 indices->indices.push_back(static_cast<int>(i));
    //             }
    //         }
    //         // 计算每个区域的平均颜色
    //         uint8_t r = 0, g = 0, b = 0;
    //         for (size_t i = 0; i < indices->indices.size(); ++i)
    //         {
    //             pcl::PointXYZRGB p_cl;
    //             pcl::copyPoint(*cloud_in, indices->indices[i], p_cl);
    //             r += p_cl.r;
    //             g += p_cl.g;
    //             b += p_cl.b;
    //         }
    //         int num_points = indices->indices.size();
    //         r = num_points > 0 ? static_cast<uint8_t>(r / num_points) : 0;
    //         g = num_points > 0 ? static_cast<uint8_t>(g / num_points) : 0;
    //         b = num_points > 0 ? static_cast<uint8_t>(b / num_points) : 0;
    //         // 将颜色分配给每个点
    //         for (size_t i = 0; i < indices->indices.size(); ++i)
    //         {
    //             pcl::PointXYZRGB p_cl;
    //             pcl::copyPoint(*cloud_in, indices->indices[i], p_cl);
    //             p_cl.r = r;
    //             p_cl.g = g;
    //             p_cl.b = b;
    //             output_cloud->push_back(p_cl);
    //         }
    //     }
    // }

}


   









}


int main(int argc, char **argv){
    ros::init(argc, argv, "patch");

    ros::NodeHandle nh("~");
    patch_ground::patch_ground pt(nh);
    // cout << "success" << endl;

    ros::spin();

    return 0;
    


}

