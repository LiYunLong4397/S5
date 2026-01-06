#include <m-detector/pcl_clip/pcl_clip.h>
#include <m-detector/pcl_clip/convex_hullxy.h>

namespace pcl_clip
{

void Pcl_clip::init(ros::NodeHandle node_handle)
{

    node_handle.param<double>("roi_x_min", roi_x_min_, -20.0);
    node_handle.param<double>("roi_x_max", roi_x_max_, 20.0);
    node_handle.param<double>("roi_y_min", roi_y_min_, -20.0);
    node_handle.param<double>("roi_y_max", roi_y_max_, 20.0);
    node_handle.param<double>("roi_z_min", roi_z_min_, -1.0);
    node_handle.param<double>("roi_z_max", roi_z_max_, 10.0);
    node_handle.param<float>("leftSize", leftSize, 0.8);
    node_handle.param<float>("radius", radius, 10.0);
    node_handle.param<float>("radius", radius, 10.0);
    node_handle.param<bool>("down_flag", down_flag, false);

    node_handle.param<double>("Tolerance", Tolerance_, 0.8);
    node_handle.param<int>("mincluster", mincluster_, 20);
    node_handle.param<int>("maxcluster", maxcluster_, 300);
}

PointCloudXYZI::Ptr Pcl_clip::GetPcl(const PointCloudXYZI::ConstPtr cloud_in){
    printf("leftSize:%f",leftSize);
    //条件滤波
    boost::shared_ptr<PointCloudXYZI> cloud_filtered(new PointCloudXYZI());
    boost::shared_ptr<PointCloudXYZI> out(new PointCloudXYZI());
    pcl::ConditionAnd<pcl::PointXYZINormal>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZINormal>());
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZINormal>("x", pcl::ComparisonOps::GT, roi_x_min_)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZINormal>("x", pcl::ComparisonOps::LT, roi_x_max_)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZINormal>("y", pcl::ComparisonOps::GT, roi_y_min_)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZINormal>("y", pcl::ComparisonOps::LT, roi_y_max_)));
    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZINormal> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(cloud_in);
    condrem.setKeepOrganized(false);
    // apply filter
    condrem.filter(*cloud_filtered);
    //降采样
    
	pcl::VoxelGrid<pcl::PointXYZINormal> sor;
	sor.setInputCloud (cloud_filtered);
	sor.setLeafSize (leftSize, leftSize, leftSize);
	sor.filter(*out);
    return out;
}

PointCloudXYZI::Ptr Pcl_clip::PclR(const PointCloudXYZI::ConstPtr cloud_in){

    boost::shared_ptr<PointCloudXYZI> cloud_filtered(new PointCloudXYZI());
    boost::shared_ptr<PointCloudXYZI> out(new PointCloudXYZI());
    for(int i=0; i<cloud_in->points.size(); i++)
    {
        double dis=sqrt(cloud_in->points[i].x*cloud_in->points[i].x+cloud_in->points[i].y*cloud_in->points[i].y);
        if(dis<radius)
          cloud_filtered->points.push_back(cloud_in->points[i]);
    }
    //降采样
    if(down_flag)
    {
        pcl::VoxelGrid<pcl::PointXYZINormal> sor;
        sor.setInputCloud (cloud_filtered);
        sor.setLeafSize (leftSize, leftSize, leftSize);
        sor.filter(*out);

        return out;
    }
    else
        return cloud_filtered;

}

// void Pcl_clip::en_cluster(const pcl::PointCloud<PointType>::ConstPtr points_in, std::vector<pcl::PointIndices> &cluster_indices)
// {
//   //设置查找方式－kdtree
//   pcl::search::Search<PointType>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointType> > (new pcl::search::KdTree<PointType>);
//   pcl::EuclideanClusterExtraction<PointType> ec;
//   ec.setClusterTolerance (Tolerance_);// 2cm
//   ec.setMinClusterSize (mincluster_); //100
//   ec.setMaxClusterSize (maxcluster_);
//   printf("maxClusterSize: %d", maxcluster_);
//   ec.setSearchMethod (tree);
//   ec.setInputCloud (points_in);
//   ec.extract (cluster_indices);
// }

// int cp = 0;
//   for (const auto &cluster_index : indices)
//   {
//     // Iterate over each idx
//     for (const auto &index : cluster_index.indices)
//     {
//       // Iterate over each dimension
//       cloud_out.points[cp] = cloud_in.points[index];
//       cp++;
//     }
//   }




PointCloudXYZI::Ptr Pcl_clip::Clip_hull(const PointCloudXYZI::ConstPtr in){

    PointCloudXYZI::Ptr out(new PointCloudXYZI);

    	/*设置滤波的边框点*/
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr boundingbox_ptr (new pcl::PointCloud<pcl::PointXYZINormal>);
	boundingbox_ptr->resize(4);
    boundingbox_ptr->points[0].x = hroi_1_x_;
    boundingbox_ptr->points[0].y = hroi_1_y_;
    boundingbox_ptr->points[0].z = static_cast<double>(0);

    boundingbox_ptr->points[1].x = hroi_2_x_;
    boundingbox_ptr->points[1].y = hroi_2_y_;
    boundingbox_ptr->points[1].z = static_cast<double>(0);

    boundingbox_ptr->points[2].x = hroi_3_x_;
    boundingbox_ptr->points[2].y = hroi_3_y_;
    boundingbox_ptr->points[2].z = static_cast<double>(0);

    boundingbox_ptr->points[3].x = hroi_4_x_;
    boundingbox_ptr->points[3].y = hroi_4_y_;
    boundingbox_ptr->points[3].z = static_cast<double>(0);
	


	/*求上面给出的这个边框点集的凸包*/
	//pcl::ConvexHull<pcl::PointXYZINormal> hull;
    ConvexHull2DXY<pcl::PointXYZINormal> hull;
	hull.setInputCloud(boundingbox_ptr);
	hull.setDimension(2); /*设置凸包维度*/
	std::vector<pcl::Vertices> poly_vt;; /*用于保存凸包顶点*/
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr surface_hull (new pcl::PointCloud<pcl::PointXYZINormal>); /*用于描绘凸包形状*/


	hull.Reconstruct2dxy(surface_hull, &poly_vt);

	pcl::PointCloud<pcl::PointXYZINormal>::Ptr objects (new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::CropHull<pcl::PointXYZINormal> bb_filter;
	bb_filter.setDim(2); /*设置维度*/
	bb_filter.setInputCloud(in);
	bb_filter.setHullIndices(poly_vt); /*封闭多边形顶点*/
	bb_filter.setHullCloud(surface_hull); /*封闭多边形形状*/
	bb_filter.filter(*out); /*结果保存到objects*/
	
    return out;
}


}//namespace remove_ground
