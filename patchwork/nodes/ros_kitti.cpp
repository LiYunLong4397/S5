// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE
#include <patchwork/node.h>
#include "patchwork/patchwork.hpp"
#include <visualization_msgs/Marker.h>

#include "tools/kitti_loader.hpp"
#include "tools/pcd_loader.hpp"
#include <pcl/filters/conditional_removal.h>

using namespace std;

ros::Publisher CloudPublisher;
ros::Publisher GroundPublisher;
ros::Publisher NonGroundPublisher;
ros::Publisher TPPublisher;
ros::Publisher FPPublisher;
ros::Publisher FNPublisher;
ros::Publisher PrecisionPublisher;
ros::Publisher RecallPublisher;
std_msgs::Header header;

using PointType = PointXYZILID;
boost::shared_ptr<PatchWork<PointType> > PatchworkGroundSeg;

std::string output_filename;
std::string acc_filename, pcd_savepath;
string      algorithm;
string      mode;
string      seq;
bool        save_flag;
bool        is_kitti;

float time_average = 0.0;
int i = 1;

void pub_score(std::string mode, double measure) {
    static int                 SCALE = 5;
    visualization_msgs::Marker marker;
    marker.header.frame_id                  = "map";
    marker.header.stamp                     = ros::Time();
    marker.ns                               = "my_namespace";
    marker.id                               = 0;
    marker.type                             = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action                           = visualization_msgs::Marker::ADD;
    if (mode == "p") marker.pose.position.x = 28.5;
    if (mode == "r") marker.pose.position.x = 25;
    marker.pose.position.y                  = 30;

    marker.pose.position.z    = 1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x            = SCALE;
    marker.scale.y            = SCALE;
    marker.scale.z            = SCALE;
    marker.color.a            = 1.0; // Don't forget to set the alpha!
    marker.color.r            = 0.0;
    marker.color.g            = 1.0;
    marker.color.b            = 0.0;
    //only if using a MESH_RESOURCE marker type:
    marker.text               = mode + ": " + std::to_string(measure);
    if (mode == "p") PrecisionPublisher.publish(marker);
    if (mode == "r") RecallPublisher.publish(marker);

}

template<typename T>
pcl::PointCloud<T> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
{
    pcl::PointCloud<T> cloudresult;
    pcl::fromROSMsg(cloudmsg,cloudresult);
    return cloudresult;
}

template<typename T>
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, std::string frame_id = "velodyne")//all change map
{
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.frame_id = frame_id;
    cloud_ROS.header.stamp = header.stamp;
    return cloud_ROS;
}

void callbackNode(const sensor_msgs::PointCloud2::ConstPtr &msg) {

    header.stamp = msg->header.stamp;
    ros::Time start_time = ros::Time::now();


    cout << msg->header.seq << "th node come" << endl;
    pcl::PointCloud<PointType>::Ptr cloud_in(new pcl::PointCloud<PointType>);
    *cloud_in = cloudmsg2cloud<PointType>(*msg);
    pcl::PointCloud<PointType> pc_curr;
    pcl::PointCloud<PointType> pc_ground;
    pcl::PointCloud<PointType> pc_non_ground;
    for(int i=0; i<cloud_in->points.size(); i++)
    {
        double dis=sqrt(cloud_in->points[i].x*cloud_in->points[i].x+cloud_in->points[i].y*cloud_in->points[i].y);
        if(dis<60)
          pc_curr.points.push_back(cloud_in->points[i]);
    }

    static double time_taken;

    cout << "Operating patchwork..." << endl;
    PatchworkGroundSeg->estimate_ground(pc_curr, pc_ground, pc_non_ground, time_taken);
  
    CloudPublisher.publish(cloud2msg(pc_curr));
    GroundPublisher.publish(cloud2msg(pc_ground));
    NonGroundPublisher.publish(cloud2msg(pc_non_ground));

    ros::Time end_time = ros::Time::now();
    float time_all = end_time.toSec() - start_time.toSec();
    if(i==1)
    time_average = (time_average+time_all)/1;
    else
    time_average = (time_average+time_all)/2;
    i++;
    std::cout << "This Time  #: " << time_all << std::endl;
    std::cout << "Average Time  #: " << time_average << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Benchmark");
    ros::NodeHandle nh;
    nh.param<string>("/algorithm", algorithm, "patchwork");
    nh.param<string>("/seq", seq, "00");

    PatchworkGroundSeg.reset(new PatchWork<PointType>(&nh));

    CloudPublisher  = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/cloud", 100, true);
    GroundPublisher = nh.advertise<sensor_msgs::PointCloud2>("/patchwork/ground", 100, true);
    NonGroundPublisher = nh.advertise<sensor_msgs::PointCloud2>("/patchwork/non_ground", 100, true);//care here

    PrecisionPublisher = nh.advertise<visualization_msgs::Marker>("/precision", 1, true);
    RecallPublisher    = nh.advertise<visualization_msgs::Marker>("/recall", 1, true);

    ros::Subscriber NodeSubscriber = nh.subscribe<sensor_msgs::PointCloud2>("/patchwork/cloud", 5000, callbackNode);
    ros::spin();

    return 0;
}
