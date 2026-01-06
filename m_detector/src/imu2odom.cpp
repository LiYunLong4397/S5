#include <ros/ros.h>
#include <math.h>
#include <iostream>
#include <cmath>
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include "nav_msgs/Odometry.h"


#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <geometry_msgs/TransformStamped.h>
#include "tf/transform_datatypes.h"


#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#define PI 3.1415926

using namespace sensor_msgs;
using namespace std;

//  --------------用于将imu和gps消息转化为odometry消息----------------2024.3.7

int k=0;//控制
//起始点GNSS接收的longitude、latitude、height
double L0,B0,H0 ;//转到初始坐标系的平移基准
Eigen::Matrix3d RN;//第一帧旋转
 

class pcl_sub
{
private:
    
public:
    ros::NodeHandle n1;
    ros::Publisher cloud_pub;
    ros::Publisher pubOdom;
   
    //构造函数
    pcl_sub():n1("~")
    {
    }
    //角度制转弧度制
    double rad(double d) 
    {
        return d * 3.1415926 / 180.0;
    }
    //获得距离差
    void GetDirectDistance(double L0, double B0,double H0, double L, double B,double H,double *x,double *y,double *z)
    {
        double N;
        double N0;
        double a=6378137;
        double e2=0.00669437999013;
        N=a/sqrt(1-e2*sin(B)*sin(B));
        N0=a/sqrt(1-e2*sin(B0)*sin(B0));
        *x=(N+H)*cos(B)*cos(L)-(N0+H0)*cos(B0)*cos(L0);
        *y=(N+H)*cos(B)*sin(L)-(N0+H0)*cos(B0)*sin(L0);
        *z=(N*(1-e2)+H)*sin(B)-(N0*(1-e2)+H0)*sin(B0);
    }

    //回调函数
    void chatterCallback(const  boost::shared_ptr<const sensor_msgs::NavSatFix>& gpsmsg,
                     const boost::shared_ptr<const sensor_msgs::Imu>& imumsg)
    {
        // ros::Time begin_time = ros::Time::now ();
        //只设置一次初始经纬高
        if(k==0)
        {
            ////起始点GNSS接收的longitude、latitude、height
            L0 = rad(gpsmsg->longitude);//纬度[度]。 正数位于赤道以北； 负面是南方。
            B0 = rad(gpsmsg->latitude);//经度[度]。 正数位于本初子午线以东； 负面是西方。
            H0 = gpsmsg->altitude;//海拔[m]。 正值高于WGS 84椭球（如果没有可用的海拔高度，则为NaN）
            //旋转矩阵的逆
            ROS_INFO("L0:%fB0:%fH0:%f",L0,B0,H0);

            RN(1,0)=-sin(B0)*cos(L0);
            RN(1,1)=-sin(B0)*sin(L0);
            RN(1,2)=cos(B0);
            RN(0,0)=-sin(L0);
            RN(0,1)=cos(L0);
            RN(0,2)=0;
            RN(2,0)=cos(B0)*cos(L0);
            RN(2,1)=cos(B0)*sin(L0);
            RN(2,2)=sin(B0);
            ROS_INFO("k:%d",k);
            k++;
        }
        if (k!=0)
        {
            //&&pc2->header.stamp.toSec()-ftime>10
            //ftime=pc2->header.stamp.toSec();
            //ROS_INFO("k:%d",k);
            // 四元数转欧拉角
            //定义一个四元数
            tf::Quaternion quat;
            //把msg形式的四元数转化为tf形式,得到quat的tf形式
            tf::quaternionMsgToTF(imumsg->orientation,quat);
            //定义存储r\p\y的容器
            double roll, pitch, yaw;
            //进行转换得到RPY欧拉角
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            ROS_INFO("roll:%f//pitch:%f//yaw:%f",roll,pitch,yaw);

            double L,B,H;
            L = rad(gpsmsg->longitude);//经度[度]。 正数位于本初子午线以东； 负面是西方。纬度[度]。
            B = rad(gpsmsg->latitude);//  正数位于赤道以北； 负面是南方。
            H = gpsmsg->altitude;//海拔[m]。 正值高于WGS 84椭球（如果没有可用的海拔高度，则为NaN）

            //计算当前时刻车辆位置与初始时刻车辆位置差值，
            double nx,ny,nz;
            GetDirectDistance(L0,B0,H0,L,B,H,&nx,&ny,&nz);//计算GPS变化量
            //ROS_INFO("x:%fy:%fz:%f",nx,ny,nz);

            //经纬高转化为东北天坐标系下坐标
            Eigen::MatrixXd mypoint(3,1);
            mypoint(0,0)=nx;
            mypoint(1,0)=ny;
            mypoint(2,0)=nz;
            mypoint=RN*mypoint;//
            float x = mypoint(0,0);
            float y = mypoint(1,0);
            float z = mypoint(2,0);
            //ROS_INFO("%f;;%f;;%f",x,y,z);
            
            nav_msgs::Odometry msgOdom;
            msgOdom.header.frame_id = "/velo_link";
            msgOdom.header.stamp = gpsmsg->header.stamp;
            msgOdom.pose.pose.position.x = x;
            msgOdom.pose.pose.position.y = y;
            msgOdom.pose.pose.position.z = z;
            //四元数由imu提供，认为该imu获得的四元数没有累积误差
            msgOdom.pose.pose.orientation = imumsg->orientation;
            pubOdom.publish(msgOdom);

            k++;       
        } //else      
        // double clustering_time = (ros::Time::now () - begin_time).toSec ();  
        // ROS_INFO ("time: %f ", clustering_time);
    }
     // 析构函数
    ~pcl_sub() {}
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_to_scan");
    pcl_sub p1;
    p1.pubOdom=p1.n1.advertise<nav_msgs::Odometry>("/odometry",10);//gps轨迹发布
    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub1(p1.n1, "/kitti/oxts/gps/fix", 1000);     
    message_filters::Subscriber<sensor_msgs::Imu> quat_sub1(p1.n1, "/kitti/oxts/imu", 1000);              // topic1 输入
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix,sensor_msgs::Imu> MySyncPolicy;
    // ROS_INFO("gouzao ::");   
    message_filters::Synchronizer<MySyncPolicy> sync1(MySyncPolicy(10), gps_sub1,quat_sub1); //queue size=10      // 同步
    sync1.registerCallback(boost::bind(&pcl_sub::chatterCallback,&p1, _1, _2));                   // 回调                         
    ros::spin();
    return 0;
}
