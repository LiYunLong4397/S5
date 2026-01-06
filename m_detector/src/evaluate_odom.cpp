
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
#include<tf/tf.h>


#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
ros::Publisher pubOdom;
int k=0;
float X0, Y0, Z0;
void callbackNode(const nav_msgs::OdometryConstPtr &cur_odom)
{  
    float x, y, z;
    x = cur_odom->pose.pose.position.x;
    y = cur_odom->pose.pose.position.y;
    z = cur_odom->pose.pose.position.z;
    nav_msgs::Odometry msgOdom;
    msgOdom.header.frame_id = "/velo_link";
    msgOdom.header.stamp = cur_odom->header.stamp;
    msgOdom.pose.pose.position.x = -x;
    msgOdom.pose.pose.position.y = -y;
    msgOdom.pose.pose.position.z = -z;

    
    // msgOdom.pose.pose.orientation = cur_odom->pose.pose.orientation;
    double xo, yo, zo, wo;
    xo = cur_odom->pose.pose.orientation.x;
    yo = cur_odom->pose.pose.orientation.y;
    zo = cur_odom->pose.pose.orientation.z;
    wo = cur_odom->pose.pose.orientation.w;

    tf::Quaternion qw1={zo,-xo,-yo,wo};
    tf::Matrix3x3 matrixwl, matrixwlout;
    // Eigen::Matrix3d matrixwl, matrixwlout;
    matrixwl.setRotation(qw1);
    matrixwlout[0] = matrixwl[1];
    matrixwlout[1] = matrixwl[2];
    matrixwlout[2] = matrixwl[0];
    tfScalar m_yaw,m_pitch,m_roll;
    matrixwlout.getEulerYPR(m_yaw,m_pitch,m_roll);
    /*欧拉角转换成消息的四元数，ROS消息的四元数转换成tf的四元数，然后再转换成tf的欧拉角*/
    geometry_msgs::Quaternion qw2;
    qw2=tf::createQuaternionMsgFromRollPitchYaw(m_roll,m_pitch,m_yaw);

    msgOdom.pose.pose.orientation.x = qw2.x;
    msgOdom.pose.pose.orientation.y = qw2.y;
    msgOdom.pose.pose.orientation.z = qw2.z;
    msgOdom.pose.pose.orientation.w = qw2.w;

    
    pubOdom.publish(msgOdom);
    
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_eva");
    ros::NodeHandle nh;
    pubOdom=nh.advertise<nav_msgs::Odometry>("/odometry_gt1",10);

    ros::Subscriber NodeSubscriber = nh.subscribe<nav_msgs::Odometry>("/odometry_gt", 5000, callbackNode);
    ros::spin();

    return 0;
}