#include <ros/ros.h>
#include <uav/UWB.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    
}

void uwbCallback(const uav::UWB::ConstPtr& msg)
{
}

int main(int argc, char** argv) 
{ 
    ros::init(argc, argv, "uav_pose_estimator_node"); 
    ros::NodeHandle n; 
    ros::NodeHandle np("~");

    //创建一个发布者用于发布位姿信息
    ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose>("pose", 1000);
    //创建一个订阅者用于订阅imu信息
    ros::Subscriber imu_sub = n.subscribe("imu", 1000, imuCallback);
    //创建一个订阅者用于订阅UWB信息
    ros::Subscriber uwb_sub = n.subscribe("uwb", 1000, uwbCallback);

    ros::spin(); 
    
    return 0; 
} 
