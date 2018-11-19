#include <ros/ros.h> 
#include <serial/serial.h> 
#include <iostream> 
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Imu.h>
#include <uav/UWB.h>
#include <uav/uav_states.h>
#include "uav/uav_communication.h"
#include "uav/task_position.h"
#include <Eigen/Dense>

Eigen::Vector3d Now_Position={0,0,0};
Eigen::Vector3d Target_Position={0,0,0};
// Eigen::Vector4f Now_q={0,0,0,0};
Eigen::Vector3d My_Angle={0,0,0};

void stateCallback(const uav::uav_states::ConstPtr& msg)
{
    Now_Position[0]=msg->position.x;
    Now_Position[1]=msg->position.y;
    Now_Position[2]=msg->position.z;
    
    Eigen::Quaterniond q; 
    q.x() = msg->orientation.x; q.y() = msg->orientation.y; q.z() = msg->orientation.z; q.w() = msg->orientation.w; 
    My_Angle = q.toRotationMatrix().eulerAngles(2, 1, 0); 
}

int main(int argc, char** argv) 
{ 
    ros::init(argc, argv, "uav_ctrl_node"); 
    ros::NodeHandle n; 
    ros::NodeHandle np("~");

    //创建一个订阅者订阅来自EKF的位置解算
    ros::Subscriber states_sub = n.subscribe("states", 1000, stateCallback);
    //创建一个发布者用于发布给无人机的命令
    ros::Publisher task_pub = n.advertise<uav::task_position>("task", 1000);

    ros::Rate loop_rate(1); 
    while(ros::ok()) 
    { 
        uav::task_position Target_Position;
        //ROS_INFO("1");
        Target_Position.x=3;
        Target_Position.y=3;
        Target_Position.z=2.5;
        task_pub.publish(Target_Position);

        //ROS_INFO("1");
        ros::spinOnce();
        loop_rate.sleep(); 
    } 
    
    return 0; 
} 
