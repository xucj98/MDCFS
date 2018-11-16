#include <ros/ros.h> 
#include <serial/serial.h> 
#include <iostream> 
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Imu.h>
#include <uav/UWB.h>
#include <uav/uav_states.h>
#include "uav/uav_communication.h"
#include <Eigen/Dense>

// static serial::Serial sp; 

// void transmitCallback(const std_msgs::UInt8::ConstPtr& msg)
// {
//     sp.write(&(msg->data), 1);
// }

Eigen::Vector3d Now_Position={0,0,0};
Eigen::Vector3d Target_Position={0,0,0};
// Eigen::Vector4f Now_q={0,0,0,0};
Eigen::Vector3d euler={0,0,0};

void stateCallback(const uav::uav_states::ConstPtr& msg)
{
    Now_Position[0]=msg->position.x;
    Now_Position[1]=msg->position.y;
    Now_Position[2]=msg->position.z;
    
    Eigen::Quaterniond q; 
    q.x() = msg->orientation.x; q.y() = msg->orientation.y; q.z() = msg->orientation.z; q.w() = msg->orientation.w; 
    euler = q.toRotationMatrix().eulerAngles(2, 1, 0); 
    
}


int main(int argc, char** argv) 
{ 
    ros::init(argc, argv, "uav_ctrl_node"); 
    ros::NodeHandle n; 
    ros::NodeHandle np("~");

    //创建一个订阅者订阅来自EKF的位置解算
    ros::Subscriber states_sub = n.subscribe("states", 1000, stateCallback);
    //创建一个发布者用于发布给无人机的命令
    ros::Publisher command_pub = n.advertise<std_msgs::UInt8>("command", 1000);
    std_msgs::UInt8 Command;

    ros::Rate loop_rate(100); 

    while(ros::ok()) 
    { 
    
        ROS_INFO("%f %f %f",euler[0],euler[1],euler[2]);
        if(Target_Position[3]-Now_Position[3]>0.05)
        {
            Command.data=0xA6;
            command_pub.publish(Command);
        }
        else if (Target_Position[3]-Now_Position[3]<-0.05)
        {
            Command.data=0xA7;
            command_pub.publish(Command);
        }
      if(sqrt((Now_Position-Target_Position).squaredNorm())<0.1)
        {


        }
        Command.data=0x0A;
       //command_pub.publish(Command);
        ros::spinOnce();
        loop_rate.sleep(); 
    } 
    
    return 0; 
} 
