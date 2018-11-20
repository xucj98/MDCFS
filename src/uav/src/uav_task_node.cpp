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
#include <boost/bind.hpp>
#include <boost/function.hpp>

Eigen::Vector3f Now_Position[4];
Eigen::Vector3f Target_Position[4];
Eigen::Vector3d My_Angle={0,0,0};

void uavStatesSubCallback(const uav::uav_states::ConstPtr& states, int uav_id)
{
    Now_Position[uav_id].x() = states->position.x;
    Now_Position[uav_id].y() = states->position.y;
    Now_Position[uav_id].z() = states->position.z;
    // orientations[uav_id].w() = states->orientation.w;
    // orientations[uav_id].x() = states->orientation.x;
    // orientations[uav_id].y() = states->orientation.y;
    // orientations[uav_id].z() = states->orientation.z;
}

void Command_Pub(ros::Publisher Pub,float x,float y,float z)
{
        uav::task_position Target_Position;
        Target_Position.x=x;
        Target_Position.y=y;
        Target_Position.z=z;
        Target_Position.c=1;

        Pub.publish(Target_Position);
}
void Take_off(ros::Publisher& Pub)
{
        uav::task_position Target_Position;
        Target_Position.x=0;
        Target_Position.y=0;
        Target_Position.z=0;
        Target_Position.c=0;

        Pub.publish(Target_Position);
}

void Land_on(ros::Publisher Pub)
{
        uav::task_position Target_Position;
        Target_Position.x=0;
        Target_Position.y=0;
        Target_Position.z=0;
        Target_Position.c=2;
        
        Pub.publish(Target_Position);
}

int main(int argc, char** argv) 
{ 
    ros::init(argc, argv, "uav_ctrl_node"); 
    ros::NodeHandle n; 
    ros::NodeHandle np("~");

    //创建一个发布者用于发布给无人机的命令
    std::vector<ros::Publisher> tasks_pub;
    std::vector<ros::Subscriber> uavs_states_sub;

    tasks_pub.resize(4);
    uavs_states_sub.resize(4);

    for (int i = 0; i < 4; i++)
        tasks_pub[i] = n.advertise<uav::task_position>("uav_" + std::to_string(i) + "/task", 1000);

    for (int i = 0; i < 4; i++)
        uavs_states_sub[i] = n.subscribe("uav_" + std::to_string(i) + "/states", 1000, boost::function<void (const uav::uav_states::ConstPtr&)>(boost::bind(uavStatesSubCallback, _1, i)));

    ros::Rate loop_rate(1000); 

    int TASK_CASE=0;
    uint32_t My_time=0;
    uav::task_position Target_Position;
                
    sleep(3);
    while(ros::ok()) 
    { 
        My_time++;
        switch(TASK_CASE)
        {
            case 0:
            if(My_time==1)
            {
                Take_off(tasks_pub[0]);
                Take_off(tasks_pub[1]);
                Take_off(tasks_pub[2]);
                Take_off(tasks_pub[3]);
            }
                if(My_time>1000){My_time=0;TASK_CASE=1;}
                break;
            case 1:
                Command_Pub(tasks_pub[0],4,3,2.5);
                Command_Pub(tasks_pub[1],1,3,2.5);
                Command_Pub(tasks_pub[2],1,0,2.5);
                Command_Pub(tasks_pub[3],4,0,2.5);
                if(My_time>25000){My_time=0;TASK_CASE=2;}
                break;               
            case 2:
                Command_Pub(tasks_pub[0],4,3,1.8);
                Command_Pub(tasks_pub[1],1,3,1.8);
                Command_Pub(tasks_pub[2],1,0,1.8);
                Command_Pub(tasks_pub[3],4,0,1.8);
                if(My_time>5000){My_time=0;TASK_CASE=3;}
                break;
            case 3:
                if(My_time>5000){My_time=0;TASK_CASE=3;}
                break;
            default:
                break;
        }

        

        

        ros::spinOnce();
        loop_rate.sleep(); 
    } 
    
    return 0; 
} 
