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


int uav_count;
int trajectory_count;
int trajectory_stamp;
Eigen::Vector3f Now_Position[4];
Eigen::Vector3f Target_Position[4];
Eigen::Vector3d My_Angle={0,0,0};

std::vector<std::vector<Eigen::Vector3f> >  uavs_trajectory;

void createTrajectory()
{
    uav_count = 3;
    trajectory_count = 2;
    uavs_trajectory.resize(uav_count);
    uavs_trajectory[0].resize(trajectory_count);
    uavs_trajectory[0][0] << 4, 0.3, 2.2;
    uavs_trajectory[0][1] << 1, 0.3, 1.8;

    uavs_trajectory[1].resize(trajectory_count);
    uavs_trajectory[1][0] << 4, 1.6, 2.2;
    uavs_trajectory[1][1] << 1, 1.6, 1.8;

    uavs_trajectory[2].resize(trajectory_count);
    uavs_trajectory[2][0] << 4, 2.9, 2.2;
    uavs_trajectory[2][1] << 1, 2.9, 1.8;
    
}

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

void Unlock(ros::Publisher Pub)
{
        uav::task_position Target_Position;
        Target_Position.x=0;
        Target_Position.y=0;
        Target_Position.z=0;
        Target_Position.c=4;
        
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
    ros::Time start_time;

    createTrajectory();

    int TASK_CASE=0;
    float My_time=0;
    uav::task_position Target_Position;
            
    sleep(5);
    for (int i = 0; i < uav_count; i++)
        Unlock(tasks_pub[i]);
    sleep(1);

    start_time = ros::Time::now();
    trajectory_stamp = -2;

    while(ros::ok()) 
    { 
        My_time = (ros::Time::now() - start_time).toSec();
        if (trajectory_stamp == -2)
        {
            for (int i = 0; i < uav_count; i++)
                Take_off(tasks_pub[i]);
            trajectory_stamp = -1;
        }

        if (My_time < 3) continue;
        
        if (trajectory_stamp == -1)
        {
            printf("trajectory stamp -1 finished\n");
            trajectory_stamp++;
            for (int i = 0; i < uav_count; i++)
                Command_Pub(tasks_pub[i], uavs_trajectory[i][trajectory_stamp].x(), uavs_trajectory[i][trajectory_stamp].y(), uavs_trajectory[i][trajectory_stamp].z());
        }

        bool arrive_flag = true;
        for (int i = 0; i < uav_count; i++)
            if ((uavs_trajectory[i][trajectory_stamp] - Now_Position[i]).squaredNorm() > 0.25)
                arrive_flag = false;

        if (arrive_flag)
        {
            printf("trajectory stamp %d finished\n", trajectory_stamp);
            trajectory_stamp++;
            if (trajectory_stamp < trajectory_count)
            {
                for (int i = 0; i < uav_count; i++)
                    Command_Pub(tasks_pub[i], uavs_trajectory[i][trajectory_stamp].x(), uavs_trajectory[i][trajectory_stamp].y(), uavs_trajectory[i][trajectory_stamp].z());
            }
        }
        
        if (trajectory_stamp == trajectory_count)
        {
            for (int i = 0; i < uav_count; i++)
                Land_on(tasks_pub[i]);
            while(1);
        }

        ros::spinOnce();
        loop_rate.sleep(); 
    } 
    
    return 0; 
} 
