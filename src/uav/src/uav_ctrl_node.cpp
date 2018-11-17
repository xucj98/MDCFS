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

float Distance(float x1,float y1,float x2,float y2)
{
return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

float Delta_angle(float x1,float y1,float x2,float y2)
{
    //   float Delta_x=Target_Position[0]-Now_Position[0];
    //   float Delta_y=Target_Position[1]-Now_Position[1];
       float Target_Angle;
       float Delta_x=x1-x2;
       float Delta_y=y1-y2;
            if(Delta_y==0)
            {
                if (Delta_x>=0) Target_Angle=0;
                else Target_Angle=3.1415926;
            }
            else if(Delta_x==0)
            {
                if (Delta_y>=0) Target_Angle=3.1415926/2;
                else Target_Angle=-3.1415926/2;
            }
            else
            Target_Angle=atan(Delta_y/Delta_x);

            if(Delta_x>0 && Delta_y>0)
            {}
            else if(Delta_x>0 && Delta_y<0)
            {
                Target_Angle+=3.1415926;
            }
            else if(Delta_x<0 && Delta_y<0)
            {
                 Target_Angle-=3.1415926;
            }
            else if(Delta_x<0 && Delta_y>0)
            {
            }
            return Target_Angle;

}

float Min_BetAngle(float angle1,float angle2)
{
    if(angle1>=0 && angle2>=0) return (angle1-angle2);
    else if(angle1>=0 && angle2<=0) 
    {
        if((angle1-angle2)>=(2*3.1415926-angle1+angle2)) return (-(2*3.1415926-angle1+angle2));
        else return (angle1-angle2);
    }
    else if(angle1<=0 && angle2<=0) return (angle1-angle2);
    else if(angle1<=0 && angle2>=0) 
    {
        if((angle2-angle1)>=(2*3.1415926-angle2+angle1)) return ((2*3.1415926-angle2+angle1));
        else return (-angle2+angle1);
    }
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

    ros::Rate loop_rate(2); 

    Now_Position[0]=2;
    Now_Position[1]=2;
    Now_Position[2]=0;

    Target_Position[0]=100;
    Target_Position[1]=100;
    Target_Position[2]=0;

    while(ros::ok()) 
    { 
       //ROS_INFO("%f %f %f",euler[0],euler[1],euler[2]);
    
       //ROS_INFO("%f",atan(-1));
        if(Target_Position[2]-Now_Position[2]>0.1)
        {
            Command.data=0xA6;
            ROS_INFO("Go Up");
            command_pub.publish(Command);
        }
        else if (Target_Position[2]-Now_Position[2]<-0.1)
        {
            Command.data=0xA7;
            ROS_INFO("Go Down");
            command_pub.publish(Command);
        }
      float Target_Angle;
      if(Distance(Target_Position[0],Target_Position[1],Now_Position[0],Now_Position[1])>0.1)
        {
            Target_Angle=Delta_angle(Target_Position[0],Target_Position[1],Now_Position[0],Now_Position[1]); //Yaw
            ROS_INFO("My_Angle:%f",My_Angle(2));
            ROS_INFO("Target_Angle:%f",Target_Angle);
            //My_Angle 无人机头朝向
            //Target_Angle 目标方向
            // float Temp_x1=0,Temp_y1=0,Temp_x2=0,Temp_y2=0;
            // Temp_x1=cos(My_Angle);
            // Temp_y1=sin(My_Angle);
            // Temp_x2=cos(Target_Angle);
            // Temp_y2=sin(Target_Angle);   

            if(Min_BetAngle(My_Angle(2),Target_Angle)>0.1)
            {
                Command.data=0xA5; //右转
                ROS_INFO("Go Right");
                command_pub.publish(Command);
            }
            else if(Min_BetAngle(My_Angle(2),Target_Angle)<-0.1)
            {
                Command.data=0xA4; //左转
                ROS_INFO("Go Left");
                command_pub.publish(Command);
            }
            if(Min_BetAngle(My_Angle(2),Target_Angle)<0.3 && Min_BetAngle(My_Angle(2),Target_Angle)>-0.3)
            {
                Command.data=0xA0; //前进
                ROS_INFO("Go Forward");
                command_pub.publish(Command);
            }

            //ROS_INFO("%f",Target_Angle);
        }


        //Command.data=0x0A;
       //command_pub.publish(Command);
        ros::spinOnce();
        loop_rate.sleep(); 
    } 
    
    return 0; 
} 
