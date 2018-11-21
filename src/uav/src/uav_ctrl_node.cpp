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
#include "uav/task_position.h"
#include <math.h>

// static serial::Serial sp; 

// void transmitCallback(const std_msgs::UInt8::ConstPtr& msg)
// {
//     sp.write(&(msg->data), 1);
// }

Eigen::Vector3d Now_Position={0,0,0};
Eigen::Vector3d Target_Position={0,0,0};
// Eigen::Vector4f Now_q={0,0,0,0};
Eigen::Vector3f My_Angle={0,0,0};

uint8_t UAV_Command =3; //命令类型 起飞0 移动1 降落2 所死3


void taskCallback(const uav::task_position::ConstPtr& msg)
{
    Target_Position[0]=msg->x;
    Target_Position[1]=msg->y;
    Target_Position[2]=msg->z;
    UAV_Command=msg->c;
    //ROS_INFO("11");
}

void stateCallback(const uav::uav_states::ConstPtr& msg)
{
    Now_Position[0]=msg->position.x;
    Now_Position[1]=msg->position.y;
    Now_Position[2]=msg->position.z;
    
    Eigen::Quaternion<float> q;
    q.x() = msg->orientation.x; q.y() = msg->orientation.y; q.z() = msg->orientation.z; q.w() = msg->orientation.w; 

    Eigen::Quaternion<float> ref;
    ref.w() = 0; ref.x() = 1; ref.y() = 0; ref.z() = 0;
    ref = q * ref * q.inverse();

    My_Angle(2)=atan2(ref.y(), ref.x());
    //ROS_INFO("yaw: %f", atan2(ref.y(), ref.x())); 
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
            float Delta_x=x2-x1;
            float Delta_y=y2-y1;
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
            //ROS_INFO("Origin_Target_Angle:%f",Target_Angle);

            if(Delta_x>0 && Delta_y>0)
            {
            }
            else if(Delta_x>0 && Delta_y<0)
            {
            }
            else if(Delta_x<0 && Delta_y<0)
            {
                 Target_Angle=-(-Target_Angle+3.1415926);
            }
            else if(Delta_x<0 && Delta_y>0)
            {
                Target_Angle=3.1415926+Target_Angle;
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

float XIangdui_BetAngle(float angle1,float angle2)
{
   float x1=cos(angle1); 
   float y1=sin(angle1); 

   float x2=cos(angle2); 
   float y2=sin(angle2); 

   float COS=x1*x2+y1*y2;

   if(COS>=0 && asin(x1*y2-y1*x2)>=0)
   return asin(x1*y2-y1*x2);
   else if (COS<0 && asin(x1*y2-y1*x2)>=0)
   return 3.1415926-asin(x1*y2-y1*x2);
   else if (COS>=0 && asin(x1*y2-y1*x2)<0)
   return asin(x1*y2-y1*x2);
   else if (COS<0 && asin(x1*y2-y1*x2)<0)
   return -3.1415926-asin(x1*y2-y1*x2);
}

int main(int argc, char** argv) 
{ 
    ros::init(argc, argv, "uav_ctrl_node"); 
    ros::NodeHandle n; 
    ros::NodeHandle np("~");

    //创建一个订阅者订阅来自EKF的位置解算
    ros::Subscriber states_sub = n.subscribe("states", 1000, stateCallback);
    ros::Subscriber task_pub = n.subscribe("task", 1000, taskCallback);
    //创建一个发布者用于发布给无人机的命令
    ros::Publisher command_pub = n.advertise<std_msgs::UInt8>("command", 1000);
    std_msgs::UInt8 Command;

    ros::Rate loop_rate(10); 

    // Now_Position[0]=2;
    // Now_Position[1]=2;
    // Now_Position[2]=0;

    // Target_Position[0]=-100;
    // Target_Position[1]=-0;
    // Target_Position[2]=0;

    while(ros::ok()) 
    { 
        if(UAV_Command==1)
        {
            if(Target_Position[2]-Now_Position[2]>0.1)
            {
                Command.data=0xA6;
            //  ROS_INFO("Go Up");
                command_pub.publish(Command);
                //sleep(0.05);
            }else if (Target_Position[2]-Now_Position[2]<-0.1)
            {
                Command.data=0xA7;
            // ROS_INFO("Go Down");
                command_pub.publish(Command);
                //sleep(0.05);
            }else
            {
            float Target_Angle;
            if(Distance(Now_Position[0],Now_Position[1],Target_Position[0],Target_Position[1])>0.5)
            {
                Target_Angle=Delta_angle(Now_Position[0],Now_Position[1],Target_Position[0],Target_Position[1]); //Yaw
                //My_Angle 无人机头朝向
                //Target_Angle 目标方向  
                //ROS_INFO("Target_Angle:%f",Target_Angle);
                //ROS_INFO("MinBet_Angle:%f",Min_BetAngle(My_Angle(2),Target_Angle));
                if(Min_BetAngle(My_Angle(2),Target_Angle)>0.1)
                {
                    Command.data=0xA5; //右转
                    //ROS_INFO("Go Right");
                    command_pub.publish(Command);
                    //sleep(0.05);
                }
                else if(Min_BetAngle(My_Angle(2),Target_Angle)<-0.1)
                {
                    Command.data=0xA4; //左转
                    //ROS_INFO("Go Left");
                    command_pub.publish(Command);
                    //sleep(0.05);
                }

                if(Min_BetAngle(My_Angle(2),Target_Angle)<0.3 && Min_BetAngle(My_Angle(2),Target_Angle)>-0.3)
                {
                    Command.data=0xA0; //前进
                //ROS_INFO("Go Forward");
                    command_pub.publish(Command);
                    //sleep(0.05);
                }

                //ROS_INFO("%f",Target_Angle);
            }
            // else if(Distance(Now_Position[0],Now_Position[1],Target_Position[0],Target_Position[1])>0.3 && Distance(Now_Position[0],Now_Position[1],Target_Position[0],Target_Position[1])<=0.5)
            // {
            //     float XiangduiAngle=XIangdui_BetAngle(My_Angle(2),Target_Angle);

            //     //ROS_INFO("Xiangdui:%f",XiangduiAngle);

            //   if(XiangduiAngle>=-3.1415926/2+3.1415926/6 && XiangduiAngle<3.1415926/2-3.1415926/6)
            //   {
            //     Command.data=0xA3; //右移
            //     command_pub.publish(Command);
            //   }  
            //   else if(XiangduiAngle>=3.1415926/2+3.1415926/6 || XiangduiAngle<-3.1415926/2-3.1415926/6)
            //   {
            //     Command.data=0xA2; //左移
            //     command_pub.publish(Command);              
            //   }
            //   else if(XiangduiAngle>=3.1415926/2-3.1415926/6 && XiangduiAngle<3.1415926/2+3.1415926/6)
            //   {
            //         Command.data=0xA0; //前进
            //        //ROS_INFO("Go Forward");
            //         command_pub.publish(Command);
            //   }
            //   else if(XiangduiAngle>=-3.1415926/2-3.1415926/6 && XiangduiAngle<-3.1415926/2+3.1415926/6)
            //   {
            //         Command.data=0xA1; //后退
            //        //ROS_INFO("Go Forward");
            //         command_pub.publish(Command);
            //   }
            // }
            }
            //Command.data=0x0A;
        //command_pub.publish(Command);
        }else if(UAV_Command==0)
        {
            Command.data=0xFC; //起飞
            command_pub.publish(Command);
            //UAV_Command=3;
        }else if(UAV_Command==2)
        {
            Command.data=0xFB; //降落
            command_pub.publish(Command);
            UAV_Command=3;
        }else if (UAV_Command==4)
        {
            Command.data=0xF1;
            // ROS_INFO("Unlock!");
            command_pub.publish(Command);
            UAV_Command=3;
        }

        ros::spinOnce();
        loop_rate.sleep();  
    }


    return 0; 
}