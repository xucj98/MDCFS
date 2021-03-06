#include <ros/ros.h> 
#include <serial/serial.h> 
#include <iostream> 
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Imu.h>
#include <uav/UWB.h>
#include <uav/baro.h>

#include "uav/uav_communication.h"

static serial::Serial sp; 

void transmitCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    //ROS_INFO("Received = %c",msg->data);
    sp.write(&(msg->data), 1);
}

int main(int argc, char** argv) 
{ 
    ros::init(argc, argv, "uav_communication_node"); 
    ros::NodeHandle n; 
    ros::NodeHandle np("~");
    
    //设置并打开串口
    std::string sp_name;
    np.param<std::string>("sp_name", sp_name, "/dev/rfcomm3");
    ROS_INFO("sp_name = %s", sp_name.c_str());

    serial::Timeout to = serial::Timeout::simpleTimeout(100); 
    sp.setPort(sp_name); 
    sp.setBaudrate(115200); 
    sp.setTimeout(to); 
    try 
    { 
        sp.open(); 
    } 
    catch(serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port."); 
        return -1; 
    } 
    
    if(sp.isOpen()) 
    { 
        ROS_INFO("%s is opened.", sp_name.c_str()); 
    } else 
    { 
        return -1; 
    } 

    //创建一个发布者用于发布串口接收到的数据
    ros::Publisher serial_pub = n.advertise<std_msgs::UInt8>("receive", 1000);
    //创建一个订阅者用于订阅需要发往无人机的数据
    ros::Subscriber serial_sub = n.subscribe("command", 1000, transmitCallback);
    
    //创建一个发布者用于发布IMU信息
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 1000);
    ros::Publisher uwb_pub = n.advertise<uav::UWB>("uwb", 1000);
    ros::Publisher baro_pub = n.advertise<uav::baro>("baro", 1000);

    ros::Rate loop_rate(10000); 

    size_t Buffer_length=0; //buffer数组长度
    int state=0;            //数据状态
    uint8_t buffer[1024000];

    while(ros::ok()) 
    { 
             
        size_t n = sp.available(); 
    
        if(1) 
        { 
            
            n= sp.read(&buffer[Buffer_length], n); 
            Buffer_length+=n;

            // std::string str((char*)buffer);
            // std::stringstream ss;
            // ss << str; 
            // std_msgs::String msg; 
            // msg.data = ss.str();            
            // ROS_INFO("receive %s", msg.data.c_str());

            // for(int j=0;j<Buffer_length;j++)
            // {          
            // ROS_INFO("receive %x",buffer[j]);
            // }
            switch(state)
            {
                case 0:
                    if(buffer[0]==0x0A) state=1;
                    else Buffer_length=0;
                    break;
                case 1:
                    if((buffer[1]==0x0C || buffer[1]==0x0B) && Buffer_length>=2) state=2;
                    else if(Buffer_length>=2)
                    {
                        state=0;
                        Buffer_length=0;
                    }
                    break;
                case 2:
                    if(buffer[1]==0x0C)
                    {
                    if(Buffer_length<15) 
                    {}
                    else
                    {    
                         uav::UWB uwb;
                         uint8_t check=0;
                         for(int j=0;j<14;j++)
                         check+=buffer[j];
                                            
                        if(check==buffer[14])
                        {
                        //  ROS_INFO("time stamp: %ld",temp_time);
                         long temp_time=(buffer[2]<<24)+(buffer[3]<<16)+(buffer[4]<<8)+buffer[5];

                         uwb.header.stamp = ros::Time((float)temp_time/1000);
                         uwb.d0 = ((float)((buffer[6]<<8)+buffer[7]))/1000-0.4;
                         if(uwb.d0<0) uwb.d0=0;
                         uwb.d1 = ((float)((buffer[8]<<8)+buffer[9]))/1000-0.4;
                         if(uwb.d1<0) uwb.d1=0;
                         uwb.d2 = ((float)((buffer[10]<<8)+buffer[11]))/1000-0.4;
                         if(uwb.d2<0) uwb.d2=0;
                         uwb.d3 = ((float)((buffer[12]<<8)+buffer[13]))/1000-0.4;
                         if(uwb.d3<0) uwb.d3=0;

                        uwb_pub.publish(uwb);
                        }
                        state=0;
                        Buffer_length=0;
                     }
                    }
                    else if(buffer[1]==0x0B)
                    {
                        if(Buffer_length<21) 
                        {}
                        else
                        {                          
                         uint8_t check=0;
                         for(int j=0;j<28;j++)
                         check+=buffer[j];

                        if(check==buffer[28])
                        {
                        long long temp_time2=(buffer[2]<<24)+(buffer[3]<<16)+(buffer[4]<<8)+buffer[5];
                        // ROS_INFO("%lld",temp_time2);
                       
                        sensor_msgs::Imu imu;
                        imu.header.stamp = ros::Time((float)temp_time2/1000);

                        imu.linear_acceleration.x = ((short)((buffer[6]<<8)+buffer[7]))/1000.0;
                        imu.linear_acceleration.y = ((short)((buffer[8]<<8)+buffer[9]))/1000.0;
                        imu.linear_acceleration.z = ((short)((buffer[10]<<8)+buffer[11]))/1000.0;

                        imu.angular_velocity.x = ((short)((buffer[12]<<8)+buffer[13]))/1000.0;
                        imu.angular_velocity.y = ((short)((buffer[14]<<8)+buffer[15]))/1000.0;
                        imu.angular_velocity.z = ((short)((buffer[16]<<8)+buffer[17]))/1000.0;  

                        imu.orientation.x=((short)((buffer[20]<<8)+buffer[21]))/1000.0;
                        imu.orientation.y=((short)((buffer[22]<<8)+buffer[23]))/1000.0;
                        imu.orientation.z=((short)((buffer[24]<<8)+buffer[25]))/1000.0;
                        imu.orientation.w=((short)((buffer[26]<<8)+buffer[27]))/1000.0;

                        imu_pub.publish(imu);

                        uav::baro baro;
                        baro.header.stamp  = ros::Time((float)temp_time2/1000);
                        baro.data = ((short)((buffer[18]<<8)+buffer[19]))/100.0;       
                        baro_pub.publish(baro);

                         //ROS_INFO("Acc:receive %f %f %f",imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z);
                         //ROS_INFO("Ang:receive %f %f %f",imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z);
                        }
                          state=0;
                          Buffer_length=0;
                          }
                    }
                    break;
                default: break;
            }
        } 
    
        ros::spinOnce();
        loop_rate.sleep(); 
    } 
    
    //关闭串口 
    sp.close(); 
    
    return 0; 
} 
