#include <ros/ros.h> 
#include <serial/serial.h> 
#include <iostream> 
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Imu.h>
#include <uav/UWB.h>

#include "uav/uav_communication.h"

static serial::Serial sp; 

void transmitCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    sp.write(&(msg->data), 1);
}

int main(int argc, char** argv) 
{ 
    ros::init(argc, argv, "uav_communication_node"); 
    ros::NodeHandle n; 
    ros::NodeHandle np("~");
    
    //设置并打开串口
    std::string sp_name;
    np.param<std::string>("sp_name", sp_name, "default");
    ROS_INFO("sp_name = %s", sp_name.c_str());

    serial::Timeout to = serial::Timeout::simpleTimeout(100); 
    sp.setPort(sp_name); 
    sp.setBaudrate(115200); 
    sp.setTimeout(to); 
    // try 
    // { 
    //     sp.open(); 
    // } 
    // catch(serial::IOException& e) 
    // { 
    //     ROS_ERROR_STREAM("Unable to open port."); 
    //     return -1; 
    // } 
    
    // if(sp.isOpen()) 
    // { 
    //     ROS_INFO("%s is opened.", sp_name.c_str()); 
    // } else 
    // { 
    //     return -1; 
    // } 

    //创建一个发布者用于发布串口接收到的数据
    ros::Publisher serial_pub = n.advertise<std_msgs::UInt8>("receive", 1000);
    //创建一个订阅者用于订阅需要发往无人机的数据
    ros::Subscriber serial_sub = n.subscribe("transmit", 1000, transmitCallback);
    //创建一个发布者用于发布IMU信息
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 1000);
    ros::Publisher uwb_pub = n.advertise<uav::UWB>("uwb", 1000);

    ros::Rate loop_rate(20); 
    while(ros::ok()) 
    { 
        //获取缓冲区内的字节数 
        // size_t n = sp.available(); 
        size_t n = 0;
        if(n!=0) 
        { 
            uint8_t buffer[1024];
            
            //读出数据 
            n = sp.read(buffer, n);
            buffer[n] = '\0';
            
            //类型转换
            std::string str((char*)buffer);
            std::stringstream ss;
            ss << str; 
            std_msgs::String msg; 
            msg.data = ss.str();
            
            ROS_INFO("receive %s", msg.data.c_str());
            serial_pub.publish(msg);
            
            // sp.write(buffer, n); 
        } 
        
        sensor_msgs::Imu imu;
        imu.header.stamp = ros::Time(1032);

        imu.linear_acceleration.x = 1.0;
        imu.linear_acceleration.y = 2.0;
        imu.linear_acceleration.z = 1.0;

        imu.angular_velocity.x = 0.01;
        imu.angular_velocity.y = 0.02;
        imu.angular_velocity.z = 0.03;
        
        imu_pub.publish(imu);

        uav::UWB uwb;
        uwb.header.stamp = ros::Time(11032);
        uwb.d0 = 30;
        uwb.d1 = 50;
        uwb.d2 = 30;
        uwb.d3 = 50;

        uwb_pub.publish(uwb);

        ros::spinOnce();
        loop_rate.sleep(); 
    } 
    
    //关闭串口 
    sp.close(); 
    
    return 0; 
} 
