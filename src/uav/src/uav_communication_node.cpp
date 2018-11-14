#include <ros/ros.h> 
#include <serial/serial.h> 
#include <iostream> 
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

#include "uav/uav_communication.h"

static serial::Serial sp; 

void transmitCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    sp.write(&(msg->data), 1);
}

int main(int argc, char** argv) 
{ 
    ros::init(argc, argv, "uav_communication_node"); 
    ros::NodeHandle n("~"); 
    
    //设置并打开串口
    std::string sp_name;
    n.param<std::string>("sp_name", sp_name, "default");
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
    ros::Subscriber serial_sub = n.subscribe("transmit", 1000, transmitCallback);

    ros::Rate loop_rate(5); 
    while(ros::ok()) 
    { 
        //获取缓冲区内的字节数 
        size_t n = sp.available(); 
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

        ros::spinOnce();
        loop_rate.sleep(); 
    } 
    
    //关闭串口 
    sp.close(); 
    
    return 0; 
} 
