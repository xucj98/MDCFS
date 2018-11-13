#include <ros/ros.h> 
#include <serial/serial.h> 
#include <iostream> 
#include <std_msgs/String.h>

//创建一个serial类 
serial::Serial sp; 

void transmitCallback(const std_msgs::String::ConstPtr& msg)
{
    sp.write((const uint8_t*)msg->data.c_str(), msg->data.size());
    ROS_INFO("transmit: %s", msg->data.c_str());
}

int main(int argc, char** argv) 
{ 
    ros::init(argc, argv, "serial_port_node"); //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错） 
    ros::NodeHandle n("~"); 
    
    std::string sp_name;
    n.param<std::string>("sp_name", sp_name, "default");
    ROS_INFO("sp_name = %s", sp_name.c_str());

    //创建timeout 
    serial::Timeout to = serial::Timeout::simpleTimeout(100); 
    //设置要打开的串口名称 
    sp.setPort(sp_name); 
    //设置串口通信的波特率 
    sp.setBaudrate(115200); 
    //串口设置timeout 
    sp.setTimeout(to); 
    try 
    { 
        //打开串口 
        sp.open(); 
    } 
    catch(serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port."); 
        return -1; 
    } 
    
    //判断串口是否打开成功 
    if(sp.isOpen()) 
    { 
        ROS_INFO("%s is opened.", sp_name.c_str()); 
    } else 
    { 
        return -1; 
    } 

    //创建一个发布者用于发布串口接收到的数据
    ros::Publisher serial_pub = n.advertise<std_msgs::String>("receive", 1000);
    ros::Subscriber serial_sub = n.subscribe("transmit", 1000, transmitCallback);

    ros::Rate loop_rate(10); 
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