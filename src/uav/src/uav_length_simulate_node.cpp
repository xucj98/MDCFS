#include <ros/ros.h> 
#include <uav/UWB.h>
#include <stdlib.h>
#include <sensor_msgs/Imu.h>

float time_stamp = -1;

static void imuSubCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    time_stamp = msg->header.stamp.toSec();
}

int main(int argc, char** argv) 
{ 
    ros::init(argc, argv, "uav_length_simulate_node"); 
    ros::NodeHandle n; 
    ros::NodeHandle np("~");

    ros::Subscriber imu_sub = n.subscribe("imu", 1000, imuSubCallback);    
    ros::Publisher uwb_pub = n.advertise<uav::UWB>("uwb", 1000);

    ros::Rate loop_rate(1); 

    while(ros::ok()) 
    { 
        ros::spinOnce();

        if (time_stamp < 0) continue;

        uav::UWB uwb;
        uwb.header.stamp = ros::Time(time_stamp);

        uwb.d0 = 4.9 + (float)(rand() % 15) / 100.0;
        uwb.d1 = 5.8 + (float)(rand() % 15) / 100.0;
        uwb.d2 = 4.5 + (float)(rand() % 15) / 100.0;
        uwb.d3 = 2.3 + (float)(rand() % 15) / 100.0;
                          
        uwb_pub.publish(uwb);
                        
        loop_rate.sleep(); 
    } 
    return 0; 
} 
