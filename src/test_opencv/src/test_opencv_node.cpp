#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>

using namespace std;

const string img_path = "/home/jay/Datasets/MH_01_pictures/";

int main(int argc, char **argv)
{
    ros::init(argc,argv,"example");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("message",1000);
    ros::Rate loop_rate(10);
    
    int index = 1;
    
    while(ros::ok())
    {
        cv::Mat img;
        
        img = cv::imread(img_path + std::to_string(index) + ".jpg");
        cv::imshow("img", img);
        index++;

        cv::waitKey(10);

        ros::spinOnce();
        loop_rate.sleep(); 
    } 
    return 0; 
}
