#include <ros/ros.h> 
#include <uav/uav_states.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>

void uavStatesSubCallback(const uav::uav_states::ConstPtr& states, int uav_id)
{
    ROS_INFO("uav id: %d", uav_id);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_visualization_node"); 
    
    ros::NodeHandle n;

    ros::Subscriber uav_states_sub_1 = n.subscribe("uav_1/states", 1000, boost::function<void (const uav::uav_states::ConstPtr&)>(boost::bind(uavStatesSubCallback, _1, 1)));
 
    ros::spin();

    return 0;
}