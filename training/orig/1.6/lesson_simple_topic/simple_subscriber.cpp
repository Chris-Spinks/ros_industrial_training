/**
 ** Simple ROS Subscriber Node
 **/
#include <ros/ros.h>


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "simple_subscriber");
    ros::NodeHandle node;

    while(ros::ok());
    return 0;
}
