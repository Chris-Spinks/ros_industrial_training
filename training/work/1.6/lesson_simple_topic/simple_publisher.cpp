/**
 ** Simple ROS Publisher Node
 **/
#include <ros/ros.h>


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "simple_publisher");
    ros::NodeHandle node;

    while(ros::ok());
    return 0;
}
