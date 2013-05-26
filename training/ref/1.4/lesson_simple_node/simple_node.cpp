/**
 ** Simple ROS Node
 **/
#include <ros/ros.h>
#include <iostream>


int main(int argc, char* argv[])
{
    // This must be called before anything else ROS-related 
    ros::init(argc, argv, "simple_node");

    // Create a ROS node handle
    ros::NodeHandle node;

    // Set the rate at which we print out our message (1Hz)
    ros::Rate loop_rate(1.0);

    // A simple counter for the number of times we iterate through the loop
    int count = 0;

    // Loop through until the ROS system tells the user to shut down
    while(ros::ok()) {
        // Print out a message
        std::cout << "We've gone through " << count << " times." << std::endl;
        ++count;
        // Wait the stated duration
        loop_rate.sleep();
    }

    // Exit the program.
    return 0;
}
