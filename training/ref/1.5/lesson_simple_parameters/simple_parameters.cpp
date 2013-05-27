/**
 ** Simple ROS Node
 **/
#include <ros/ros.h>
#include <string>


int main(int argc, char* argv[])
{
    // This must be called before anything else ROS-related 
    ros::init(argc, argv, "simple_node");

    // Create ROS node handles
    ros::NodeHandle node;
    ros::NodeHandle local_node("~");

    // Set the rate at which we print out our message (1Hz)
    ros::Rate loop_rate(1.0);

    // A simple counter for the number of times we iterate through the loop
    int count = 0;

    // Loop through until the ROS system tells the user to shut down
    while(ros::ok()) {
        // Print out a message
        ROS_INFO("We've gone through %d times", count);
        ++count;
        
        // --== ADD NEW LINES HERE... ==-- //
        // Print out the value we've read, if it has been set
        int value_int;
        if(!node.hasParam("integer")) {
            ROS_WARN("Integer parameter is not set.");
        } else if(node.getParam("integer", value_int)) {
            ROS_INFO("Integer parameter is currently %d.", value_int);
        } else {
            ROS_ERROR("Integer parameter is not an integer.");
        }

        std::string value_string;
        if(!node.hasParam("values/string")) {
            ROS_WARN("String parameter is not set.");
        } else if(node.getParam("values/string", value_string)) {
            ROS_INFO("String parameter is currently '%s'.", value_string.c_str());
        } else {
            ROS_ERROR("String parameter is not a string."); 
        }

        double x, y;
        node.param("/values/point/x", x, 0.0);
        node.param("/values/point/y", y, 0.0);
        local_node.setParam("point_sum", x + y);

        // Wait the stated duration
        loop_rate.sleep();
    }

    // Exit the program.
    return 0;
}
