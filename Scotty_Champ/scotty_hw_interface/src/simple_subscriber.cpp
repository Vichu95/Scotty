#include <ros/ros.h>
#include <std_msgs/String.h>

// Callback function for subscriber
void messageCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received: %s", msg->data.c_str());
}

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "simple_subscriber");

    // Create a NodeHandle
    ros::NodeHandle nh;

    // Create a subscriber to the "/test_topic"
    ros::Subscriber sub = nh.subscribe("/test_topic", 10, messageCallback);

    // Keep the node running
    ros::spin();

    return 0;
}
