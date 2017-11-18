
#include <ros/ros.h>

int main (int argc, char** argv) {

    ros::init(argc, argv, "test_node1");

    ros::NodeHandle nh("~");

    ros::Rate rate(2);

    while(ros::ok()){
        ROS_INFO("Hello, test node!");
        rate.sleep();
    }

    return 0;

}