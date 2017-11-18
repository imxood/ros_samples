#include <ros/ros.h>

#include <controller_manager/controller_manager.h>

#include "robot_bringup/robot.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot");
    ros::NodeHandle nh;
    ros::Rate rate(5);

    Robot<> robot;
    controller_manager::ControllerManager cm(&robot, nh);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok())
    {
        cm.update(robot.get_time(), robot.get_period());
        robot.read();
        robot.write();
        rate.sleep();
    }

    spinner.stop();

    return 0;
}
