
#ifndef ROBOT_ROS_H
#define ROBOT_ROS_H

/*****************************************************************************
 ** class Robot中定义，机器人各组件底层控制
 ** class RobotRos中定义，机器人上层应用数据的通信交互
 *****************************************************************************/


#include <string>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <angles/angles.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <ecl/sigslots.hpp>
#include <controller_manager/controller_manager.h>

#include "robot_node/robot.h"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace robot
{

class RobotRos
{
public:
    RobotRos(std::string& node_name);
    ~RobotRos();
    bool init(ros::NodeHandle& nh);
    bool update();

private:


    std::string name_; // name of the ROS node

    Robot robot;

    sensor_msgs::JointState joint_states_;

    ros::NodeHandle nh_;

    ecl::Thread diff_update_thread_;

    bool cmd_vel_timed_out_;
    bool serial_timed_out_;


    /*********************
     ** Ros Comms
     **********************/
    ros::Publisher joint_state_publisher_;

    ros::Subscriber velocity_command_subscriber_;

    void advertiseTopics(ros::NodeHandle &nh);
    void subscribeTopics(ros::NodeHandle &nh);

    //设置底盘速度
    void setChassisVelocity(double l_speed, double r_speed);


    /*********************
     ** SigSlots
     **********************/
    ecl::Slot<const ChassisJointState&> slot_chassis_joint_state_;


    /*********************
     ** Slot Callbacks
     **********************/
    void publishChassisJointState(const ChassisJointState &chassis);


    /*********************
    ** Diagnostics
    **********************/
    //diagnostic_updater::Updater updater;
    //BatteryTask     battery_diagnostics;

};

} // namespace robot

#endif
