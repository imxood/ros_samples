#ifndef CHASSIS_HW_H
#define CHASSIS_HW_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <controller_manager/controller_manager.h>

#include <ecl/config.hpp>
#include <ecl/threads.hpp>
#include <ecl/devices.hpp>
#include <ecl/threads/mutex.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/sigslots.hpp>

#include "robot_driver/parameters.h"
#include "robot_driver/robot_data.h"

namespace robot {

//底盘控制
//三个轮子，两个驱动轮，一个万向轮
class ChassisHW : public hardware_interface::RobotHW
{
public:
    ChassisHW() {}

    ~ChassisHW() {
        ROS_INFO("ChassisHW::~ChassisHW()");
    }

    void init(Parameters parameters) {

        ROS_INFO("ChassisHW::init()!!!");

        if (!parameters.validate())
        {
            throw ecl::StandardException(LOC, ecl::ConfigurationError, "robot's parameters settings did not validate.");
        }

        std::fill_n(cmd_, 2, 0);
        std::fill_n(pos_, 2, 0);
        std::fill_n(vel_, 2, 0);
        std::fill_n(eff_, 2, 0);

        //注册相关HardWare接口

        //注册关节state接口
        hardware_interface::JointStateHandle state_handle_left(parameters.left_wheel_joint, &pos_[0], &vel_[0], &eff_[0]);
        jnt_state_interface_.registerHandle(state_handle_left);

        hardware_interface::JointStateHandle state_handle_right(parameters.right_wheel_joint, &pos_[1], &vel_[1], &eff_[1]);
        jnt_state_interface_.registerHandle(state_handle_right);

        registerInterface(&jnt_state_interface_);

        //注册关节command接口
        hardware_interface::JointHandle pos_handle_left(jnt_state_interface_.getHandle(parameters.left_wheel_joint), &cmd_[0]);
        jnt_vel_interface_.registerHandle(pos_handle_left);

        hardware_interface::JointHandle pos_handle_right(jnt_state_interface_.getHandle(parameters.right_wheel_joint), &cmd_[1]);
        jnt_vel_interface_.registerHandle(pos_handle_right);

        registerInterface(&jnt_vel_interface_);

    }



    ros::Time get_time() const {return ros::Time::now();}

    ros::Duration get_period() const {return ros::Duration(0.01);}

    void publish_joint_state()
    {
        sig_chassis_joint_state.emit(ChassisJointState(cmd_[0], pos_[0], vel_[0], eff_[0], cmd_[1], pos_[1], vel_[1], eff_[1]));
    }


private:
    hardware_interface::JointStateInterface    jnt_state_interface_;
    hardware_interface::VelocityJointInterface jnt_vel_interface_;

    ecl::Signal<const ChassisJointState&> sig_chassis_joint_state;

public:
    double cmd_[2];
    double pos_[2];
    double vel_[2];
    double eff_[2];

};


}

#endif
