#include "robot_node/robot_ros.h"

namespace robot {

//发布底座车轮关节状态
void RobotRos::publishChassisJointState(const ChassisJointState &jointState)
{
    if (ros::ok())
    {
        for(int i=0; i<2; i++){
            joint_states_.velocity.at(i) = jointState.vel_[i];
            joint_states_.position.at(i) = jointState.pos_[i];
        }

        joint_states_.header.stamp = ros::Time::now();

        joint_state_publisher_.publish(joint_states_);

        setChassisVelocity(jointState.cmd_[0], jointState.cmd_[1]);
    }
}

//设置底盘速度
void RobotRos::setChassisVelocity(double l_speed, double r_speed)
{
    if (ros::ok())
    {
        //servo_.set_speed(l_speed);
    }
}

}
