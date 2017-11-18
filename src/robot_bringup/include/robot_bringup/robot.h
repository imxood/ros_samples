#ifndef MY_ROBOT_H
#define MY_ROBOT_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <sensor_msgs/JointState.h>

template <unsigned int NUM_JOINTS = 3>
class Robot : public hardware_interface::RobotHW
{
public:
    Robot() : wheel_left_joint_("left_wheel"), wheel_right_joint_("right_wheel")
    {
        std::fill_n(pos_, 2, 0);
        std::fill_n(vel_, 2, 0);

        // connect and register the joint state interface
        hardware_interface::JointStateHandle state_handle_left(wheel_left_joint_, &pos_[0], &vel_[0], &eff_[0]);
        jnt_state_interface_.registerHandle(state_handle_left);

        hardware_interface::JointStateHandle state_handle_right(wheel_right_joint_, &pos_[1], &vel_[1], &eff_[1]);
        jnt_state_interface_.registerHandle(state_handle_right);

        registerInterface(&jnt_state_interface_);

        // connect and register the joint position interface
        hardware_interface::JointHandle pos_handle_left(jnt_state_interface_.getHandle(wheel_left_joint_), &cmd_[0]);
        jnt_vel_interface_.registerHandle(pos_handle_left);

        hardware_interface::JointHandle pos_handle_right(jnt_state_interface_.getHandle(wheel_right_joint_), &cmd_[1]);
        jnt_vel_interface_.registerHandle(pos_handle_right);

        registerInterface(&jnt_vel_interface_);

        joint_states_.name.push_back(wheel_left_joint_);
        joint_states_.name.push_back(wheel_right_joint_);
        joint_states_.name.push_back("ommi_wheel");

        joint_states_.position.resize(NUM_JOINTS,0.0);
        joint_states_.velocity.resize(NUM_JOINTS,0.0);
        joint_states_.effort.resize(NUM_JOINTS,0.0);

        joint_state_publisher_ = nh_.advertise <sensor_msgs::JointState>("joint_states",100);
    }

    ros::Time get_time() const {return ros::Time::now();}
    ros::Duration get_period() const {return ros::Duration(0.01);}

    void read()
    {
        std::ostringstream os;
        for (unsigned int i = 0; i < NUM_JOINTS - 1; ++i)
        {
            os << cmd_[i] << ", ";
        }
        os << cmd_[NUM_JOINTS - 1];

        ROS_INFO_STREAM("Commands for joints: " << os.str());
    }

    void write()
    {
        for(int i=0; i<NUM_JOINTS; i++){
            joint_states_.velocity.at(i) = vel_[i];
            joint_states_.position.at(i) = pos_[i];
        }

        joint_states_.header.stamp = ros::Time::now();

        joint_state_publisher_.publish(joint_states_);
    }

private:
    hardware_interface::JointStateInterface    jnt_state_interface_;
    hardware_interface::VelocityJointInterface jnt_vel_interface_;

    sensor_msgs::JointState joint_states_;

    ros::Publisher joint_state_publisher_;

    double cmd_[2];
    double pos_[2];
    double vel_[2];
    double eff_[2];

    std::string wheel_left_joint_, wheel_right_joint_;

    bool running_;

    ros::NodeHandle nh_;
};


#endif // MY_ROBOT_H
