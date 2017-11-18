
#include <robot_node/robot.h>

namespace robot {


Robot::Robot() : is_alive_(false)
{

}

Robot::~Robot()
{
    chassis_thread_.join();
}

void Robot::init(Parameters parameters)
{
    servo_.init();
    chassisHW_.init(parameters);

    //启动底盘更新线程
    chassis_thread_.start(&Robot::chassis_update, *this);
}


void Robot::chassis_update(){

    controller_manager::ControllerManager cm_(&chassisHW_, ros::NodeHandle());
    ros::Rate rate(20);

    while(is_alive_){
        cm_.update(ros::Time::now(), rate.expectedCycleTime());
        chassisHW_.publish_joint_state();
    }

}


}
