#ifndef ROBOT_H
#define ROBOT_H

/**
 * @brief The Robot class
 * class Robot中定义，机器人各组件底层控制
 * class RobotRos中定义，机器人上层应用数据的通信
 */

#include <ecl/threads.hpp>

#include <robot_driver/parameters.h>
#include <robot_driver/chassis_hw.h>
#include <robot_driver/servo_driver.h>


namespace robot {

class Robot {

public:
    Robot();

    ~Robot();

    void init(Parameters parameters);

private:
    bool is_alive_;                 //机器人当前是否在运行

    ChassisHW chassisHW_;           //底盘左右轮的控制

    ServoDriver servo_;             //伺服机驱动

    /********************
     * 线程对象
     ********************/
    ecl::Thread chassis_thread_;

    /********************
     * 线程函数
     ********************/
    void chassis_update();              //更新底盘

};

}

#endif // ROBOT_H
