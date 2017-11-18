#ifndef ROBOT_DATA_H
#define ROBOT_DATA_H

namespace robot {

//底盘车轮信息
class ChassisWheelState {

public:
    ChassisWheelState(int status = 0, int voltage = 0, float electricity = 0.0, int speed = 0, int pos = 0, int pos_feedback = 0) :
        status(status), voltage(voltage), electricity(electricity),
        speed(speed), pos(pos), pos_feedback(pos_feedback)
    {}

    enum {
        LEFT, RIGHT
    } target;           //左轮还是右轮

    int status;         //故障状态
    int voltage;        //母线电压
    float electricity;  //输出电流
    int speed;          //转速
    int pos;            //位置
    int pos_feedback;   //位置反馈

};

//底盘关节信息
class ChassisJointState {

public:
    ChassisJointState(double l_cmd = 0.0, double l_pos = 0.0, double l_vel = 0.0, double l_eff = 0.0, double r_cmd = 0.0, double r_pos = 0.0, double r_vel = 0.0, double r_eff = 0.0)
    {
        cmd_[0] = l_cmd; cmd_[1] = r_cmd;
        pos_[0] = l_pos; pos_[1] = r_pos;
        vel_[0] = l_vel; vel_[1] = r_vel;
        eff_[0] = l_eff; eff_[1] = r_eff;
    }

    double cmd_[2];
    double pos_[2]; //位置
    double vel_[2]; //速度
    double eff_[2];

};

}

#endif // ROBOT_DATA_H
