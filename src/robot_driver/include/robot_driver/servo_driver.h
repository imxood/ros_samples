#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <string>
#include <ros/ros.h>
#include <ecl/threads.hpp>
#include <ecl/time.hpp>
#include <ecl/devices.hpp>
#include <ecl/containers.hpp>
#include <ecl/threads/mutex.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include <boost/shared_ptr.hpp>


class ServoDriver {

    struct wheel_data {
        int status;         //故障状态
        int voltage;        //母线电压
        int electricity;    //输出电流(实际电流要除以100)
        int speed;          //转速
        int pos;            //位置
        int pos_feedback;   //位置反馈
    };

    enum WheelStatus{

    };

public:

    ServoDriver();
    ~ServoDriver();

    void start_vectory();
    void start_position();

    void read_status();
    void parse_status(unsigned char* buf, int size);

private:
    bool open_serial();

    ecl::Serial serial_;
    ecl::Thread thread_;
    ecl::Mutex data_mutex_;

    ecl::PushAndPop<unsigned char> data_;

    struct wheel_data wheel_status_;
    bool is_alive;

};

#endif // SERVO_DRIVER_H
