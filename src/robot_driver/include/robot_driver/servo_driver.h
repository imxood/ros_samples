#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <string>
#include <ecl/threads.hpp>
#include <ecl/time.hpp>
#include <ecl/devices.hpp>
#include <ecl/containers.hpp>
#include <ecl/threads/mutex.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include "robot_driver/robot_data.h"

namespace robot {

class ServoDriver {

public:

    ServoDriver();
    ~ServoDriver();

    void init();

    void start();

    void set_velocity_mode();
    void set_speed(double speed);

    void status_thread();
    void parse_status(unsigned char* buf, int size);

    void stop();

private:
    bool open_serial();

    ecl::Serial serial_;
    ecl::Thread thread_;
    ecl::Mutex data_mutex_;

    double sec_timeout;

    ChassisWheelState wheel_status_;
    bool is_alive;

};


}
#endif // SERVO_DRIVER_H
