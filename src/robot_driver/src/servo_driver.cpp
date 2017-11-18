
#include "robot_driver/servo_driver.h"

namespace robot {


ServoDriver::ServoDriver() : is_alive(false), sec_timeout(5) {}

ServoDriver::~ServoDriver()
{
    ROS_INFO("ServoDriver::~ServoDriver()");
    stop();
    thread_.join();
    serial_.close();
}

void ServoDriver::init()
{
    start();

    set_velocity_mode();

    set_speed(0.44);

    thread_.start(&ServoDriver::status_thread, *this);
}

//启动电机
void ServoDriver::start() {

    ROS_INFO("ServoDriver::start()");

    data_mutex_.lock();

    unsigned char buf[32] = {0x00, 0x00, 0x01, 0x01};

    int count = 0;

    if( !open_serial() ) {
        data_mutex_.unlock();
        return;
    }

    if(serial_.write(buf, 4) != 4){
        ROS_INFO("ServoDriver::start: write fail!");
        data_mutex_.unlock();
        return;
    }

    if( ( count = serial_.read(buf, 2)) != 2){
        ROS_INFO_STREAM("ServoDriver::start: read fail! count: " << count);
        data_mutex_.unlock();
        return;
    }

    ROS_INFO("start motor successed!");
    data_mutex_.unlock();

}

//设置速度模式
void ServoDriver::set_velocity_mode(){

    data_mutex_.lock();

    unsigned char buf[32] = {0x02, 0x00, 0xc4, 0xc6};

    if( !open_serial() ) {
        data_mutex_.unlock();
        return;
    }

    if(serial_.write(buf, 4) != 4){
        ROS_INFO("ServoDriver::set_velocity_mode: write fail!");
        data_mutex_.unlock();
        return;
    }

    if(serial_.read(buf, 2) != 2){
        ROS_INFO("ServoDriver::set_velocity_mode: read fail!");
        data_mutex_.unlock();
        return;
    }

    ROS_INFO("set velocity mode successed!");

    is_alive = true;

    data_mutex_.unlock();

}

//设置速度,speed: m/s
void ServoDriver::set_speed(double speed)
{
    data_mutex_.lock();

    unsigned char buf[32] = {0x06};

    if( !open_serial() ) {
        data_mutex_.unlock();
        return;
    }

    double rpm = speed * 60 / 0.53;

    ROS_DEBUG_STREAM("set rpm: " << rpm);

    //把速度m/s转换成每分钟多少转，并转换成要写入的数据
    short value = (short)(rpm/6000*16384);

    ROS_DEBUG_STREAM("set value: " << value);

    //设置速度指令
    buf[1] = (unsigned char)(value >> 8 & 0xff);
    buf[2] = (unsigned char)(value & 0xff);
    buf[3] = buf[0] + buf[1] + buf[2];

    printf("buf[1]:%d, buf[2]:%d\n", buf[1], buf[2]);

    if(serial_.write(buf, 4) != 4){
        ROS_INFO("ServoDriver::set_speed: write fail!");
        data_mutex_.unlock();
        return;
    }

    if(serial_.read(buf, 2) != 2){
        ROS_INFO("ServoDriver::set_speed: read fail!");
        data_mutex_.unlock();
        return;
    }

    ROS_INFO_STREAM("set speed " << speed << "m/s successed!");

    data_mutex_.unlock();

}


//单独启动的一个线程，每50ms读一次轮子的状态
void ServoDriver::status_thread() {

    int size = 32, i;  //一共32个char型
    unsigned char buf[size] = {0};

    ros::Rate rate(4);

    while(ros::ok() && open_serial()) {

        if(!is_alive){
            //stop();
            ROS_INFO("is_alive false");
            rate.sleep();
            continue;
        }

        data_mutex_.lock();

        //发送监控指令
        buf[0] = 0x80; buf[1] = 0x00; buf[2] = 0x80;

        if(serial_.write(buf, 3) != 3){
            ROS_INFO("ServoDriver::status_thread: write fail!");
            data_mutex_.unlock();
            rate.sleep();
            return;
        }

        if(serial_.read(buf, size) != size){
            ROS_INFO("ServoDriver::status_thread: read fail!");
            data_mutex_.unlock();
            rate.sleep();
            return;
        }

        parse_status(buf, size);

        ROS_DEBUG_STREAM("voltage: " << wheel_status_.voltage << "V, "
                         "electricity: " << wheel_status_.electricity <<"A, "
                         "speed: " << wheel_status_.speed <<"RPM, "
                         "pos: " << wheel_status_.pos <<", "
                         "pos_feedback: " << wheel_status_.pos_feedback);

        data_mutex_.unlock();
        rate.sleep();

        continue;

    }

}

//解析状态
void ServoDriver::parse_status(unsigned char* buf, int size)
{
    //四个数据为一帧 格式为: 地址 数据高八位 数据低八位 校验和(取低八位)
    //unsigned char buf[] = {0x80, 0x00, 0x01, 0x81, 0xE1, 0x00, 0x1C, 0xFD, 0xE2, 0x00, 0x11, 0xF3, 0xE4, 0x01, 0x0E, 0xF3, 0xE6, 0x00, 0x00, 0xE6, 0xE7, 0x00, 0x00, 0xE7, 0xE8, 0x00, 0x00, 0xE8, 0xE9, 0xDC, 0x0B, 0xD0};

    unsigned int a, b, c, d;

    for(int i=0; i<size; i+=4){

        a=buf[i+0], b=buf[i+1], c=buf[i+2], d=buf[i+3];

        if( (a+b+c & 0xff) != d){//校验失败
            break;
        }

        switch(i/4){

        case 0:     //故障状态

            break;

        case 1:     //母线电压
            wheel_status_.voltage = (b<<8) + c;
            break;

        case 2:     //输出电流
            wheel_status_.electricity = ((b<<8) + c)/100.0;
            break;

        case 3:     //输出转速
            wheel_status_.speed = ((b<<8) + c) * 3000 >> 13;
            break;

        case 4:     //位置
            wheel_status_.pos = (b<<24) + (c<<16);
            break;

        case 5:
            wheel_status_.pos = wheel_status_.pos + ((b<<8) + c);
            break;

        case 6:     //位置反馈
            wheel_status_.pos_feedback = (b<<24) + (c<<16);
            break;

        case 7:
            wheel_status_.pos_feedback = wheel_status_.pos + (b<<8) + c;
            break;
        }

    }
}

//打开串口
bool ServoDriver::open_serial()
{

    while(!serial_.open()){
        try {
            serial_.open("/dev/ttyUSB0", ecl::BaudRate_57600, ecl::DataBits_8, ecl::StopBits_1, ecl::NoParity);
            serial_.block(1000);

            ROS_INFO("open port success!");
        }
        catch (const ecl::StandardException &e)
        {
            if (e.flag() == ecl::NotFoundError) {
                ROS_INFO_STREAM("no valid poat to use! " << std::string(e.what()));
            } else if (e.flag() == ecl::OpenError) {
                ROS_INFO_STREAM("open port failed! " << std::string(e.what()));
            } else {
                ROS_INFO("Error!");
                throw ecl::StandardException(LOC, e);
            }

            ROS_INFO("waiting for reading serial port...");

            ecl::Sleep(sec_timeout)();

        }
    }

    return true;
}

//停止电机
void ServoDriver::stop() {

    data_mutex_.lock();

    is_alive = false;

    unsigned char buf[32] = {0x00, 0x00, 0x00, 0x00, 0};

    if( !open_serial() ) {
        data_mutex_.unlock();
        return;
    }

    if(serial_.write(buf, 4) != 4){
        ROS_INFO("ServoDriver::stop: write fail!");
        data_mutex_.unlock();
        return;
    }

    if(serial_.read(buf, 2) != 2){
        ROS_INFO("ServoDriver::stop: read fail!");
        data_mutex_.unlock();
        return;
    }

    ROS_INFO("stop motor successed!");

    data_mutex_.unlock();

}


}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "servo_driver");
    ros::NodeHandle nh;

    robot::ServoDriver driver;

    driver.init();

    ros::spin();

    return 0;
}
