
#include "robot_driver/servo_driver.h"

ServoDriver::ServoDriver()
{

    start_vectory();

    is_alive = true;

    thread_.start(&ServoDriver::read_status, *this);

    ecl::Sleep(5)();

    data_mutex_.lock();

    is_alive = false;

    data_mutex_.unlock();

    ecl::Sleep(1)();

}

ServoDriver::~ServoDriver()
{
    thread_.join();
    serial_.close();
}

void ServoDriver::start_vectory(){

    data_mutex_.lock();

    unsigned char buf[32] = {0};

    if( !open_serial() ){
        data_mutex_.unlock();
        return;
    }

    //启动电机
    buf[0] = 0x00;
    buf[1] = 0x00;
    buf[2] = 0x01;
    buf[3] = 0x01;

    if( serial_.write(buf, 4) == 4 && serial_.read(buf, 2) == 2) {
        printf("行号:%d, 返回值: %x, %x\n\n", __LINE__, buf[0], buf[1]);
    } else {
        printf("行号:%d, 写入失败\n", __LINE__);
        data_mutex_.unlock();
        return;
    }

    //设置速度模式
    buf[0] = 0x02;
    buf[1] = 0x00;
    buf[2] = 0xc4;
    buf[3] = 0xc6;

    if( serial_.write(buf, 4) == 4 && serial_.read(buf, 2) == 2) {
        printf("行号:%d, 返回值: %x, %x\n\n", __LINE__, buf[0], buf[1]);
    } else {
        printf("行号:%d, 写入失败\n", __LINE__);
        data_mutex_.unlock();
        return;
    }

    //设置速度:-50rpm
    buf[0] = 0x06;
    buf[1] = 0x00;
    buf[2] = 0x44;
    buf[3] = 0x4a;

    if( serial_.write(buf, 4) == 4 && serial_.read(buf, 2) == 2) {
        printf("行号:%d, 返回值: %x, %x\n\n", __LINE__, buf[0], buf[1]);
    } else {
        printf("行号:%d, 写入失败\n", __LINE__);
        data_mutex_.unlock();
        return;
    }

    data_mutex_.unlock();

}

void ServoDriver::start_position(){


}

//读取状态
void ServoDriver::read_status() {
//四个数据为一帧 格式为: 地址 数据高八位 数据低八位 校验和(取低八位)
//80 00 81 01 E1 00 1D FE E2 00 0F F1 E4 00 00 E4 E6 00 00 E6 E7 00 00 E7 E8 00 00 E8 E9 00 00 E9

    int size = 32, i;  //一共32个char型
    unsigned char buf[size] = {0};

    ros::Rate rate(4);

    while(ros::ok() && open_serial() && is_alive) {

        data_mutex_.lock();

        //发送监控指令
        buf[0] = 0x80;
        buf[1] = 0x00;
        buf[2] = 0x80;

        if(serial_.write(buf, 3) == 3 && serial_.read(buf, size) == size){
            printf("行号:%d, 接收内容: ", __LINE__);
            for(i=0; i<size-1; i++){
                printf("%x, ", buf[i]);
            }
            printf("%x\n\n", buf[i]);

            parse_status(buf, size);

            printf("电压:%d, 电流:%f, 转速:%d\n", wheel_status_.voltage, (float)wheel_status_.electricity / 100, wheel_status_.speed);

            data_mutex_.unlock();
            rate.sleep();

            continue;
        }

        printf("行号:%d, 写入失败\n", __LINE__);

        data_mutex_.unlock();
        rate.sleep();
    }

    if(!is_alive){

        data_mutex_.lock();

        buf[0] = 0x00;
        buf[1] = 0x00;
        buf[2] = 0x00;
        buf[4] = 0x00;

        if( serial_.write(buf, 4) == 4 ) {

            ROS_INFO("run end!\n");

        }else{
            printf("行号:%d, 写入失败\n", __LINE__);
        }
        data_mutex_.unlock();
    }
}

//解析状态
void ServoDriver::parse_status(unsigned char* buf, int size)
{

    unsigned char a, b, c, d;

    data_.clear();

    for(int i=0; i<size; i+=4){

        a=buf[i+0], b=buf[i+1], c=buf[i+2], d=buf[i+3];

        printf("a:%x, b:%x, c:%x, d:%x\n", a, b, c, d);

        if( (a+b+c & 0xff) != d){//校验失败
            break;
        }

//        data_.push_back(a);
//        data_.push_back(b);
//        data_.push_back(c);
//        data_.push_back(d);

        switch(i/4){

            case 0:     //故障状态

                break;

            case 1:     //母线电压
                wheel_status_.voltage = (int)d;
                break;

            case 2:     //输出电流
                wheel_status_.electricity = (int)d;
                break;

            case 3:     //输出转速
                wheel_status_.speed = (int)d * 3000 >> 13;
                break;

            case 4:     //位置
                wheel_status_.pos = (int)d << 16;
                break;

            case 5:
                wheel_status_.pos = wheel_status_.pos + (int)d;
                break;

            case 6:     //位置反馈
                wheel_status_.pos_feedback = (int)d << 16;
                break;

            case 7:
                wheel_status_.pos_feedback = wheel_status_.pos + (int)d;
                break;
        }

    }
}

bool ServoDriver::open_serial()
{
    if(!serial_.open()){
        try{
            serial_.open("/dev/ttyUSB1", ecl::BaudRate_57600, ecl::DataBits_8, ecl::StopBits_1, ecl::NoParity);
            serial_.block(1000);
            ROS_INFO("open successed!");
        }
        catch (const ecl::StandardException &e)
        {
            if (e.flag() == ecl::NotFoundError) {
                ROS_INFO_STREAM("device does not (yet) available on this port, waiting...");
            } else if (e.flag() == ecl::OpenError) {
                ROS_INFO_STREAM("device failed to open, waiting... [" << std::string(e.what()) << "]");
            } else {
                // This is bad - some unknown error we're not handling! But at least throw and show what error we came across.
                throw ecl::StandardException(LOC, e);
            }
            return false;
        }
    }
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "servo_driver");
    ros::NodeHandle nh;

    ServoDriver driver;

    return 0;
}
