
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ecl/threads/thread.hpp>


namespace robot
{

class TestNodelet : public nodelet::Nodelet
{
public:
    TestNodelet() {}

    ~TestNodelet()
    {
        NODELET_DEBUG_STREAM("Robot : waiting for update thread to finish.");
        update_thread_.join();
    }

    virtual void onInit()
    {
        std::string nodelet_name = this->getName();

        update_thread_.start(&TestNodelet::update, *this);

    }
private:
    void update()
    {
        ros::Rate spin_rate(1);
        int i = 0;
        while (ros::ok())
        {
            i++;
            NODELET_DEBUG_STREAM("debug: " << i);
            spin_rate.sleep();
        }
    }
    
    ecl::Thread update_thread_;
};

} // namespace robot

PLUGINLIB_EXPORT_CLASS(robot::TestNodelet, nodelet::Nodelet)
