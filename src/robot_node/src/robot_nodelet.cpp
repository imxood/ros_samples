
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ecl/threads/thread.hpp>
#include "robot_node/robot_ros.h"


namespace robot
{

class RobotNodelet : public nodelet::Nodelet
{
public:
    RobotNodelet() : shutdown_requested_(false) {}

    ~RobotNodelet()
    {
        NODELET_DEBUG_STREAM("Robot : waiting for update thread to finish.");
        shutdown_requested_ = true;
        update_thread_.join();
    }

    virtual void onInit()
    {
        std::string nodelet_name = this->getName();

        robot_.reset(new RobotRos(nodelet_name));

        if (robot_->init(this->getNodeHandle()))
        {
            update_thread_.start(&RobotNodelet::update, *this);
            NODELET_INFO_STREAM("Robot : initialised.");
        }
        else
        {
            NODELET_ERROR_STREAM("Robot : could not initialise! Please restart.");
        }
    }
private:
    void update()
    {
        ros::Rate spin_rate(10);
        while (!shutdown_requested_ && ros::ok() && robot_->update())
        {
            spin_rate.sleep();
        }
    }

    boost::shared_ptr<RobotRos> robot_;
    ecl::Thread update_thread_;
    bool shutdown_requested_;
};

} // namespace robot

PLUGINLIB_EXPORT_CLASS(robot::RobotNodelet, nodelet::Nodelet)
