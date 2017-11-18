
#include <float.h>
#include <tf/tf.h>
#include <robot_node/robot_ros.h>

namespace robot
{

RobotRos::RobotRos(std::string& node_name) :
    name_(node_name),
    slot_chassis_joint_state_(&RobotRos::publishChassisJointState, *this)
{

}

/**
 * This will wait some time while robot internally closes its threads and destructs
 * itself.
 */
RobotRos::~RobotRos()
{
    ROS_INFO_STREAM("Robot : waiting for robot thread to finish [" << name_ << "].");
}

bool RobotRos::init(ros::NodeHandle &nh)
{

    ROS_INFO("RobotRos::init()!!!");

    advertiseTopics(nh);
    subscribeTopics(nh);

    /*********************
     ** Slots
     **********************/
    slot_chassis_joint_state_.connect(name_ + std::string("/chassis_joint_state"));

    /*********************
     ** 硬件所需参数
     **********************/
    Parameters parameters;

    parameters.sigslots_namespace = name_; // name is automatically picked up by device_nodelet parent.

    robot.init(parameters);

    //设置需要发布的关节状态消息的初始值
    joint_states_.name.push_back(parameters.left_wheel_joint);
    joint_states_.name.push_back(parameters.right_wheel_joint);
    joint_states_.name.push_back(parameters.ommi_wheel_joint);

    joint_states_.position.resize(3,0.0);
    joint_states_.velocity.resize(3,0.0);
    joint_states_.effort.resize(3,0.0);

    return true;
}

/**
 * This is a worker function that runs in a background thread initiated by
 * the nodelet. It gathers diagnostics information from the robot driver,
 * and broadcasts the results to the rest of the ros ecosystem.
 *
 * Note that the actual driver data is collected via the slot callbacks in this class.
 *
 * @return Bool : true/false if successfully updated or not (robot driver shutdown).
 */
bool RobotRos::update()
{
    //判断超时或停止
    //...


    //诊断信息update
    //...

    return true;
}


/**
 * Two groups of publishers, one required by turtlebot, the other for
 * robot esoterics.
 */
void RobotRos::advertiseTopics(ros::NodeHandle &nh)
{
    joint_state_publisher_ = nh.advertise <sensor_msgs::JointState>("/joint_states",100);
}

/**
 * Two groups of subscribers, one required by turtlebot, the other for
 * robot esoterics.
 */
void RobotRos::subscribeTopics(ros::NodeHandle &nh)
{
    //velocity_command_subscriber = nh.subscribe(std::string("commands/velocity"), 10, &RobotRos::subscribeVelocityCommand, this);
}


} // namespace robot
