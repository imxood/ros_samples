#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_tutorials/FibonacciAction.h>

using namespace actionlib_tutorials;
typedef actionlib::SimpleActionClient<FibonacciAction> Client;

class MyNode
{
public:
  MyNode() : ac("fibonacci", true)
  {
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");
  }

  void doStuff(int order)
  {
    FibonacciGoal goal;
    goal.order = order;

    // Need boost::bind to pass in the 'this' pointer
    ac.sendGoal(goal,
                boost::bind(&MyNode::doneCb, this, _1, _2),
                boost::bind(&MyNode::ActionActiveCB, this),
                boost::bind(&MyNode::ActionFeedbackCB, this, _1));

  }

  void doneCb(const actionlib::SimpleClientGoalState& state,
              const FibonacciResultConstPtr& result)
  {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("Answer: %i", result->sequence.back());
    ros::shutdown();
  }

  void ActionActiveCB() {
      ROS_INFO("ActionActiveCB!");
  }

  // 接收服务器的反馈信息
  void ActionFeedbackCB(const actionlib_tutorials::FibonacciFeedbackConstPtr& feedback) {
      ROS_INFO("ActionFeedbackCB!");
  }

private:
  Client ac;
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci_class_client");
  MyNode my_node;
  my_node.doStuff(10);
  ros::spin();
  return 0;
}
