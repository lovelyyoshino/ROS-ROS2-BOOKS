#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "learning_action/CountAction.h"

typedef actionlib::SimpleActionServer<learning_action::CountAction> Server;

class CountActionServer
{
  private:
  learning_action::CountFeedback feedback;
  learning_action::CountResult   result;
  public:
  Server server;
  CountActionServer(ros::NodeHandle nh):
  server(nh,"count",boost::bind(&CountActionServer::ExecuteCb,this,_1),false)
  {
    server.registerPreemptCallback(boost::bind(&CountActionServer::preemptCb,this));
  }

  void Start()
  {
    server.start();
  }

  void ExecuteCb(const learning_action::CountGoalConstPtr& goal)
  {
    ros::Rate rate(1);
    ROS_INFO("Get the goal,the goal value is: %d", goal->goal_num);
    // learning_action::CountFeedback feedback;
    // learning_action::CountResult   result;
    int count_num  = 1;
    int finish_num = 10;
    for (; count_num <= finish_num; count_num++)
    {
      ROS_INFO("Counting the number: %d",count_num);
      feedback.percent_complete = (float)count_num/finish_num*100;
      server.publishFeedback(feedback);
      rate.sleep();
      result.finish = count_num;

    }
    // result.finish = count_num;

    if(server.isActive())
    server.setSucceeded(result);
    
  }

  void preemptCb()
  {
    if(server.isActive())
    server.setPreempted();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "count_server");
  ros::NodeHandle nh;
  CountActionServer countserver(nh);
  countserver.Start();
  ros::spin();
  return 0;
}

