#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "learning_action/CountAction.h"

typedef actionlib::SimpleActionClient<learning_action::CountAction> Client;

class CountActionClient1
{
    private:
        Client client;

        void doneCb(const actionlib::SimpleClientGoalState &state,
               const learning_action::CountResultConstPtr &result)
               {
                   ROS_INFO("Finished in state %s",state.toString().c_str());
                   ROS_INFO("Total counted number: %i", result->finish);
                   ros::shutdown();
               }

        void activeCb()
        {
            ROS_INFO("Goal has been active");
        }

        void feedbackCb(const learning_action::CountFeedbackConstPtr &feedback)
        {
            ROS_INFO("The progress is:%0.2f%s", feedback->percent_complete,"%");
        }
    public:
        CountActionClient1(const std::string client_name, bool flag = true) :
        client(client_name, flag)
        {
        }
        void Start()
        {
            client.waitForServer();
            learning_action::CountGoal goal;
            goal.goal_num = 1;
            client.sendGoal(goal,
             boost::bind(&CountActionClient1::doneCb, this, _1, _2),
             boost::bind(&CountActionClient1::activeCb, this),
             boost::bind(&CountActionClient1::feedbackCb, this, _1));
             client.waitForResult(ros::Duration(30.0));
             if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
             {
                ROS_INFO("The number are counting");
             }
             else
             {
                 ROS_INFO("Cancel the goal");
                 client.cancelAllGoals();

             }
             printf("Current State: %s\n", client.getState().toString().c_str());
             
        }
          
};   

int main(int argc, char **argv)
{

    ros::init(argc, argv, "count_client1");

    ros::NodeHandle nh;

    CountActionClient1 client("count", true);

    client.Start();

    ros::spin();

    return 0;
}
