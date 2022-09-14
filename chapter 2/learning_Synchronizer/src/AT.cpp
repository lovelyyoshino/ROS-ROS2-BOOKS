#include "ros/ros.h"    
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include "learning_Synchronizer/Person.h"

#include <iostream>

using namespace std;
using namespace message_filters;

learning_Synchronizer::Person syn_pub1;
learning_Synchronizer::Person syn_pub2;

void Syncallback(const learning_Synchronizer::PersonConstPtr& pub1,const learning_Synchronizer::PersonConstPtr& pub2)
{
    cout << "\033[1;32m Syn! \033[0m" << endl;
    syn_pub1 = *pub1;
    syn_pub2 = *pub2;
    cout << "pub1's timestamp : " << syn_pub1.header.stamp << endl;
    cout << "pub2's timestamp : " << syn_pub2.header.stamp << endl;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ET");
    ros::NodeHandle nh;

    cout << "\033[1;31m hw1! \033[0m" << endl;

    // 建立需要订阅的消息对应的订阅器
    message_filters::Subscriber<learning_Synchronizer::Person> pub1_sub(nh, "chatter1", 1);
    message_filters::Subscriber<learning_Synchronizer::Person> pub2_sub(nh, "chatter2", 1);
    
    typedef sync_policies::ApproximateTime<learning_Synchronizer::Person, learning_Synchronizer::Person> MySyncPolicy; 

    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pub1_sub, pub2_sub); //queue size=10
    sync.registerCallback(boost::bind(&Syncallback, _1, _2));

    ros::spin();
    return 0;
}