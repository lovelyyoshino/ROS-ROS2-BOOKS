#include<ros/ros.h>
#include<learning_service/Add.h>
#include<cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc ,argv, "add_client");

    if (argc != 3)
    {
        ROS_ERROR("usage: add two number X Y");
        return 1;
    }
    ros::NodeHandle nh;

    ros::ServiceClient add_client = nh.serviceClient<learning_service::Add>("add_service");

    learning_service::Add srv;
    srv.request.a = atof(argv[1]);
    srv.request.b = atof(argv[2]);

    if(add_client.call(srv))
    {
        ROS_INFO("sum is: %0.2f", srv.response.sum);
    }
    else
    {
        ROS_ERROR("Failed to call service add_service");
        return 1;
    }

    return 0;
}