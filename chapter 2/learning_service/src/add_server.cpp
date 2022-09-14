#include<ros/ros.h>
#include<learning_service/Add.h>

bool addCallback(learning_service::Add::Request  &req,
                 learning_service::Add::Response &res)
{
    res.sum = req.a + req.b;

    ROS_INFO("request: x=%0.2f, y=%0.2f", req.a, req.b);
    ROS_INFO("the result is: %0.2f", res.sum);

    return true;
}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "add_server");

        ros::NodeHandle nh;

        ros::ServiceServer add_service = nh.advertiseService("add_service", addCallback);

        ROS_INFO("Ready to add two number.");

        ros::spin();

        return 0;
}