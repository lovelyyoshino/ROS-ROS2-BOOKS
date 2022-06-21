#include <ros/ros.h> 
#include <geometry_msgs/Twist.h>

int count = 0;
void velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    ROS_INFO("Turtle vel: linear x:%0.6f, y:%0.6f ,z:%0.6f angular x:%0.6f, y:%0.6f ,z:%0.6f", msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);
    ROS_INFO("Subcriber times are:[%d]", ++::count);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle1_vel_subscriber");

    ros::NodeHandle nh;

    ros::Subscriber turtle1_vel_sub = nh.subscribe("/turtle1/cmd_vel", 1000, velCallback);
    
    ros::spin();

    return 0;
}
