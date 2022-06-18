#include <ros/ros.h> 
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtle1_vel_publisher");

	ros::NodeHandle nh;

	ros::Publisher turtle1_vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

	ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok())
	{
		geometry_msgs::Twist vel_msg;
		vel_msg.linear.x = 1.0;
		vel_msg.angular.z = 1.0;

		turtle1_vel_pub.publish(vel_msg);
		ROS_INFO("Publish turtle1 velocity msssage is：[%0.2f m/s, %0.2f rad/s]", vel_msg.linear.x, vel_msg.angular.z);
        ROS_INFO("Publish times are:[%d]",++count);
        
	    loop_rate.sleep();
	}

	return 0;
}