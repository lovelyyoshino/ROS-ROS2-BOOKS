#include <ros/ros.h>
#include <FrameSize.hpp>
#include <geometry_msgs/Twist.h>
#include <deque>

#include "Params.h"

#define V_ANGULAR 0.6
#define V_FORWARD 1
#define V_BACKWARD 0.5

#define ANGULAR_THRESHOLD 0.5
#define ANGULAR_LOWER_THRESHOLD (ANGULAR_THRESHOLD - 0.05)
#define ANGULAR_UPPER_THRESHOLD (ANGULAR_THRESHOLD + 0.05)

#define LINEAR_THRESHOLD 0.75
#define LINEAR_LOWER_THRESHOLD (LINEAR_THRESHOLD - 0.05)
#define LINEAR_UPPER_THRESHOLD (LINEAR_THRESHOLD + 0.05)

#define MAX_MEAN 5

bool backward = false;
double ticks = 0;
double total = 0;

std::deque<double> linear_mean(MAX_MEAN, 0), angular_mean(MAX_MEAN, 0);

void callback(cv_tesina::Params msg);

ros::Publisher pub;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "person_follower");

	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("commands", 1, callback);
	pub = nh.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);
//	pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

	ros::spin();

	return EXIT_SUCCESS;
}

void callback(cv_tesina::Params msg)
{
	angular_mean.pop_front();

	if(msg.x == -1)
	{
		angular_mean.push_back(0);
	}
	else if(msg.x < frameSize.width * ANGULAR_LOWER_THRESHOLD)
	{
		angular_mean.push_back(-((msg.x - (frameSize.width * ANGULAR_LOWER_THRESHOLD)) / (frameSize.width * ANGULAR_LOWER_THRESHOLD)) * V_ANGULAR);
	}
	else if(msg.x > frameSize.width * ANGULAR_UPPER_THRESHOLD)
	{
		angular_mean.push_back(-((msg.x - (frameSize.width * ANGULAR_UPPER_THRESHOLD)) / (frameSize.width * (1 - ANGULAR_UPPER_THRESHOLD))) * V_ANGULAR);
	}
	else
	{
		angular_mean.push_back(0);
	}

	if(msg.height == -1)
	{
		if(backward)
		{
			double precTick = ticks;

			ticks = (double) cv::getTickCount();

			double dT = (ticks - precTick) / cv::getTickFrequency();
			total += dT;
		}

		if(!backward || total >= 1)
		{
			backward = false;
			linear_mean.pop_front();
			linear_mean.push_back(0);
		}
	}
	else if(msg.height < frameSize.height * LINEAR_LOWER_THRESHOLD)
	{
		linear_mean.pop_front();
		linear_mean.push_back(-((msg.height - (frameSize.height * LINEAR_LOWER_THRESHOLD)) / (frameSize.height * (LINEAR_LOWER_THRESHOLD - 0.5))) * V_FORWARD);
		backward = false;
		total = 0;
	}
	else if(msg.height > frameSize.height * LINEAR_UPPER_THRESHOLD)
	{
		linear_mean.pop_front();
		linear_mean.push_back(-((msg.height - (frameSize.height * LINEAR_UPPER_THRESHOLD)) / (frameSize.height * (1 - LINEAR_UPPER_THRESHOLD))) * V_BACKWARD);
		backward = true;
		ticks = (double) cv::getTickCount();
		total = 0;
	}
	else
	{
		linear_mean.pop_front();
		linear_mean.push_back(0);
	}

	geometry_msgs::Twist twist;
	twist.angular.z = 0;
	twist.linear.x = 0;

	for(int i = 0; i < MAX_MEAN; i++)
	{
		twist.angular.z += angular_mean[i] / MAX_MEAN;
		twist.linear.x += linear_mean[i] / MAX_MEAN;
	}

	twist.angular.z = std::min((double)twist.angular.z, (double)V_ANGULAR);

	if(twist.linear.x > 0)
	{
		twist.linear.x = std::min((double)twist.linear.x, (double)V_FORWARD);
	}
	else if(twist.linear.x < 0)
	{
		twist.linear.x = std::max((double)twist.linear.x, (double)-V_BACKWARD);
	}

	pub.publish(twist);
}
