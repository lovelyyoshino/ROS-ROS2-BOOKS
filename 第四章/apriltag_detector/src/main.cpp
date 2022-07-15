#include "apriltag_detector.cpp"
#include <ros/ros.h>

int main(int argc,char** argv)
{
	ros::init(argc,argv,"tag_detector");

	ros::NodeHandle nh;
	TagDetector tag(nh);

	ros::spin();

	return 0;
}