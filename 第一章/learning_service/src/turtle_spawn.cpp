#include <ros/ros.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "turtle_spawn");

	ros::NodeHandle nh;

	ros::service::waitForService("/spawn");
	ros::ServiceClient spawn_turtle = nh.serviceClient<turtlesim::Spawn>("/spawn");

	turtlesim::Spawn srv;
	srv.request.x = 1.0;
	srv.request.y = 1.0;
	srv.request.name = "turtle2";

	ROS_INFO("Call service to spwan turtle [x:%0.6f, y:%0.6f, name:%s]", 
			 srv.request.x, srv.request.y, srv.request.name.c_str());

	spawn_turtle.call(srv); 

	ROS_INFO("Spwan turtle successfully, new turtle name:%s", srv.response.name.c_str());

	return 0;
};
