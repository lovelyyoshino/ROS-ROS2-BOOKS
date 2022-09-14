#include "ros/ros.h"
#include <string>

int main(int argc, char **argv)
{
	int red, green, blue;

    // ROS节点初始化
    ros::init(argc, argv, "param_config");

    // 创建节点句柄
    ros::NodeHandle nh;

        // 读取背景颜色参数
        bool ifget1 = ros::param::get("/turtlesim/background_r", red);
	    bool ifget2 = nh.getParam("/turtlesim/background_g", green);
	    bool ifget3 = nh.param("/turtlesim/background_b", blue, 100);

        if(ifget1&&ifget2&&ifget3)
	     ROS_INFO("Get Backgroud Color[%d, %d, %d]", red, green, blue);
         else
             ROS_WARN("Didn't retrieve all param successfully");
	// 设置背景颜色参数
	ros::param::set("/turtlesim/background_r", 255);
	ros::param::set("/turtlesim/background_g", 255);
        nh.setParam("/turtlesim/background_b",255);

	ROS_INFO("Set Backgroud Color[255, 255, 255]");

       // 读取背景颜色参数
	 if(ros::param::get("/turtlesim/background_r", red))
	       ROS_INFO("background_r = %d", red);
         else
                ROS_WARN("Didn't get param successfully");
        if(ros::param::get("/turtlesim/background_r", green))
	   ROS_INFO("background_g = %d", green);
        else
            ROS_WARN("Didn't get param successfully");
        if(ros::param::get("/turtlesim/background_r", blue))
	   ROS_INFO("background_b = %d", blue);
        else
            ROS_WARN("Didn't get param successfully");
        // 删除背景颜色参数
        bool ifdeleted1 = nh.deleteParam("/turtlesim/background_r");
        bool ifdeleted2 = ros::param::del("/turtlesim/background_b");
        if(ifdeleted1)
		ROS_INFO("/turtlesim/background_r deleted");
	else
		ROS_INFO("/turtlesim/background_r not deleted");
        if(ifdeleted2)
		ROS_INFO("/turtlesim/background_b deleted");
	else
		ROS_INFO("/turtlesim/background_b not deleted");
        // 查看背景颜色参数
        bool ifparam1 = nh.hasParam("/turtlesim/background_r");
        bool ifparam2 = ros::param::has("/turtlesim/background_b");
        if(ifparam1) 
		ROS_INFO("background_r exists");
	else
		ROS_INFO("background_r doesn't exist");
         if(ifparam2) 
		ROS_INFO("background_b exists");
	else
		ROS_INFO("background_b doesn't exist");
      

	sleep(1);

    return 0;
}
