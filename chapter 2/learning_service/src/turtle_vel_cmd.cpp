#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>

bool pubflag = false;

// service回调函数，输入参数req，输出参数res
bool commandCallback(std_srvs::Trigger::Request  &req,
         			std_srvs::Trigger::Response &res)
{
	pubflag = !pubflag;

    // 显示请求数据
    ROS_INFO("Publish turtle velocity command: %s", pubflag==true?"Yes":"No");

	// 设置反馈数据
	res.success = true;
	res.message = "Change turtle pubflag value successfully";

    return true;
}

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "turtle_vel_cmd");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 创建一个名为/turtle_command的server，注册回调函数commandCallback
    ros::ServiceServer cmd_service = nh.advertiseService("/turtle_cmd", commandCallback);

	// 创建一个Publisher，发布名为/turtle1/cmd_vel的topic，消息类型为geometry_msgs::Twist，队列长度10
	ros::Publisher turtle_vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

    // 循环等待回调函数
    ROS_INFO("Ready to receive turtle velocity command.");

	geometry_msgs::Twist vel_msg;
	vel_msg.linear.x = 1.0;
	vel_msg.angular.z = 1.0;
	
	// 设置循环的频率
	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		// 查看一次回调函数队列
    	ros::spinOnce();
		
		// 如果标志为true，则发布速度指令
		if(pubflag)
		{
			turtle_vel_pub.publish(vel_msg);
		}

		//按照循环频率延时
	    loop_rate.sleep();
	}

    return 0;
}