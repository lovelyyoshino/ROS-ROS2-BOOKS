#include <ros/ros.h>
#include "learning_topic/Student.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "student_publisher");

    ros::NodeHandle nh;

    // 创建一个Publisher，发布名为/person_info的topic，>消息类型为learning_topic::Person，队列长度10
    ros::Publisher student_info_pub = nh.advertise<learning_topic::Student>("/student_info", 1000);

    // 设置循环的频率
    ros::Rate loop_rate(1);

    int count = 0;
    while (ros::ok())
    {
        // 初始化learning_topic::Person类型的消息
    	learning_topic::Student student_msg;
		student_msg.name   = "Xiaoming";
		student_msg.age    = 18;
		student_msg.sex    = learning_topic::Student::male;
        student_msg.height = 178;
        student_msg.weight = 135;
        // 发布消息
		student_info_pub.publish(student_msg);

       	ROS_INFO("Publish Student Info: name:%s age:%d sex:%d height:%d weight:%d ", 
				  student_msg.name.c_str(), student_msg.age, student_msg.sex, student_msg.height, student_msg.weight);
        ROS_INFO("Publish times are:[%d]", ++count);
        // 按照循环频率延时
        loop_rate.sleep();
    }

    return 0;
}