#!/usr/bin/env python
# coding:utf-8

import rospy
from learning_topic.msg import Student

def student_publisher():
	# ROS节点初始化
    rospy.init_node('student_publisher', anonymous=True)

	# 创建一个Publisher，发布名为/person_info的topic，消息类型为learning_topic::Person，队列长度10
    student_info_pub = rospy.Publisher('/student_info', Student, queue_size=1000)

    rate = rospy.Rate(10) 

    count = 0
    while not rospy.is_shutdown():
        student_msg = Student()
        student_msg.name   = "Xiaoming";
        student_msg.age    = 18;
        student_msg.sex    = Student.male;
        student_msg.height = 178;
        student_msg.weight = 135;

        student_info_pub.publish(student_msg)
        rospy.loginfo("Publsh student messages[%s, %d, %d]", 
				student_msg.name, student_msg.age, student_msg.sex)
        count+=1
        rospy.loginfo("Publish times are:[%d]", count)
        rate.sleep()

if __name__ == '__main__':
    try:
        student_publisher()
    except rospy.ROSInterruptException:
        pass