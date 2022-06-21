#!/usr/bin/env python
# coding:utf-8

import rospy
from learning_topic.msg import Student

count = 0

def studentInfoCallback(msg):
    rospy.loginfo("Subscribe Student Info: name:%s age:%d sex:%d height:%d weight:%d ", 
			 msg.name, msg.age, msg.sex, msg.height, msg.weight)
    global count 
    count+=1
    rospy.loginfo("Subscribe times are:[%d]", count)  
def student_subscriber():
	# ROS节点初始化
    rospy.init_node('student_subscriber', anonymous=True)

	# 创建一个Subscriber，订阅名为/person_info的topic，注册回调函数personInfoCallback
    rospy.Subscriber("/student_info", Student, studentInfoCallback)

	# 循环等待回调函数
    rospy.spin()

if __name__ == '__main__':
    student_subscriber()