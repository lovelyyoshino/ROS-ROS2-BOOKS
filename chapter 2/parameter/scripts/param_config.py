#!/usr/bin/env python
# coding:utf-8

import sys
import rospy

def param_config():
	# ROS节点初始化
    rospy.init_node('param_config', anonymous=True)

	# 读取背景颜色参数
    red   = rospy.get_param('/turtlesim/background_r')
    green = rospy.get_param('/turtlesim/background_g')
    blue  = rospy.get_param('/turtlesim/background_b')

    rospy.loginfo("Get Backgroud Color[%d, %d, %d]", red, green, blue)

	# 设置背景颜色参数
    rospy.set_param("/turtlesim/background_r", 255);
    rospy.set_param("/turtlesim/background_g", 255);
    rospy.set_param("/turtlesim/background_b", 0);

    rospy.loginfo("Set Backgroud Color[255, 255, 0]");

	# 读取背景颜色参数
    red   = rospy.get_param('/turtlesim/background_r')
    green = rospy.get_param('/turtlesim/background_g')
    blue  = rospy.get_param('/turtlesim/background_b')

    rospy.loginfo("Get Backgroud Color[%d, %d, %d]", red, green, blue)

        # 删除背景颜色参数
    rospy.delete_param('/turtlesim/background_b')
     
    ifparam1 = rospy.has_param('/turtlesim/background_b')
    if(ifparam1):
       rospy.loginfo('/turtlesim/background_b exists')
    else:
       rospy.loginfo('/turtlesim/background_b does not exist')
        # 参数列表
    params = rospy.get_param_names()
    rospy.loginfo('param list: %s', params)

    
if __name__ == "__main__":
    param_config()
