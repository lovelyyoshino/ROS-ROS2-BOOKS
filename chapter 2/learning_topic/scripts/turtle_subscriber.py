#!/usr/bin/env python
# coding:utf-8

import rospy
from geometry_msgs.msg import Twist

global count 
count = 0

def velCallback(msg):
    rospy.loginfo("Turtle vel: linear x:%0.6f, y:%0.6f ,z:%0.6f angular x:%0.6f, y:%0.6f ,z:%0.6f", msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z)
    count+=1
    rospy.loginfo("Subscribe times are:[%d]", count)    

def vel_subscriber():
    rospy.init_node('turtle1_vel_subscriber', anonymous=True)

    rospy.Subscriber("/turtle1/cmd_vel", Twist, velCallback)

    rospy.spin()
   
if __name__ == '__main__':
    vel_subscriber()

