#!/usr/bin/env python
# coding:utf-8

import rospy
from geometry_msgs.msg import Twist

def velocity_publisher():
	
    rospy.init_node('turtle1_vel_publisher', anonymous=True)

    turtle1_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1000)

    rate = rospy.Rate(10) 
    
    count = 0
    while not rospy.is_shutdown():

        vel_msg = Twist()
        vel_msg.linear.x = 1.0
        vel_msg.angular.z = 1.0

        turtle1_vel_pub.publish(vel_msg)
        rospy.loginfo("Publish turtle1 velocity msssage is：[%0.2f m/s, %0.2f rad/s]", vel_msg.linear.x, vel_msg.angular.z)  
        count+=1
        rospy.loginfo("Publish times are:[%d]", count)
        rate.sleep()

if __name__ == '__main__':
    try:
        velocity_publisher()
    except rospy.ROSInterruptException:
        pass