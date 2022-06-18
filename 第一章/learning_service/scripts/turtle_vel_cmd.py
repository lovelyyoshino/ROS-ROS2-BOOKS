#!/usr/bin/env python
# coding:utf-8

import rospy
import threading
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse

pubflag = False;
turtle_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1000)

def commandCallback(req):
	global pubflag
	pubflag = bool(1-pubflag)

	rospy.loginfo("Publish turtle velocity command: %s", pubflag)
	
	return TriggerResponse(1, "Change turtle command state successfully")

def turtle_command_server():

    rospy.init_node('turtle_vel_cmd', anonymous=True)

    s = rospy.Service('/turtle_cmd', Trigger, commandCallback)
    
    rate = rospy.Rate(10) 

    vel_msg = Twist()
    vel_msg.linear.x = 1.0;
    vel_msg.angular.z = 1.0;

    print( "Ready to receive turtle command.")
    while 1:
        if pubflag:
            turtle_vel_pub.publish(vel_msg)
            print("The number of threads are: %d" %len(threading.enumerate()))
            rate.sleep()

    rospy.spin()

if __name__ == "__main__":
    turtle_command_server()