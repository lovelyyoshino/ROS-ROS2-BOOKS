#!/usr/bin/env python
# coding:utf-8

import sys

from requests import request
import rospy
from turtlesim.srv import Spawn

def turtle_spawn():
	# ROS节点初始化
    rospy.init_node('turtle_spawn')

	# 发现/spawn服务后，创建一个服务客户端，连接名为/spawn的service
    rospy.wait_for_service('/spawn')
    try:
        spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)

        srv = Spawn()
        srv.x     = 1.0;
        srv.y     = 1.0;
        srv.theta = 0.0;
        srv.name  = "turtle2";
        
        response = spawn_turtle(srv.x, srv.y, srv.theta, srv.name)

        print("Call service to spawn turtle [x:%0.6f, y:%0.6f, name:%s]" %(srv.x,srv.y,srv.name))
        print("Spawn turtle successfully, new turtle name:%s" %response.name)
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)

if __name__ == "__main__":
	turtle_spawn()
   
    