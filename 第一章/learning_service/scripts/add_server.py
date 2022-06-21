#!/usr/bin/env python
# coding:utf-8

import rospy
from learning_service.srv import Add, AddResponse

def addCallback(req):
    print("the request and resutle is: %0.2f + %0.2f = %0.2f"%(req.a, req.b, (req.a + req.b)))
    return AddResponse(req.a + req.b)

def add_server():
    rospy.init_node('add_server')
    s = rospy.Service('add_service', Add, addCallback)
    print("Ready to add two number.")
    rospy.spin()

if __name__ == "__main__":
    add_server()

