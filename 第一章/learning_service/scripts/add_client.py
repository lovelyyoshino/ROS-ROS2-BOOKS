#!/usr/bin/env python
# coding:utf-8

import sys
import rospy
from learning_service.srv import Add

def add_client(x, y):
    rospy.init_node('add_client')
    rospy.wait_for_service('add_service')
    try:
        add_two_ints = rospy.ServiceProxy('add_service', Add)
        response = add_two_ints(x, y)
        print("sum is: %0.2f" %response.sum)
    except rospy.ServiceException as e:
        print("Failed to call service add_service: %s"%e)

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        add_client(x, y)
    else:
        print("usage: add two number X Y")
        sys.exit(1)