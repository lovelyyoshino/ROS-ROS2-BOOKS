#!/usr/bin/env python
# coding:utf-8

import rospy
import actionlib
from learning_action.msg import CountAction, CountGoal

class CountActionClient:
    def __init__(self) :
        self.client = actionlib.SimpleActionClient("count", CountAction)

    def doneCb(self, state, result) :    
        rospy.loginfo("Finished in state %d",state)
        rospy.loginfo("Total counted number: %d", result.finish)
        rospy.is_shutdown()
    
    def activeCb(self) :
        rospy.loginfo("Goal has been active")
        
    def feedbackCb(self, feedback) :
        rospy.loginfo("The progress is:%0.2f%%", feedback.percent_complete)
    
    def Start(self) :
        self.client.wait_for_server()
        goal = CountGoal()
        goal.goal_num = 1
        self.client.send_goal(goal, 
        self.doneCb, 
        self.activeCb, 
        self.feedbackCb)
        self.client.wait_for_result(rospy.Duration(30.0))
        if(self.client.get_state() ==actionlib.GoalStatus.SUCCEEDED) :
            rospy.loginfo("The number are counted")
        else :
            rospy.loginfo("Cancel the goal")
            self.client.cancel_all_goals()
        rospy.loginfo("Current State: %d\n", self.client.get_state())

if __name__ == '__main__' :
    rospy.init_node("count_client")
    countclient = CountActionClient()
    countclient.Start()
    rospy.spin()

