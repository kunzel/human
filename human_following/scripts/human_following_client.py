#! /usr/bin/env python


import rospy
import actionlib
from human_following.msg import *
def following_client():

    rospy.init_node('navi', anonymous=False)

    client = actionlib.SimpleActionClient('following_Server', human_followingAction)

    rospy.loginfo('Waiting for Server') 
    client.wait_for_server(rospy.Duration(60))

    goal = human_followingGoal()

    goal.time = 60
    
    client.send_goal(goal)


if __name__ == '__main__':
    try:
        following_client() 

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
