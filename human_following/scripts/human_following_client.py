#! /usr/bin/env python

import rospy
import actionlib
from human_msgs.msg import human_followingAction, human_followingGoal


def following_client():

    rospy.init_node('human_following_client', anonymous=False)

    client = actionlib.SimpleActionClient(
        'following_Server',
        human_followingAction)

    rospy.loginfo('Waiting for Server')
    client.wait_for_server(rospy.Duration(60))

    goal = human_followingGoal()

    goal.time = 300
    goal.distance = 2
    client.send_goal(goal)


if __name__ == '__main__':
    try:
        following_client()
        rospy.spin()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
