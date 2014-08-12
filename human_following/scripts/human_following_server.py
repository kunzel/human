#! /usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import math
from smach_ros import SimpleActionState
from human_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import *
from actionlib.msg import *
from nav_goals_msgs.srv import NavGoals
from geometry_msgs.msg import Polygon, Pose, PoseStamped, Point
from tf.transformations import quaternion_about_axis, quaternion_multiply, euler_from_quaternion
from scitos_ptu.msg import *
from sensor_msgs.msg import *
from actionlib_msgs.msg import *
from strands_perception_people_msgs.msg import *
from human_following_states import *

class followingServer(object):
    # create messages that are used to publish feedback/result
    #_feedback = followingFeedback()
    #_result   = followingResult()

    def __init__(self):
        self.server = actionlib.SimpleActionServer('following_Server', human_followingAction, self.execute, False)
        self.server.start()
    
    def execute(self, goal):
        rospy.loginfo('Starting Service')
    	timestart = rospy.get_time()
  	timegoing = rospy.get_param('time',300)


	self.agent = FollowingSM()
        smach_thread = threading.Thread(target = self.agent.executing_sm)
        smach_thread.start()

	while not rospy.is_shutdown():
	    if rospy.get_time() - timestart > timegoing:
		#smach_thread.stop()	
		rospy.signal_shutdown('Timeout')		
		return


def main():
    rospy.init_node('human_following_server')
    server = followingServer()
    rospy.spin()
      
if __name__ == '__main__':
    main()


