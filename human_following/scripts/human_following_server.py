#! /usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from smach_ros import SimpleActionState
from human_following.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import *
from actionlib.msg import *
from nav_goals_msgs.srv import NavGoals
from geometry_msgs.msg import Polygon, Pose, PoseStamped
from tf.transformations import quaternion_about_axis, quaternion_multiply, euler_from_quaternion
import math
from scitos_ptu.msg import *
from sensor_msgs.msg import *
from actionlib_msgs.msg import *

class followingServer(object):
  # create messages that are used to publish feedback/result
  #_feedback = followingFeedback()
  #_result   = followingResult()

  def __init__(self):
    self.server = actionlib.SimpleActionServer('following_Server', human_followingAction, self.execute, False)
    self.server.start()
    
  def execute(self, goal):
    rospy.loginfo('Starting Service')
    timegoing = rospy.get_time()
    rospy.loginfo('%d %f',goal.time,timegoing)
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    with sm:
	class Wandering(smach.State):
	    def __init__(self):
		smach.State.__init__(self,outcomes=['succeeded','aborted','preempted'])
	    def execute(self,userdata):
		 
 		rospy.wait_for_service('nav_goals')
       		try:
            	    self.nav_goals = rospy.ServiceProxy('nav_goals', NavGoals)
        	except rospy.ServiceException, e:
            	    rospy.logerr("Service call failed: %s" % e)

		self.polygon = Polygon()
		self.num = 1
		self.radius = 0.5
		nav_goal = self.nav_goals(self.num,self.radius,self.polygon)


		client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    		client.wait_for_server(rospy.Duration(60))
    		goal = MoveBaseGoal()
   		goal.target_pose.header.frame_id = 'map'
 	 	goal.target_pose.header.stamp = rospy.Time.now()
    		goal.target_pose.pose = nav_goal.goals.poses[0]
    		#goal.target_pose.pose.position.y = -5.0;
    		#goal.target_pose.pose.orientation.w = 1.0;
		    		
		client.send_goal(goal)

		finished_within_time = client.wait_for_result(rospy.Duration(60))

		return 'succeeded'
	
	class Following(smach.State):
	    def __init__(self):
		smach.State.__init__(self,outcomes=['succeeded','aborted','preempted'])
		self.pub = rospy.Publisher('/b21/pose_tf', PoseStamped)
	    	rospy.Subscriber('/b21/pose', PoseStamped, self.tf_cb)
		rospy.Subscriber('/b21/pose_tf', PoseStamped, self.following_cb)
		rospy.Subscriber('/robot_pose', Pose, self.robot_pose_cb)
		self.current_pose = Pose()
		self.current_pose_tf = Pose()
		self.current_robot = Pose()

	    def execute(self,userdata):

		rospy.sleep(rospy.Duration(0.5))
		rospy.loginfo("Execute Following")
		move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    		move_client.wait_for_server(rospy.Duration(60))
    		move_goal = MoveBaseGoal()
   		move_goal.target_pose.header.frame_id = 'map'
 	 	move_goal.target_pose.header.stamp = rospy.Time.now()
    		
		
		xx = self.current_pose.position.x - self.current_robot.position.x
		yy = self.current_pose.position.y - self.current_robot.position.y
		dist = math.sqrt(xx * xx + yy * yy)		
		q = [self.current_robot.orientation.x, self.current_robot.orientation.y, self.current_robot.orientation.z, self.current_robot.orientation.w ]
		robot_angles = euler_from_quaternion(q,'rxyz')
		print robot_angles[2]
		human_angle = math.atan((yy)/(xx))
		if xx<0:
		    if yy>0:
			human_angle = human_angle + 3.14
		    else:
			human_angle = human_angle - 3.14
		print human_angle
		rot_angle = human_angle - robot_angles[2]
		if rot_angle < -3.14:
		    rot_angle = rot_angle + 6.28
		if rot_angle > 3.14:
		    rot_angle = rot_angle - 6.28
		rot_angle_degree = rot_angle / 6.28 * 360
		print rot_angle_degree

		if dist <= 3.0:
		    move_goal.target_pose.pose = self.current_robot
		    rotq = quaternion_about_axis(rot_angle, [0,0,1])
		    newq = quaternion_multiply(q,rotq) 
		    move_goal.target_pose.pose.orientation.x = newq[0]
		    move_goal.target_pose.pose.orientation.y = newq[1]
		    move_goal.target_pose.pose.orientation.z = newq[2]
		    move_goal.target_pose.pose.orientation.w = newq[3]
		else:
	 	    move_goal.target_pose.pose = self.current_pose_tf		
		move_client.send_goal(move_goal)
		finished_within_time = move_client.wait_for_result(rospy.Duration(0.5))
		
		rospy.loginfo("Finish Following")

		#pan_client = actionlib.SimpleActionClient('SetPTUState', PtuGotoAction)
	        #pan_client.wait_for_server(rospy.Duration(60))
	        #pan_goal = PtuGotoGoal() 
	        #pan_goal.pan = 0
	        #pan_goal.tilt = 0
	        #pan_goal.pan_vel = 1
	        #pan_goal.tilt_vel = 1

		
		#pan_goal.pan = robot_angles[2] - human_angle
		#if pan_goal.pan < -3.14:
		#    pan_goal.pan = pan_goal.pan + 6.28
		#if pan_goal.pan > 3.14:
		#    pan_goal.pan = pan_goal.pan - 6.28
		#pan_goal.pan = pan_goal.pan / 6.28 * 360
		#print pan_goal.pan
	        #pan_client.send_goal(pan_goal)
	        #pan_client.wait_for_result(rospy.Duration(0.5)) 
 
		rospy.loginfo("New Position")
 		return 'succeeded'
	
	  
	    def tf_cb(self, data):
		self.current_pose.position.x = -data.pose.position.y - 4.1
		self.current_pose.position.y = data.pose.position.x - 3.75
		self.current_pose.position.z = data.pose.position.z
		oldq = [data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w]
		rotq = quaternion_about_axis(1.57, [0,0,1])
		newq = quaternion_multiply(oldq,rotq) 
		self.current_pose.orientation.x = newq[0]
		self.current_pose.orientation.y = newq[1]
		self.current_pose.orientation.z = newq[2]
		self.current_pose.orientation.w = newq[3]
		pose_stamped = PoseStamped()
		pose_stamped.header = data.header
		pose_stamped.pose = self.current_pose
		self.pub.publish(pose_stamped)
	
	    def following_cb(self, data):
		self.current_pose_tf = data.pose
	
	    def robot_pose_cb(self, data):
		self.current_robot = data

	
	mygoal = MoveBaseGoal()
        mygoal.target_pose.header.frame_id = 'map'
        mygoal.target_pose.header.stamp = rospy.Time.now()
        mygoal.target_pose.pose.position.x = 3.0;
        mygoal.target_pose.pose.orientation.w = 1.0;

        smach.StateMachine.add('Start',
                      SimpleActionState('move_base',
                                        MoveBaseAction,
                                        goal=mygoal),
                      transitions={'succeeded':'Following'})
	smach.StateMachine.add('Wandering',Wandering(),
				transitions={'succeeded':'Wandering'})
	smach.StateMachine.add('Following',Following(),
				transitions={'succeeded':'Following',
					     'aborted':'Wandering'})
        
    #self._feedback.trackstatus = 0
    rospy.loginfo('Executing following')
    outcome = sm.execute()

def main():
    rospy.init_node('following')
    server = followingServer()
    rospy.spin()
      
if __name__ == '__main__':
    main()


