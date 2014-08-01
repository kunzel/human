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
		smach.State.__init__(self,
				     outcomes=['succeeded','aborted','preempted']
				    )
	    def execute(self,userdata):
		 
 		rospy.wait_for_service('nav_goals')
       		try:
            	    self.nav_goals = rospy.ServiceProxy('nav_goals', NavGoals)
        	except rospy.ServiceException, e:
            	    rospy.logerr("Service call failed: %s" % e)

		self.polygon = Polygon()
		self.num = 1
		self.radius = 0.5
		self.polygon.points = []
		p = Point()
		p.x = 5.0
		p.y = 5.0
		self.polygon.points.append(p)
		p = Point()
		p.x = -5.0
		p.y = 5.0
		self.polygon.points.append(p)
		p = Point()
		p.x = 5.0
		p.y = -18.0
		self.polygon.points.append(p)
		p = Point()
		p.x = -5.0
		p.y = -18.0
		self.polygon.points.append(p)
		#print  self.polygon
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

		finished_within_time = client.wait_for_result(rospy.Duration(1))

		return 'succeeded'
	
	class Searching(smach.State):
	    def __init__(self):
		smach.State.__init__(self,
				     outcomes=['succeeded','aborted','preempted'],
				     output_keys=['current_robot','current_pose_tf']
				     )
		self.pub = rospy.Publisher('/b21/pose_tf', PoseStamped)
	    	rospy.Subscriber('/b21/pose', PoseStamped, self.tf_cb)
		rospy.Subscriber('/b21/pose_tf', PoseStamped, self.following_cb)
		rospy.Subscriber('/robot_pose', Pose, self.robot_pose_cb)
		self.current_robot = Pose()
		self.current_pose = Pose()
		self.current_pose_tf = Pose()
		self.is_received = bool()
		#self.is_received = True

	    def execute(self,userdata):
		count = 0
		while(1):
		    count = count + 1
		    if self.is_received:
		    	userdata.current_robot = self.current_robot
		   	userdata.current_pose_tf = self.current_pose_tf
			return 'succeeded'
		    if count >= 10000:
			break
		rospy.sleep(rospy.Duration(0.5))	
		
		return 'aborted'

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
		if (data.pose.position.x == 0) and (data.pose.position.y == 0) and (data.pose.position.z == 0):
		    self.is_received = False
		else:
		    self.is_received = True

	    def robot_pose_cb(self, data):
		self.current_robot = data

	class LocalSearching(smach.State):
	    def __init__(self):
		smach.State.__init__(self,outcomes=['succeeded','aborted','preempted'])
	    def execute(self,userdata):
		angles = [90, 60, 30, 0, -30, -60, -90]
		pan_client = actionlib.SimpleActionClient('SetPTUState', PtuGotoAction)
		rospy.loginfo("Waiting for Pantilt action server...")		
	        pan_client.wait_for_server(rospy.Duration(60))
 		rospy.loginfo("Connected to pan tilt server")
  		pan_goal = PtuGotoGoal()
		for angle in angles:		 
	            pan_goal.pan = angle
	            pan_goal.tilt = 0
	            pan_goal.pan_vel = 1
	            pan_goal.tilt_vel = 1
		    pan_client.send_goal(pan_goal)
	            pan_client.wait_for_result(rospy.Duration(0.1)) 
		    rospy.sleep(rospy.Duration(0.3))
 		return 'succeeded'

	class Following(smach.State):
	    def __init__(self):
		smach.State.__init__(self,
				     outcomes=['succeeded','aborted','preempted'],
				     input_keys = ['current_robot','current_pose_tf'])
		#self.pub = rospy.Publisher('/b21/pose_tf', PoseStamped)
	    	#rospy.Subscriber('/b21/pose', PoseStamped, self.tf_cb)
		#rospy.Subscriber('/b21/pose_tf', PoseStamped, self.following_cb)
		#rospy.Subscriber('/robot_pose', Pose, self.robot_pose_cb)
		#self.current_pose = Pose()
		#self.current_pose_tf = Pose()
		#self.current_robot = Pose()

	    def execute(self,userdata):

		rospy.sleep(rospy.Duration(0.3))
		move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		rospy.loginfo("Waiting for move_base action server...")
    		move_client.wait_for_server(rospy.Duration(60))
		rospy.loginfo("Connected to move base server")
    		move_goal = MoveBaseGoal()
   		move_goal.target_pose.header.frame_id = 'map'
 	 	move_goal.target_pose.header.stamp = rospy.Time.now()
    		
		
		xx = userdata.current_pose_tf.position.x - userdata.current_robot.position.x
		yy = userdata.current_pose_tf.position.y - userdata.current_robot.position.y
		dist = math.sqrt(xx * xx + yy * yy)		
		q = [userdata.current_robot.orientation.x, userdata.current_robot.orientation.y, userdata.current_robot.orientation.z, userdata.current_robot.orientation.w ]
		robot_angles = euler_from_quaternion(q,'rxyz')
		#print robot_angles[2]
		human_angle = math.atan((yy)/(xx))
		if xx<0:
		    if yy>0:
			human_angle = human_angle + 3.14
		    else:
			human_angle = human_angle - 3.14
		#print human_angle
		rot_angle = human_angle - robot_angles[2]
		if rot_angle < -3.14:
		    rot_angle = rot_angle + 6.28
		if rot_angle > 3.14:
		    rot_angle = rot_angle - 6.28
		rot_angle_degree = rot_angle / 6.28 * 360
		#print rot_angle_degree

		if dist <= 3.0:
		    move_goal.target_pose.pose = userdata.current_robot
		    rotq = quaternion_about_axis(rot_angle, [0,0,1])
		    newq = quaternion_multiply(q,rotq) 
		    move_goal.target_pose.pose.orientation.x = newq[0]
		    move_goal.target_pose.pose.orientation.y = newq[1]
		    move_goal.target_pose.pose.orientation.z = newq[2]
		    move_goal.target_pose.pose.orientation.w = newq[3]
		else:
	 	    move_goal.target_pose.pose = userdata.current_pose_tf		
		move_client.send_goal(move_goal)
		finished_within_time = move_client.wait_for_result(rospy.Duration(0.1))
		
		pan_client = actionlib.SimpleActionClient('SetPTUState', PtuGotoAction)
		rospy.loginfo("Waiting for Pantilt action server...")		
	        pan_client.wait_for_server(rospy.Duration(60))
 		rospy.loginfo("Connected to pan tilt server")


		xx = userdata.current_pose_tf.position.x - userdata.current_robot.position.x
		yy = userdata.current_pose_tf.position.y - userdata.current_robot.position.y
		dist = math.sqrt(xx * xx + yy * yy)		
		q = [userdata.current_robot.orientation.x, userdata.current_robot.orientation.y, userdata.current_robot.orientation.z, userdata.current_robot.orientation.w ]
		robot_angles = euler_from_quaternion(q,'rxyz')
		#print robot_angles[2]
		human_angle = math.atan((yy)/(xx))
		if xx<0:
		    if yy>0:
			human_angle = human_angle + 3.14
		    else:
			human_angle = human_angle - 3.14
		#print human_angle
		rot_angle = human_angle - robot_angles[2]
		if rot_angle < -3.14:
		    rot_angle = rot_angle + 6.28
		if rot_angle > 3.14:
		    rot_angle = rot_angle - 6.28
		rot_angle_degree = rot_angle / 6.28 * 360
		#print rot_angle_degree

	        pan_goal = PtuGotoGoal() 
	        pan_goal.pan = rot_angle_degree
	        pan_goal.tilt = 0
	        pan_goal.pan_vel = 1
	        pan_goal.tilt_vel = 1
		if dist <=2.0:
		    pan_goal.tilt = 10

	        pan_client.send_goal(pan_goal)
	        pan_client.wait_for_result(rospy.Duration(0.1)) 
 
		rospy.loginfo("New Position")
 		return 'succeeded'
	
	  
	    #def tf_cb(self, data):
		#self.current_pose.position.x = -data.pose.position.y - 4.1
		#self.current_pose.position.y = data.pose.position.x - 3.75
		#self.current_pose.position.z = data.pose.position.z
		#oldq = [data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w]
		#rotq = quaternion_about_axis(1.57, [0,0,1])
		#newq = quaternion_multiply(oldq,rotq) 
		#self.current_pose.orientation.x = newq[0]
		#self.current_pose.orientation.y = newq[1]
		#self.current_pose.orientation.z = newq[2]
		#self.current_pose.orientation.w = newq[3]
		#pose_stamped = PoseStamped()
		#pose_stamped.header = data.header
		#pose_stamped.pose = self.current_pose
		#self.pub.publish(pose_stamped)
	
	    #def following_cb(self, data):
		#self.current_pose_tf = data.pose
	
	    #def robot_pose_cb(self, data):
		#self.current_robot = data

	
	#mygoal = MoveBaseGoal()
        #mygoal.target_pose.header.frame_id = 'map'
        #mygoal.target_pose.header.stamp = rospy.Time.now()
        #mygoal.target_pose.pose.position.x = 3.0;
        #mygoal.target_pose.pose.orientation.w = 1.0;

	pan_goal = PtuGotoGoal() 
	pan_goal.pan = 0
	pan_goal.tilt = 0
	pan_goal.pan_vel = 1
	pan_goal.tilt_vel = 1

        #smach.StateMachine.add('Start',
        #              		SimpleActionState('move_base',
        #                                MoveBaseAction,
        #                                goal=mygoal),
        #              		transitions={'succeeded':'cc2'})

        smach.StateMachine.add('Start',
                      		SimpleActionState('SetPTUState',
                                        PtuGotoAction,
                                        goal=pan_goal),
                      		transitions={'succeeded':'cc1'})


	def child_term_cb(outcome_map):
	    print "jj"
 	    if outcome_map['Searching'] == 'succeeded':
 	       	return True
	    print "should never see it"
 	    return False

	def out_cb(outcome_map):
	    print "kk"
   	    if outcome_map['Searching'] == 'succeeded':
      		return 'aborted'
   	    else:
      		return 'succeeded'


	cc1 = smach.Concurrence(outcomes=['succeeded','aborted','preempted'],
				default_outcome = 'succeeded',
				child_termination_cb = child_term_cb,
                		outcome_cb = out_cb)	

	
	with cc1:
	    smach.Concurrence.add('Wandering',Wandering())				
	    smach.Concurrence.add('Searching',Searching())
	smach.StateMachine.add('cc1',cc1,
                      		transitions={'succeeded':'cc1',
						'aborted':'cc2'})


	cc2 = smach.Concurrence(outcomes=['succeeded','aborted','preempted'],
				default_outcome = 'aborted',
				outcome_map = {'succeeded':{'Searching':'succeeded'}})	
	with cc2:
	    smach.Concurrence.add('Following',Following())				  
	    smach.Concurrence.add('Searching',Searching())
	smach.StateMachine.add('cc2',cc2,
				transitions={'succeeded':'cc2',
						'aborted':'cc3'})

	cc3 = smach.Concurrence(outcomes=['succeeded','aborted','preempted'],
				default_outcome = 'aborted',
				outcome_map = {'succeeded':{'Searching':'succeeded'}})				
	with cc3:
	    smach.Concurrence.add('LocalSearching',LocalSearching())				
	    smach.Concurrence.add('Searching',Searching())
	smach.StateMachine.add('cc3',cc3,
                      		transitions={'succeeded':'cc2',
						'aborted':'cc1'})




	#smach.StateMachine.add('Wandering',Wandering(),
	#			transitions={'succeeded':'Wandering'})
	#smach.StateMachine.add('Following',Following(),
	#			transitions={'succeeded':'Following',
	#				     'aborted':'Wandering'})
        
    #self._feedback.trackstatus = 0
    rospy.loginfo('Executing following')
    outcome = sm.execute()

def main():
    rospy.init_node('following')
    server = followingServer()
    rospy.spin()
      
if __name__ == '__main__':
    main()


