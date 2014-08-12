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
from geometry_msgs.msg import Polygon, Pose, PoseStamped, Point, Point32
from tf.transformations import quaternion_about_axis, quaternion_multiply, euler_from_quaternion
from scitos_ptu.msg import *
from sensor_msgs.msg import *
from actionlib_msgs.msg import *
from strands_perception_people_msgs.msg import *

class FollowingSM(object):
    
    def __init__(self):
	rospy.loginfo("SM starts")
    
    def build_sm(self):
	self.sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
	with self.sm:
	
	    pan_goal = PtuGotoGoal() 
	    pan_goal.pan = 0
	    pan_goal.tilt = 0
	    pan_goal.pan_vel = 1
	    pan_goal.tilt_vel = 1

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
			  	    outcome_map = {'succeeded':{'MoveSearching':'succeeded'}})	
	    with cc2:
		smach.Concurrence.add('Following',Following())				  
		smach.Concurrence.add('MoveSearching',MoveSearching())
	    smach.StateMachine.add('cc2',cc2,
		  	           transitions={'succeeded':'cc2',
						'aborted':'cc3'})

	    cc3 = smach.Concurrence(outcomes=['succeeded','aborted','preempted'],
				    default_outcome = 'aborted',
				    outcome_map = {'succeeded':{'MoveSearching':'succeeded'}})				
	    with cc3:
		smach.Concurrence.add('LocalSearching',LocalSearching())				
		smach.Concurrence.add('MoveSearching',MoveSearching())
	    smach.StateMachine.add('cc3',cc3,
		             	   transitions={'succeeded':'cc2',
						'aborted':'cc1'})

    def executing_sm(self):
	self.build_sm()
	outcome = self.sm.execute()	    



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
	while (not self.preempt_requested()):
	    '''
	    self.polygon = Polygon()

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
		'''
            poly = rospy.get_param('wander_area',[])
            points = []
            for point in poly:
                rospy.loginfo('Point: %s', point)
            	points.append(Point32(float(point[0]),float(point[1]),0))
            self.polygon = Polygon(points) 
	    self.num = 1
	    self.radius = 0.5
	    nav_goal = self.nav_goals(self.num,self.radius,self.polygon)


	    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    	    client.wait_for_server(rospy.Duration(60))
    	    goal = MoveBaseGoal()
   	    goal.target_pose.header.frame_id = 'map'
 	    goal.target_pose.header.stamp = rospy.Time.now()
    	    goal.target_pose.pose = nav_goal.goals.poses[0]
	    		
	    client.send_goal(goal)
		
	
	    finished_within_time = client.wait_for_result(rospy.Duration(1))
	    while (not self.preempt_requested()) and client.get_state() == GoalStatus.ACTIVE:
	 	rospy.sleep(rospy.Duration(0.5))
	    client.cancel_goal()
	    print "Waited for move_base"

	return 'succeeded'


class Searching(smach.State):
    def __init__(self):
	smach.State.__init__(self,
			     outcomes=['succeeded','aborted','preempted'],
			     )

	if rospy.get_param('envr','') == 'sim':
	    self.topic_setting_sim()
	else:
	    self.topic_setting_real()

	self.current_pose = Pose()
	self.current_pose_tf = Pose()
	self.is_received = bool()
	#self.is_received = True

    def topic_setting_sim(self):
	self.pub = rospy.Publisher('/b21/pose_tf', PoseStamped)
	rospy.Subscriber('/b21/pose', PoseStamped, self.tf_cb)
	rospy.Subscriber('/b21/pose_tf', PoseStamped, self.following_cb)

    def topic_setting_real(self):
	rospy.Subscriber('/strands_perception_people_msgs', PedestrianLocations, self.people_pose_cb)

    def execute(self,userdata):
	while(1):
	    print self.is_received
	    if self.is_received:
		return 'succeeded'
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
	#print data.pose.position.x, data.pose.position.y, data.pose.position.z, self.is_received
    
    def people_pose_cb(self, data):
	#self.current_pose_tf = data.poses[0]
	if len(data.ids) == 0:
	    self.is_received = False
	else:
	    self.is_received = True
		
class MoveSearching(smach.State):
    def __init__(self):
	smach.State.__init__(self,
			     outcomes=['succeeded','aborted','preempted'],
			     output_keys=['current_robot','current_pose_tf']
			     )


	if rospy.get_param('envr','') == 'sim':
	    self.topic_setting_sim()
	else:
	    self.topic_setting_real()

	rospy.Subscriber('/robot_pose', Pose, self.robot_pose_cb)
	self.current_robot = Pose()
	self.current_pose = Pose()
	self.current_pose_tf = Pose()
	self.is_received = bool()
	self.suspend = False
	self.id_now = -1



    def topic_setting_sim(self):
	self.pub = rospy.Publisher('/b21/pose_tf', PoseStamped)
	rospy.Subscriber('/b21/pose', PoseStamped, self.tf_cb)
	rospy.Subscriber('/b21/pose_tf', PoseStamped, self.following_cb)

    def topic_setting_real(self):
	rospy.Subscriber('/strands_perception_people_msgs', PedestrianLocations, self.people_pose_cb)

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

    def people_pose_cb(self, data):
	if self.id_now == -1:
	    if len(data.ids) == 0:
		self.is_received = False
		return
	    else:
		#find the nearest
		self.id_now = data.ids[data.distances.index(min(data.distances))]
		return
	if self.id_now != -1 and self.id_now in data.ids:
	    self.current_pose_tf = data.poses[data.ids.index(self.id_now)]
	if (not self.suspend) and not(self.id_now in data.ids):
	    self.suspend = True
	else:
	    self.suspend = False 
	if (not self.suspend) and not(self.id_now in data.ids):
	    self.is_received = False
	    self.id_now = -1
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

    def ray_intersect_seg(self, p, a, b):
	if a.y > b.y:
	    a,b = b,a
	if p.y == a.y or p.y == b.y:
	    p = Point32(p.x, p.y + 0.00001, 0)
	intersect = False
	if (p.y > b.y or p.y < a.y) or (p.x > max(a.x, b.x)):
   	    return False
	if p.x < min(a.x, b.x):
	    intersect = True
	else:
	    if abs(a.x - b.x) > sys.float_info.min:
		m_red = (b.y - a.y) / float(b.x - a.x)
	    else:
		m_red = sys.float_info.max
	    if abs(a.x - p.x) > sys.float_info.min:
		m_blue = (p.y - a.y) / float(p.x - a.x)
	    else:
		m_blue = sys.float_info.max
    	    intersect = m_blue >= m_red
	return intersect

    def is_odd(self, x): return x%2 == 1

    def is_inside(self, p, poly):
        ln = len(poly)
	num_of_intersections = 0
	for i in range(0,ln):
	    num_of_intersections += self.ray_intersect_seg(p, poly[i], poly[(i + 1) % ln])
	return self.is_odd(num_of_intersections)


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


        poly = rospy.get_param('follow_area',[])
        points = []
        for point in poly:
            rospy.loginfo('Point: %s', point)
            points.append(Point32(float(point[0]),float(point[1]),0))
	p = Point(userdata.current_pose_tf.position.x, userdata.current_pose_tf.position.y, userdata.current_pose_tf.position.z)


	if dist <= 3.0 or not self.is_inside(p, points):
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

	distance = rospy.get_param('distance', 2)
	if dist <= distance:
	    pan_goal.tilt = 10

        pan_client.send_goal(pan_goal)
        pan_client.wait_for_result(rospy.Duration(0.1)) 
 
	rospy.loginfo("New Position")
	return 'succeeded'
	
	
