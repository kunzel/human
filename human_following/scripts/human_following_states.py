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
from sensor_msgs.msg import *

class FollowingSM(object):
    
    def __init__(self):
	rospy.loginfo("SM starts")
    
    def build_sm(self):
	self.sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
	with self.sm:
	
	    pan_goal = PtuGotoGoal() 
	    pan_goal.pan = 0
	    pan_goal.tilt = 0
	    pan_goal.pan_vel = 100
	    pan_goal.tilt_vel = 100

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
				    output_keys = ['degree_to_go'],
			  	    outcome_map = {'succeeded':{'MoveSearching':'succeeded'}})	
	    with cc2:
		smach.Concurrence.add('Following',Following())				  
		smach.Concurrence.add('MoveSearching',MoveSearching())
	    smach.StateMachine.add('cc2',cc2,
		  	           transitions={'succeeded':'cc2',
						'aborted':'cc3'})

	    cc3 = smach.Concurrence(outcomes=['succeeded','aborted','preempted'],
				    default_outcome = 'aborted',
				    input_keys = ['degree_to_go'],
				    outcome_map = {'succeeded':{'MoveSearching':'succeeded'}})				
	    with cc3:
		smach.Concurrence.add('LocalSearching',LocalSearching())				
		smach.Concurrence.add('MoveSearching',MoveSearching())
	    smach.StateMachine.add('cc3',cc3,
		             	   transitions={'succeeded':'cc2',
						'aborted':'Start'})

    def executing_sm(self):
	self.build_sm()
	outcome = self.sm.execute()	    



class Wandering(smach.State):
    def __init__(self):
	smach.State.__init__(self,
			     outcomes=['succeeded','aborted','preempted']
			    )
        rospy.loginfo('Wait for nav_goals')
	rospy.wait_for_service('nav_goals')
	try:
            self.nav_goals = rospy.ServiceProxy('nav_goals', NavGoals)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
	rospy.loginfo('receiving nav_goals')


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

        if self.preempt_requested():
            self.service_preempt()
            print 'Reset preempt'
	 
	print 'Enter wandering', self.preempt_requested()
	
	while (not self.preempt_requested()):
            poly = rospy.get_param('wander_area',[])
            points = []
            for point in poly:
                rospy.loginfo('Point: %s', point)
            	points.append(Point32(float(point[0]),float(point[1]),0))
            self.polygon = Polygon(points) 
	    self.num = 1
	    self.radius = 0.7
	    nav_goal = self.nav_goals(self.num,self.radius,self.polygon)

	    wait_point = Pose()
	    wp_array = rospy.get_param('wait_point',[])
	    wait_point.position.x = wp_array[0]
	    wait_point.position.y = wp_array[1]
	    wait_point.position.z = wp_array[2]
	    wait_point.orientation.x = wp_array[3]
	    wait_point.orientation.y = wp_array[4]
	    wait_point.orientation.z = wp_array[5]
	    wait_point.orientation.w = wp_array[6]

	    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    	    client.wait_for_server(rospy.Duration(60))
    	    goal = MoveBaseGoal()
   	    goal.target_pose.header.frame_id = 'map'
 	    goal.target_pose.header.stamp = rospy.Time.now()

	    mode = rospy.get_param('wandering_mode','normal')
	    if mode == 'normal':
	  	goal.target_pose.pose = nav_goal.goals.poses[0]
		client.sent_goal(goal)
	    else:
		goal.target_pose.pose = wait_point   		
		if self.is_inside(wait_point.position, points):
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

    def topic_setting_sim(self):
	self.pub = rospy.Publisher('/b21/pose_tf', PoseStamped)
	rospy.Subscriber('/b21/pose', PoseStamped, self.tf_cb)
	rospy.Subscriber('/b21/pose_tf', PoseStamped, self.following_cb)

    def topic_setting_real(self):
	rospy.Subscriber('/people_tracker/positions', PeopleTracker, self.people_pose_cb)

    def execute(self,userdata):
	while(1):
	    #print self.is_received
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
    
    def people_pose_cb(self, data):
	if len(data.uuids) == 0:
	    self.is_received = False
	else:
	    self.is_received = True
		
class MoveSearching(smach.State):
    def __init__(self):
	smach.State.__init__(self,
			     outcomes=['succeeded','aborted','preempted'],
			     output_keys=['current_robot','current_pose_tf','id_now']
			     )


	if rospy.get_param('envr','') == 'sim':
	    self.topic_setting_sim()
	else:
	    self.topic_setting_real()

	rospy.Subscriber('/robot_pose', Pose, self.robot_pose_cb)
	self.current_robot = Pose()
	self.current_pose = Pose()
	self.current_pose_tf = Pose()
	self.last_move_pose = Pose()
	self.last_post = Pose()
	self.last_move_time = rospy.get_time()
	self.is_received = bool()
	self.suspend = 0
	self.id_now = -1
	self.max_t_frames = rospy.get_param('max_t_frames',20)

    def topic_setting_sim(self):
	self.pub = rospy.Publisher('/b21/pose_tf', PoseStamped)
	rospy.Subscriber('/b21/pose', PoseStamped, self.tf_cb)
	rospy.Subscriber('/b21/pose_tf', PoseStamped, self.following_cb)

    def topic_setting_real(self):
	rospy.Subscriber('/people_tracker/positions', PeopleTracker, self.people_pose_cb)

    def execute(self,userdata):
	count = 0
	while(1):
	    count = count + 1
	    if self.is_received:
	    	userdata.current_robot = self.current_robot
	   	userdata.current_pose_tf = self.current_pose_tf
		userdata.id_now = self.id_now
		t = rospy.get_time() - self.last_move_time
		if t > 1:
		    print t 
		if t > 20:
		    return 'aborted'
		#    print 'static'
		    #return 'succeeded'
		#else:
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

    def move_dis(self, a, b):
	return (a.position.x - b.position.x)*(a.position.x - b.position.x) + (a.position.y - b.position.y)*(a.position.y - b.position.y) + (a.position.z - b.position.z)*(a.position.z - b.position.z) 

    def people_pose_cb(self, data):
	if self.id_now == -1:
	    if len(data.uuids) == 0:
		self.is_received = False
		return
	    else:
		#find the nearest
		self.id_now = data.uuids[data.distances.index(min(data.distances))]
		print self.id_now		
		return
	if self.id_now != -1 and self.id_now in data.uuids:
	    self.last_pose = self.current_pose_tf
	    self.current_pose_tf = data.poses[data.uuids.index(self.id_now)]
	    if self.move_dis(self.last_move_pose,self.current_pose_tf) > 0.5:
		self.last_move_pose = self.current_pose_tf
		self.last_move_time = rospy.get_time()
		print 'found new move'
	if  not(self.id_now in data.uuids):
	    self.suspend = self.suspend + 1
	else:
	    self.suspend = 0
	if (self.suspend >= self.max_t_frames) and not(self.id_now in data.uuids):
	    self.is_received = False
	    self.id_now = -1
	    self.suspend = 0
	else:
	    self.is_received = True
		

    def robot_pose_cb(self, data):
	self.current_robot = data



class LocalSearching(smach.State):
    def __init__(self):
	smach.State.__init__(self,
			     outcomes=['succeeded','aborted','preempted'],
			     input_keys = ['degree_to_go'])
	rospy.Subscriber('/ptu/states', JointState, self.ptu_cb)
	self.pan_status = 0.0
    def ptu_cb(self,data):
	self.pan_status = data.position[0] * 360 / 6.28
    def execute(self,userdata):

	rospy.sleep(rospy.Duration(0.5))
	if userdata.degree_to_go > 0:
	    angles = [0, 30, -30, 0]
	else:
	    angles = [0, -30, 30, 0]
	pan_client = actionlib.SimpleActionClient('SetPTUState', PtuGotoAction)
	rospy.loginfo("Waiting for Pantilt action server...")		
        pan_client.wait_for_server(rospy.Duration(60))
	rospy.loginfo("Connected to pan tilt server")
 	pan_goal = PtuGotoGoal()
	for angle in angles:		 
            pan_goal.pan = angle + self.pan_status
            pan_goal.tilt = 0
            pan_goal.pan_vel = 100
            pan_goal.tilt_vel = 50
	    if (self.preempt_requested()):	
		return 'succeeded'
	    pan_client.send_goal(pan_goal)
            pan_client.wait_for_result(rospy.Duration(1)) 
	    rospy.sleep(rospy.Duration(1))
	  
	return 'succeeded'



class Following(smach.State):
    def __init__(self):
	smach.State.__init__(self,
			     outcomes=['succeeded','aborted','preempted'],
			     input_keys = ['current_robot','current_pose_tf','id_now'],
			     output_keys = ['degree_to_go'])
	self.angle_old = 0
	#self.is_near_old = 0

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
	alpha = rospy.get_param('alpha', 0.7)
	rospy.sleep(rospy.Duration(0.3))
	if userdata.id_now == -1:
	    self.angle_old = 0	
	    rospy.sleep(rospy.Duration(0.3))
	move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	rospy.loginfo("Waiting for move_base action server...")
	move_client.wait_for_server(rospy.Duration(60))
	rospy.loginfo("Connected to move base server")
	move_goal = MoveBaseGoal()
	move_goal.target_pose.header.frame_id = 'map'
 	move_goal.target_pose.header.stamp = rospy.Time.now()
    		
	
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


        poly = rospy.get_param('follow_area',[])
        points = []
        for point in poly:
            rospy.loginfo('Point: %s', point)
            points.append(Point32(float(point[0]),float(point[1]),0))
	p = Point(userdata.current_pose_tf.position.x, userdata.current_pose_tf.position.y, userdata.current_pose_tf.position.z)


	distance = rospy.get_param('distance', 2)
	if dist <= distance or not self.is_inside(p, points):
	   
	    move_goal.target_pose.pose = userdata.current_robot
	    #self.is_near_old = 1
	    #rotq = quaternion_about_axis(rot_angle, [0,0,1])
	    #rotq = quaternion_about_axis(0, [0,0,1])
	    #newq = quaternion_multiply(q,rotq) 
	    #move_goal.target_pose.pose.orientation.x = newq[0]
	    #move_goal.target_pose.pose.orientation.y = newq[1]
	    #move_goal.target_pose.pose.orientation.z = newq[2]
	    #move_goal.target_pose.pose.orientation.w = newq[3]
	else:
 	    move_goal.target_pose.pose = userdata.current_pose_tf		

            #if self.is_near_old == 1:
		#pan_goal = PtuGotoGoal()
	        #pan_goal.pan = 0
        	#pan_goal.tilt = 0
            	#pan_goal.pan_vel = 100
            	#pan_goal.tilt_vel = 10
            	#pan_client.send_goal(pan_goal)
            	#pan_client.wait_for_result(rospy.Duration(0.1))


	    #self.is_near_old = 0



  	move_client.send_goal(move_goal)
	finished_within_time = move_client.wait_for_result(rospy.Duration(0.1))
	
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
 	
	if rot_angle_degree >= 20 or rot_angle_degree <= -20:
	    pan_goal = PtuGotoGoal() 
            pan_goal.pan = self.angle_old * alpha + rot_angle_degree * (1 - alpha)
            pan_goal.tilt = 0
            pan_goal.pan_vel = 50
            pan_goal.tilt_vel = 10

	    #if dist <= distance:
	    #    pan_goal.tilt = 10

            pan_client.send_goal(pan_goal)
            pan_client.wait_for_result(rospy.Duration(0.1)) 
 
	    self.angle_old = pan_goal.pan
	userdata.degree_to_go = rot_angle_degree
 	rospy.loginfo("New Position")
	return 'succeeded'
	
	
