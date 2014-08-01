#!/usr/bin/env python

import roslib; roslib.load_manifest("human_trajectory_analyzer")
import rospy
import pymongo
import random
from visualization_msgs.msg import Marker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import *

from geometry_msgs.msg import Pose, Point
from std_msgs.msg import ColorRGBA

from strands_perception_people_msgs.msg import Logging

import json
import argparse
import math

def trapezoidal_shaped_func(a, b, c, d, x):
    min_val = min(min((x - a)/(b - a), float(1.0)), (d - x)/(d - c))
    return max(min_val, float(0.0))


def r_func(x):
    a = -0.125
    b =  0.125
    c =  0.375
    d =  0.625
    x = 1.0 - x
    value = trapezoidal_shaped_func(a,b,c,d,x)
    return value

def g_func(x):
    a =  0.125
    b =  0.375
    c =  0.625
    d =  0.875
    x = 1.0 - x
    value = trapezoidal_shaped_func(a,b,c,d,x)
    return value


def b_func(x):
    a =  0.375
    b =  0.625
    c =  0.875
    d =  1.125
    x = 1.0 - x
    value = trapezoidal_shaped_func(a,b,c,d,x)
    return value



class Trajectory:

    def __init__(self, uuid):

        self.uuid = uuid
        self.pose = []
        self.secs = []
        self.nsecs = []
        self.vel = []
        self.max_vel = 0.0
        self.length = 0.0
        
    def append_pose(self, pose, secs, nsecs):
        self.pose.append(pose)
        self.secs.append(secs)
        self.nsecs.append(nsecs)

    def calc_stats(self):

        length = 0.0
        if len(self.pose) < 2:
            return length
        self.vel.append(0.0) 
        for i in range(1,len(self.pose)):
            j = i - 1

            distance =  math.sqrt( math.pow(self.pose[i]['position']['x'] - self.pose[j]['position']['x'], 2) +
                                   math.pow(self.pose[i]['position']['y'] - self.pose[j]['position']['y'], 2))

            vel = distance / ((self.secs[i] - self.secs[j]) + (self.nsecs[i] - self.nsecs[j]) / math.pow(10,9))

            length += distance
            if vel > self.max_vel:
                self.max_vel = vel
            self.vel.append(vel) 
            
        self.length = length
        
    def to_JSON(self):
        return json.dumps(self, default=lambda o: o.__dict__, sort_keys=True, indent=4)



class TrajectoryAnalyzer():

    def __init__(self):
        
        host=rospy.get_param("datacentre_host")
        port=rospy.get_param("datacentre_port")

        self._client=pymongo.MongoClient(host,port)

        self._traj = dict()

        self._retrieve_logs()

        self._server = InteractiveMarkerServer("traj_vis")
                

    def _retrieve_logs(self):

        logs = self._client.message_store.people_perception.find() #.sort({'header.stamp': 1})

        for log in logs:
            for i, uuid in enumerate(log['uuids']):
                if uuid not in self._traj:
                    t = Trajectory(uuid)
                    t.append_pose(log['people'][i],log['header']['stamp']['secs'],log['header']['stamp']['nsecs'])
                    self._traj[uuid] = t
                else:
                    t = self._traj[uuid]
                    t.append_pose(log['people'][i],log['header']['stamp']['secs'],log['header']['stamp']['nsecs'])
        
    def visualize_trajectories(self):

        for uuid in self._traj:
            if len(self._traj[uuid].pose) > 1:
                self.visualize_trajectory(self._traj[uuid])
                _in = raw_input("Press 'Enter' for the next trajectory.")
                self.delete_trajectory(self._traj[uuid])
                
    def _update_cb(self,feedback):
        return
                    
    def visualize_trajectory(self, traj):

        int_marker = self.create_trajectory_marker(traj)
        self._server.insert(int_marker, self._update_cb)
        self._server.applyChanges()

    def delete_trajectory(self, traj):
        self._server.erase(traj.uuid)
        self._server.applyChanges()

    def create_trajectory_marker(self, traj):
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "/map"
        int_marker.name = traj.uuid
        int_marker.description = traj.uuid
        pose = Pose()
        pose.position.x = traj.pose[0]['position']['x']
        pose.position.y = traj.pose[0]['position']['y']
        int_marker.pose = pose

        line_marker = Marker()
        line_marker.type = Marker.LINE_STRIP
        line_marker.scale.x = 0.05

        # random.seed(traj.uuid)
        # val = random.random()
        # line_marker.color.r = r_func(val)
        # line_marker.color.g = g_func(val)
        # line_marker.color.b = b_func(val)
        # line_marker.color.a = 1.0

        line_marker.points = []

        MOD  = 1
        for i, point in enumerate(traj.pose):
            if i % MOD == 0:
                x = point['position']['x']
                y = point['position']['y']
                p = Point()
                p.x = x - int_marker.pose.position.x  
                p.y = y - int_marker.pose.position.y
                line_marker.points.append(p)

        line_marker.colors = []
        for i, vel in enumerate(traj.vel):
            if i % MOD == 0:
                color = ColorRGBA()
                val = vel / traj.max_vel
                color.r = r_func(val)
                color.g = g_func(val)
                color.b = b_func(val)
                color.a = 1.0
                line_marker.colors.append(color)
                

                
        # create a control which will move the box
        # this control does not contain any markers,
        # which will cause RViz to insert two arrows
        control = InteractiveMarkerControl()
        control.markers.append(line_marker) 
        int_marker.controls.append(control)
        
        return int_marker

        


if __name__=="__main__":

    parser = argparse.ArgumentParser(prog='trajectory')
                        
    args = parser.parse_args()
    
    rospy.init_node("traj")
    rospy.loginfo("Running Trajectory Analyzer")
    ta = TrajectoryAnalyzer()

    for k, v in ta._traj.iteritems():
        v.calc_stats()
        print k, len(v.pose), v.length, v.max_vel
    ta.visualize_trajectories()
    print 'No more trajectories.'    
    #rospy.spin()
   


