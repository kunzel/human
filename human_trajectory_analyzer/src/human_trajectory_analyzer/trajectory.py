#!/usr/bin/env python

import rospy
import pymongo
from visualization_msgs.msg \
    import Marker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server \
    import InteractiveMarkerServer, InteractiveMarker
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import ColorRGBA

import json
import argparse
import math


def trapezoidal_shaped_func(a, b, c, d, x):
    min_val = min(min((x - a)/(b - a), float(1.0)), (d - x)/(d - c))
    return max(min_val, float(0.0))


def r_func(x):
    a = -0.125
    b = 0.125
    c = 0.375
    d = 0.625
    x = 1.0 - x
    value = trapezoidal_shaped_func(a, b, c, d, x)
    return value


def g_func(x):
    a = 0.125
    b = 0.375
    c = 0.625
    d = 0.875
    x = 1.0 - x
    value = trapezoidal_shaped_func(a, b, c, d, x)
    return value


def b_func(x):
    a = 0.375
    b = 0.625
    c = 0.875
    d = 1.125
    x = 1.0 - x
    value = trapezoidal_shaped_func(a, b, c, d, x)
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

    def sort_pose(self):
        if len(self.pose) > 1:
            self.pose, self.secs, self.nsecs = self.__quick_sort(self.pose,
                                                                 self.secs,
                                                                 self.nsecs)

    # this function assumes that all objects have the same length
    # and the secs are the same
    def __validate_poses(self, pose, secs, nsecs):
        i = 0
        while i < len(nsecs):
            reduced_nsecs = nsecs[(i + 1):]
            if nsecs[i] in reduced_nsecs:
                index = reduced_nsecs.index(nsecs[i])
                pose_index = pose[i + 1 + index]

                prev_nsecs = next_nsecs = 1000000
                prev_pose = next_pose = pose[i]
                for j in range(len(nsecs)):
                    delta = nsecs[j] - nsecs[i]
                    if delta == 0:
                        continue
                    if delta > 0 and delta < next_nsecs:
                        next_nsecs = nsecs[j]
                        next_pose = pose[j]
                    if delta < 0 and delta < prev_nsecs:
                        prev_nsecs = nsecs[j]
                        prev_pose = pose[j]

                if prev_nsecs == 1000000 and next_nsecs == 1000000:
                    pose = [pose[i]]
                    secs = [secs[i]]
                    nsecs = [nsecs[i]]
                    break

                delta_i = abs(pose[i] - prev_pose) + abs(pose[i] - next_pose)
                delta_index = abs(pose_index - prev_pose) + \
                    abs(pose_index - next_pose)
                if delta_i < delta_index:
                    del nsecs[i + 1 + index]
                    del pose[i + 1 + index]
                    del secs[i + 1 + index]
                    i -= 1
                else:
                    del nsecs[i]
                    del pose[i]
                    del secs[i]
                    i -= 1
            i += 1
        return pose, secs, nsecs

    def __quick_sort(self, pose, secs, nsecs, secs_sorted=False):
        less_pose = []
        equal_pose = []
        greater_pose = []
        less_secs = []
        equal_secs = []
        greater_secs = []
        less_nsecs = []
        equal_nsecs = []
        greater_nsecs = []

        if len(secs) > 1:
            pivot = secs[0]
            for i, sec in enumerate(secs):
                if sec < pivot:
                    less_secs.append(sec)
                    less_pose.append(pose[i])
                    less_nsecs.append(nsecs[i])
                elif sec == pivot:
                    equal_secs.append(sec)
                    equal_pose.append(pose[i])
                    equal_nsecs.append(nsecs[i])
                else:
                    greater_secs.append(sec)
                    greater_pose.append(pose[i])
                    greater_nsecs.append(nsecs[i])

            less_pose, less_secs, less_nsecs = \
                self.__quick_sort(less_pose, less_secs, less_nsecs,
                                  secs_sorted)
            greater_pose, greater_secs, greater_nsecs = \
                self.__quick_sort(greater_pose, greater_secs, greater_nsecs,
                                  secs_sorted)
            if not secs_sorted:
                equal_pose, equal_secs, equal_nsecs = \
                    self.__validate_poses(equal_pose, equal_secs, equal_nsecs)
                equal_pose, equal_nsecs, equal_secs = \
                    self.__quick_sort(equal_pose, equal_nsecs, equal_secs,
                                      not secs_sorted)

            return less_pose + equal_pose + greater_pose, less_secs + \
                equal_secs + greater_secs, less_nsecs + equal_nsecs + \
                greater_nsecs
        else:
            return pose, secs, nsecs

    def calc_stats(self):

        length = 0.0
        if len(self.pose) < 2:
            return length
        self.vel.append(0.0)
        for i in range(1, len(self.pose)):
            j = i - 1

            distance = math.hypot((self.pose[i]['position']['x']
                                  - self.pose[j]['position']['x']),
                                  (self.pose[i]['position']['x']
                                  - self.pose[j]['position']['x']))
            vel = distance / ((self.secs[i] - self.secs[j])
                              + (self.nsecs[i] - self.nsecs[j])
                              / math.pow(10, 9))
            length += distance
            if vel > self.max_vel:
                self.max_vel = vel
            self.vel.append(vel)

        self.length = length

    def to_JSON(self):
        return json.dumps(self, default=lambda o: o.__dict__,
                          sort_keys=True, indent=4)


class TrajectoryAnalyzer():

    def __init__(self, marker_name):

        host = rospy.get_param("mongodb_host")
        port = rospy.get_param("mongodb_port")

        self._client = pymongo.MongoClient(host, port)
        self._traj = dict()
        self._retrieve_logs()
        self._server = InteractiveMarkerServer(marker_name)

    def get_poses_persecond(self):
        average_poses = 0
        for uuid in self._traj:
            traj = self._traj[uuid]
            inner_counter = 1
            outer_counter = 1
            prev_sec = traj.secs[0]
            for i, sec in enumerate(traj.secs[1:]):
                if prev_sec == sec:
                    inner_counter += 1
                else:
                    prev_sec = sec
                    outer_counter += 1
            average_poses += round(inner_counter/outer_counter)
        return round(average_poses/len(self._traj))

    def _retrieve_logs(self):
        logs = self._client.message_store.people_perception.find()

        for log in logs:
            for i, uuid in enumerate(log['uuids']):
                if uuid not in self._traj:
                    t = Trajectory(uuid)
                    t.append_pose(log['people'][i],
                                  log['header']['stamp']['secs'],
                                  log['header']['stamp']['nsecs'])
                    self._traj[uuid] = t
                else:
                    t = self._traj[uuid]
                    t.append_pose(log['people'][i],
                                  log['header']['stamp']['secs'],
                                  log['header']['stamp']['nsecs'])

    def visualize_trajectories(self, mode="all", average_length=0,
                               longest_length=0):
        counter = 0

        for uuid in self._traj:
            if len(self._traj[uuid].pose) > 1:
                if mode == "average":
                    if abs(self._traj[uuid].length - average_length) \
                            < (average_length / 10):
                        self.visualize_trajectory(self._traj[uuid])
                        counter += 1
                elif mode == "longest":
                    if abs(self._traj[uuid].length - longest_length) \
                            < (longest_length / 10):
                        self.visualize_trajectory(self._traj[uuid])
                        counter += 1
                elif mode == "shortest":
                    if self._traj[uuid].length < 1:
                        self.visualize_trajectory(self._traj[uuid])
                        counter += 1
                else:
                    self.visualize_trajectory(self._traj[uuid])
                    counter += 1

        rospy.loginfo("Total Trajectories: " + str(len(self._traj)))
        rospy.loginfo("Printed trajectories: " + str(counter))
        self.delete_trajectory(self._traj[uuid])

    def _update_cb(self, feedback):
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
        # int_marker.description = traj.uuid
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
        MOD = 3
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
                if traj.max_vel == 0:
                    val = vel / 0.01
                else:
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

    def trajectory_visualization(self, mode):
        average_length = 0
        longest_length = -1
        short_trajectories = 0
        average_max_vel = 0
        highest_max_vel = -1
        minimal_frame = self.get_poses_persecond() * 5

        for k, v in self._traj.items():
            v.sort_pose()
            v.calc_stats()
            # Delete non-moving objects
            if (v.max_vel < 0.1 or v.length < 0.1) and k in self._traj:
                del self._traj[k]
            # Delete trajectories that appear less than 5 seconds
            if len(v.pose) < minimal_frame and k in self._traj:
                del self._traj[k]

        for k, v in self._traj.iteritems():
            average_length += v.length
            average_max_vel += v.max_vel
            if v.length < 1:
                short_trajectories += 1
            if longest_length < v.length:
                longest_length = v.length
            if highest_max_vel < v.max_vel:
                highest_max_vel = v.max_vel

        average_length /= len(self._traj)
        average_max_vel /= len(self._traj)
        rospy.loginfo("Average length of tracks is " + str(average_length))
        rospy.loginfo("Longest length of tracks is " + str(longest_length))
        rospy.loginfo("Short trajectories are " + str(short_trajectories))
        rospy.loginfo("Average maximum velocity of tracks is " +
                      str(average_max_vel))
        rospy.loginfo("Highest maximum velocity of tracks is " +
                      str(highest_max_vel))

        self.visualize_trajectories(mode, average_length, longest_length)


if __name__ == "__main__":
    mode = "all"
    parser = argparse.ArgumentParser(prog='trajectory')
    parser.add_argument("mode", help="[all | average | shortest | longest]")
    args = parser.parse_args()
    if args.mode != "":
        mode = args.mode

    rospy.init_node("human_trajectory_visualization")
    rospy.loginfo("Running Trajectory Analyzer")

    ta = TrajectoryAnalyzer('traj_vis')
    ta.trajectory_visualization(mode)

    raw_input("Press 'Enter' to exit.")
