#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import math
import numpy as np
from scipy.spatial import KDTree
from std_msgs.msg import Int32

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.traffic_waypoint_idx = None
        self.obstacle_waypoint = None
        self.current_pose = None
        self.base_waypoints = None
        self.waypoints_2D = None
        self.waypoint_tree = None

        self.loop()

    def loop(self):
      rate = rospy.Rate(50)
      while not rospy.is_shutdown():
        if self.current_pose and self.waypoint_tree and self.traffic_waypoint_idx:
          self.publish_waypoints()
        rate.sleep()

    def publish_waypoints(self):
      final_waypoints = self.generate_lane()
      self.final_waypoints_pub.publish(final_waypoints)

    def generate_lane(self):
      lane = Lane()

      closest_idx = self.get_closest_waypoint_idx()
      farthest_idx = closest_idx + LOOKAHEAD_WPS
      base_waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]

      if self.traffic_waypoint_idx == -1 or (self.traffic_waypoint_idx >= farthest_idx):
        lane.waypoints = base_waypoints

      else:
        lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

      return lane

    def decelerate_waypoints(self, waypoints, closest_idx):
      temp = []

      for i, waypoint in enumerate(waypoints):

        wp = Waypoint()
        wp.pose = waypoint.pose

        stop_idx = max(self.traffic_waypoint_idx - closest_idx - 2, 0)
        dist = self.distance(waypoints, i, stop_idx)
        vel = math.sqrt(2 * MAX_DECEL * dist)
        if vel < 1.:
          vel = 0.
        
        wp.twist.twist.linear.x = min(vel, waypoint.twist.twist.linear.x)
        temp.append(wp)
        
      return temp


    def get_closest_waypoint_idx(self):
      x = self.current_pose.pose.position.x
      y = self.current_pose.pose.position.y 
      closest_idx = self.waypoint_tree.query([x, y], 1)[1]

      closest_pt = self.waypoints_2D[closest_idx]
      prev_pt = self.waypoints_2D[closest_idx -1]

      closest_vt = np.array(closest_pt)
      prev_vt = np.array(prev_pt)
      current_vt = np.array([x, y])

      direction = np.dot(closest_vt-prev_vt, current_vt - closest_vt)

      if direction > 0:
        closest_idx = (closest_idx + 1) % len(self.waypoints_2D)
      return closest_idx

    def pose_cb(self, msg):
        self.current_pose = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        if not self.waypoints_2D:
          self.waypoints_2D = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] \
           for waypoint in self.base_waypoints.waypoints]
          self.waypoint_tree = KDTree(self.waypoints_2D)


    def traffic_cb(self, msg):
        self.traffic_waypoint_idx = msg.data

    def obstacle_cb(self, msg):
        self.obstacle_waypoint = msg

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
            
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
