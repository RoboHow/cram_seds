#!/usr/bin/python

import roslib
roslib.load_manifest('visualization_msgs')
import rospy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose,Point,Quaternion,Vector3

from math import *

rospy.init_node('baker_model')
pub = rospy.Publisher('visualization_marker', Marker)

import sys
frame_name = sys.argv[1]

def publishCylinder(pos, ori, scale, color, id):
  m = Marker()

  m.id = id
  m.ns = 'baker'
  m.action = m.ADD
  m.type = m.CYLINDER

  m.header.stamp = rospy.Time.now()
  m.header.frame_id = frame_name

  m.lifetime = rospy.Duration(4.0)

  m.color = ColorRGBA(*color)

  m.pose.position = Point(*pos)
  m.pose.orientation = Quaternion(*ori)
  m.scale = Vector3(*scale)

  pub.publish(m)


def publishCube(pos, ori, scale, id):
  m = Marker()

  m.id = id
  m.ns = 'baker'
  m.action = m.ADD
  m.type = m.CUBE

  m.header.stamp = rospy.Time.now()
  m.header.frame_id = frame_name

  m.lifetime = rospy.Duration(4.0)

  m.color = ColorRGBA(0.6, 0.4, 0.2, 1)

  m.pose.position = Point(*pos)
  m.pose.orientation = Quaternion(*ori)
  m.scale = Vector3(*scale)

  pub.publish(m)

rate = rospy.Rate(0.5)
while not rospy.is_shutdown():
  publishCylinder([0,0,0], [0,0,0,1], [0.12, 0.12, 0.02], [0.8, 0.6, 0.0, 1], 1)
  rate.sleep()
