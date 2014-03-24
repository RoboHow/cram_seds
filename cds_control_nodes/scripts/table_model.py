#!/usr/bin/python

import roslib
roslib.load_manifest('visualization_msgs')
import rospy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose,Point,Quaternion,Vector3

from math import *

rospy.init_node('table_model')
pub = rospy.Publisher('visualization_marker', Marker)

import sys
frame_name = sys.argv[1]

def publishCylinder(pos, ori, scale, color, id):
  m = Marker()

  m.id = id
  m.ns = 'table'
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
  m.ns = 'table'
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
  publishCube([0.0, 0.0, 0.625], [0, 0, 0, 1], [0.6, 1.2, 0.05], 0) # table surface
  publishCube([0.25, -0.56, 0.3115], [0, 0, 0, 1], [0.05, 0.05, 0.623], 1)
  publishCube([0.25, 0.56,  0.3115], [0, 0, 0, 1], [0.05, 0.05, 0.623], 2)
  publishCube([-0.25, -0.56, 0.3115], [0, 0, 0, 1], [0.05, 0.05, 0.623], 3)
  publishCube([-0.25, 0.56,  0.3115], [0, 0, 0, 1], [0.05, 0.05, 0.623], 4)
  rate.sleep()
