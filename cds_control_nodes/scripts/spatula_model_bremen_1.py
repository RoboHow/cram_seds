#!/usr/bin/python

import roslib
roslib.load_manifest('visualization_msgs')
import rospy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose,Point,Quaternion,Vector3

from math import *

rospy.init_node('spatula_model')
pub = rospy.Publisher('visualization_marker', Marker)

# default: /l_gripper_tool_frame
import sys
frame_name = sys.argv[1]


def publishCube(pos, ori, scale, id):
  m = Marker()

  m.id = id
  m.ns = frame_name
  m.action = m.ADD
  m.type = m.CUBE

  m.header.stamp = rospy.Time.now()
  m.header.frame_id = frame_name

  m.lifetime = rospy.Duration(4.0)

  m.color = ColorRGBA(0.3, 0.3, 0.3, 1)

  m.pose.position = Point(*pos)
  m.pose.orientation = Quaternion(*ori)
  m.scale = Vector3(*scale)

  m.frame_locked = True

  pub.publish(m)

rate = rospy.Rate(0.5)
while not rospy.is_shutdown():
  publishCube([0.06, 0, -0.14], [0, sin(-0.28), 0, cos(-0.28)], [0.01, 0.01, 0.22], 1)
  publishCube([0, 0, 0], [0, 0, 0, 1], [0.01, 0.075, 0.095], 2)
  rate.sleep()

