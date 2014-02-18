#!/usr/bin/env python
# Copyright (c) 2014 Alexis Maldonado Herrera <amaldo@cs.uni-bremen.de>
# Universitaet Bremen - Institute for Artificial Intelligence (Prof. Beetz)
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

import roslib; roslib.load_manifest('seds_learning_data')
import rospy

from dlr_msgs.msg import rcu2tcu
import PyKDL as kdl

import numpy

class DLR2SEDS(object):
    '''Takes the status messages from the DLR action interface
       and outputs a text file as needed by Lucia from EPFL'''
    def __init__(self, filename="data_001.txt"):
        
        self.dest_file = file(filename, mode="w")
        
        self.old_timestamp = 0
        self.old_time = 0

        self.chain = self.get_kdl_chain_lwr()

        self.nJoints = self.chain.getNrOfJoints()
        self.jnt_pos = kdl.JntArray(self.nJoints)            
        
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.jac_solver = kdl.ChainJntToJacSolver(self.chain)         
        self._jac = kdl.Jacobian(self.nJoints)
        
        
        self.dlr_listener = rospy.Subscriber('/BEASTY/state', rcu2tcu, self.dlr_callback)        
        
    def get_kdl_chain_lwr(self):
        from PyKDL import Frame, Rotation, Vector, Joint, Chain, Segment
        from math import pi
        # arm configuration
        arm_segments = [
                Segment(Joint(Joint.None),
                    Frame(Rotation.RPY(0.695, 0.868, -0.720), Vector(0.222, -0.339, 0.949))),
                Segment(Joint(Joint.None),
                    Frame(Rotation.Identity(), Vector(0.0, 0.0, 0.11))),
                Segment(Joint(Joint.RotZ),
                    Frame(Rotation.RotX(-pi/2), Vector(0.0, 0.0, 0.20))),
                Segment(Joint(Joint.RotZ, -1),
                    Frame(Rotation.RotX(pi/2), Vector(0.0, -0.20, 0.0))),
                Segment(Joint(Joint.RotZ),
                    Frame(Rotation.RotX(pi/2), Vector(0, 0, .20))),
                Segment(Joint(Joint.RotZ, -1),
                    Frame(Rotation.RotX(-pi/2), Vector(0, 0.2, 0))),
                Segment(Joint(Joint.RotZ),
                    Frame(Rotation.RotX(-pi/2), Vector(0, 0, 0.19))),
                Segment(Joint(Joint.RotZ, -1),
                    Frame(Rotation.RotX(pi/2), Vector(0, -0.078, 0.0))),
                Segment(Joint(Joint.RotZ),
                    Frame(Rotation.Identity(), Vector(0, 0, 0))),
                    ]
        chain = Chain()
        for segment in arm_segments:
            chain.addSegment(segment)
        return(chain)

    def get_jacobian(self):
        jac = kdl.Jacobian(self.nJoints)
        self.jac_solver.JntToJac(self.jnt_pos, jac)
        
        jacn = numpy.empty([6,7])
        for i in range(6):
            for j in range(7):
                jacn[i,j] = jac[i,j]
        return jacn    
    
    def set_joints(self, joints):
        for i in range(len(joints)):
            self.jnt_pos[i] = joints[i]
            
    def get_joints(self):
        return(self.jnt_pos)
          
        
    def kdlFrame_to_12floats(self,frame):
        f = kdl.Frame(frame)
        
        return  [f.M[0,0], f.M[0,1], f.M[0,2], f.p[0], \
                 f.M[1,0], f.M[1,1], f.M[1,2], f.p[1], \
                 f.M[2,0], f.M[2,1], f.M[2,2], f.p[2], \
                 ]
    def get_fk(self):
        f = kdl.Frame()
        self.fk_solver.JntToCart(self.jnt_pos, f)
        return f
        
    def dlr_callback(self, data):
        
        #Setting the current joint angles
        self.set_joints(data.robot.q)
        
        #getting the wrench at the EE
        jacn = self.get_jacobian()
        torques = numpy.empty([7,1])
        for i in range(7):
            torques[i,0] = data.safety.tau_ext[i]
        
        ee_wrench = -1 * numpy.dot(jacn, torques)
        print ee_wrench
        #print jacn
        #print self.get_fk()
        #print self.jnt_pos
        print "\n"        
        
        #Read the Transformation from base of the arm to the tip of the arm
        O_T_X = kdl.Frame()
        robot_pose = data.robot.O_T_x
        O_T_X.M[0,0] = robot_pose[0]
        O_T_X.M[1,0] = robot_pose[1]
        O_T_X.M[2,0] = robot_pose[2]
        O_T_X.M[0,1] = robot_pose[3]
        O_T_X.M[1,1] = robot_pose[4]
        O_T_X.M[2,1] = robot_pose[5]
        O_T_X.M[0,2] = robot_pose[6]
        O_T_X.M[1,2] = robot_pose[7]
        O_T_X.M[2,2] = robot_pose[8]
        O_T_X.p[0] = robot_pose[9]
        O_T_X.p[1] = robot_pose[10]
        O_T_X.p[2] = robot_pose[11]
        
        #The transformation from BaseLink to the Base of the Arm
        BL_T_O = kdl.Frame()
        BL_T_O.p[0] = 0.222
        BL_T_O.p[1] = -0.339
        BL_T_O.p[2] = 0.949
        quat = [0.428, 0.261, -0.435, 0.748]
        BL_T_O.M = kdl.Rotation.Quaternion(*quat)        
        
        #Transformation from BaseLink to the Tip of the arm
        BL_T_X = BL_T_O * O_T_X
        
        j_angles_text = ("%r " * 7) %(data.robot.q)
        ee_cart_pose_text = ("%r " * 12) %(tuple(self.kdlFrame_to_12floats(BL_T_X)))
        joint_torque_text = ("%r " * 7) %(data.robot.tau)
        est_external_torque_text = ("%r " * 7) %(data.safety.tau_ext)
        
        ee_cart_force_torque_text = ("%r " * 6) %(tuple([float(ee_wrench[i]) for i in range(6)]))
        
        index_text = "%r " %(0.0)

        delta_timestamp = data.com.timestamp - self.old_timestamp
        self.old_timestamp = data.com.timestamp
        
        #now = rospy.Time.now().to_nsec()
        #delta_ros_timestamp = now - self.old_time
        #self.old_time = now
        
        #ros_timestamp = "%r" %(rospy.Time.now().to_nsec()/1000000.0)
        timestamp_text = "%r " %(data.com.timestamp)
    
        
        
        full_text = j_angles_text + ee_cart_pose_text + joint_torque_text + \
            est_external_torque_text + ee_cart_force_torque_text + index_text + \
            timestamp_text
        
        self.dest_file.write(full_text + "\n")
        



def main():
    rospy.init_node('dlr_to_seds')
    
    worker = DLR2SEDS()

    r = rospy.Rate(10)

    #All the work is happening on the callback
    while not rospy.is_shutdown():
        r.sleep()
        
        
if __name__ == '__main__':
    main()
