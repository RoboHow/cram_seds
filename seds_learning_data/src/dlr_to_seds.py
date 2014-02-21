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
import time


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

        #The transformation from BaseLink to the Base of the Arm
        self.BL_T_O = kdl.Frame()
        self.BL_T_O.p[0] = 0.222
        self.BL_T_O.p[1] = -0.339
        self.BL_T_O.p[2] = 0.949
        quat = [0.428, 0.261, -0.435, 0.748]
        self.BL_T_O.M = kdl.Rotation.Quaternion(*quat)        

        self.print_counter = 0
        self.dlr_listener = rospy.Subscriber('/beasty_state_msgs', rcu2tcu, self.dlr_callback)        

    def get_kdl_chain_lwr(self):
        from PyKDL import Frame, Rotation, Vector, Joint, Chain, Segment
        from math import pi
        # arm segments that behave exactly like the DLR system. (Last pose in the middle of the potato)
        #Some parts of the DLR system take into account the TCP_T_EE transformation, for example to the Flange
        arm_segments = [
            Segment(Joint(Joint.None),
                    Frame(Rotation.Quaternion(0.428, 0.261, -0.435, 0.748), Vector(0.222, -0.339, 0.949))),
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
                    Frame(Rotation.RotX(pi/2), Vector(0, 0, 0.0))),  #-0.078
            Segment(Joint(Joint.RotZ),
                    Frame(Rotation.Identity(), Vector(0, 0, 0.0))),
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


        #fk = self.get_fk()
        #print "FK:"
        #print fk
        #print ""
        #tested that fk is the same as BL_T_X

        #getting the wrench at the EE
        jac = self.get_jacobian()
        jact = numpy.transpose(jac)

        torques = numpy.empty([7,1])
        for i in range(7):
            torques[i,0] = data.safety.tau_ext[i]

        #Wrench at the end effector in the coordinate frame of the beginning of the kinematic chain (the arm base or base_link)
        #From Sami's thesis: Fext = (jac * jac_transp) ^-1 * jac * tau_ext
        #It is derived from the known: jac_transp * F_end_effector = Tau
        ee_wrench = numpy.dot( numpy.dot (numpy.linalg.pinv( numpy.dot(jac, jact)  ), jac), torques)
        
        self.print_counter += 1
        if self.print_counter >= 200:
            self.print_counter = 0
            print "From Jacobian:"
            print ee_wrench
            #print jacn
            #print self.get_fk()
            #print self.jnt_pos

            print "From RTI: (reference frame of the base of the arm)"
            print  data.safety.f_ext_hat

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



        #Transformation from BaseLink to the Tip of the arm
        BL_T_X = self.BL_T_O * O_T_X

        j_angles_text = ("%r " * 7) %(data.robot.q)
        ee_cart_pose_text = ("%r " * 12) %(tuple(self.kdlFrame_to_12floats(BL_T_X)))
        joint_torque_text = ("%r " * 7) %(data.robot.tau)
        est_external_torque_text = ("%r " * 7) %(data.safety.tau_ext)

        #ee_cart_force_torque_text = ("%r " * 6) %(data.safety.f_ext_hat) #f_ext_hat is in the reference of the base of the robot
        ee_cart_force_torque_text = ("%r " * 6) %(tuple([float(i) for i in ee_wrench]))
        
        #print("%06.2f   %06.2f   %06.2f") %( data.safety.f_ext_hat[0], data.safety.f_ext_hat[1], data.safety.f_ext_hat[2] ) 
        index_text = "%r " %(0.0)

        delta_timestamp = data.com.timestamp - self.old_timestamp
        self.old_timestamp = data.com.timestamp

        #now = rospy.Time.now().to_nsec()
        #delta_ros_timestamp = now - self.old_time
        #self.old_time = now

        #ros_timestamp = "%r" %(rospy.Time.now().to_nsec()/1000000.0)
        timestamp_text = "%r " %(data.com.timestamp)


        #This is the format according to Lucia. 40 floats + one added at the end for the timestamp
        full_text = j_angles_text + ee_cart_pose_text + joint_torque_text + \
            est_external_torque_text + ee_cart_force_torque_text + index_text + \
            timestamp_text

        self.dest_file.write(full_text + "\n")




def main():
    rospy.init_node('dlr_to_seds')


    filename = "data_" + str(time.time()).split('.')[0] + ".txt"
    worker = DLR2SEDS(filename)

    r = rospy.Rate(10)

    print("dlr_to_seds: Saving data to %s") % (filename)
    #All the work is happening on the callback
    while not rospy.is_shutdown():
        r.sleep()

    print("dlr_to_seds: Exiting") 

if __name__ == '__main__':
    main()