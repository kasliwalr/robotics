#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        
        # create symbols for DH parameters
	q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
	d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
	a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
	alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        #create symbols for yaw, pitch, roll of the gripper link with respect to base link
        y, p, r = symbols('y, p, r')
	
	# create DH parameter table and populate with calculated DH parameters
	param_dict = {d1:0.75,d2:0,d3:0,d4:1.5,d5:0,d6:0,d7:0.303, 
		      a0:0,a1:0.35,a2:1.25,a3:-0.054,a4:0,a5:0,a6:0,
		      alpha0:0,alpha1:-pi/2,alpha2:0,alpha3:-pi/2,alpha4:pi/2,alpha5:-pi/2,alpha6:0,
		      q2: -pi/2 + q2, q7:0}


        #param_dict2 = {y:0, p:0, r:0}
        # Calculate the correction matrix to account for difference between orientation of DH gripper link frame and URDF gripper link frame
	R_zcorr = Matrix([[-1,0,0],[0,1,0],[0,0,1]])
	R_ycorr = Matrix([[0,0,-1],[0,1,0],[1,0,0]])
	R_corr = R_zcorr*R_ycorr

        def TF_Matrix(alpha, a, d, q):
            TF = Matrix([[cos(q), -sin(q), 0, a],[sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                     [sin(q)*sin(alpha), cos(q)*sin(alpha), cos(alpha), cos(alpha)*d],[0,0,0,1]])
            return TF

        #Create individual transformation matrices from base link to gripper link
        T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(param_dict)
        T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(param_dict)
        T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(param_dict)
        T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(param_dict)
        T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(param_dict)
        T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(param_dict)
        T6_G = TF_Matrix(alpha6, a6, d7, q7).subs(param_dict)

        #Total homogenous transform from base link to gripper link. No simplication as that runs much slower
	T_tot = T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_G
  
        #Homogenous transform from base link to link 3
        T0_3 = simplify(T0_1*T1_2*T2_3)
        

        #create symbolic individual rotatation matrices from yaw, pitch, roll of end gripper wrt base link
        R_yaw = Matrix([[cos(y), -sin(y), 0],[sin(y), cos(y), 0],[0, 0, 1]])
        R_pitch = Matrix([[cos(p), 0, sin(p)],[0, 1, 0],[-sin(p), 0, cos(p)]])
        R_roll = Matrix([[1, 0, 0],[0, cos(r), -sin(r)],[0, sin(r), cos(r)]])
        #R_tot_h = simplify(R_yaw*R_pitch*R_roll*R_corr)

        # Initialize service response
        joint_trajectory_list = []
        count = 0
        for x in xrange(0, len(req.poses)):
            count += 1
            print count
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
            #param_dict2[yaw] = yaw
            #param_dict2[pitch] = pitch
            #param_dict2[roll] = roll
            #R_tot_h_value = R_tot_h.evalf(subs=param_dict2)
            #Calculate rotation matrix to  calculate DH gripper link with respect to base link from gripper pose
            R_tot_h_value = R_yaw.subs({y:yaw})*R_pitch.subs({p:pitch})*R_roll.subs({r:roll})*R_corr

            #Calculate wrist center
            dWC_G = param_dict[d6] + param_dict[d7]
	    xc  =  px - (dWC_G)*R_tot_h_value[0,2]
            yc = py - (dWC_G)*R_tot_h_value[1,2]
            zc = pz - (dWC_G)*R_tot_h_value[2,2]
	    
	    
	    # Calculate angles for joints 1, 2, 3 using geometrix IK method
            l = sqrt(param_dict[a3]**2 + param_dict[d4]**2)
            theta3_0 = atan2(abs(param_dict[a3]), param_dict[d4])
            radius = sqrt(yc**2 + xc**2)
            phi = atan2(zc - param_dict[d1], radius - param_dict[a1])
            s3 = sqrt((radius - param_dict[a1])**2 + (zc - param_dict[d1])**2)
            alpha =  acos((param_dict[a2]**2 + s3**2 - l**2)/(2*param_dict[a2]*s3))
            beta = acos((param_dict[a2]**2 + l**2 - s3**2)/(2*param_dict[a2]*l))
            phi = atan2(zc - param_dict[d1], radius - param_dict[a1])

	    the1 = atan2(yc, xc)
            the2 = pi/2 - phi - alpha
	    the3 = pi/2 - beta - theta3_0
            R0_3_value = T0_3.evalf(subs={q1:the1, q2:the2, q3:the3})
	    R0_3_value = R0_3_value[0:3,0:3]
            R3_6 = R0_3_value.inv("LU")*R_tot_h_value
         
            theta1 = the1
            theta2 = the2
            theta3 = the3
	    theta4 = atan2(R3_6[2,2],-R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
            theta6 = atan2(-R3_6[1,1],R3_6[1,0])
            
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
