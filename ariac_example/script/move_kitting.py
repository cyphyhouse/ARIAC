#!/usr/bin/env python

import rospy
import tf2_ros
import moveit_commander as mc
import math

import geometry_msgs.msg
from tf.transformations import euler_from_quaternion

import sys

def find_alphabeta(x, z):
	r1 = 0.573 # length of upper arm link (radius of its range of motion)
	r2 = 0.400 # length of forearm link

	ab = math.dist([-1.3, 1.127], [x,z]) # dist from kitting base joint to desired (x,z) point (a + b in proof)
	beta = law_cosines_gamma(r1, r2, ab)

	# alpha = alpha' + alpha'' (check proof)
	a1 = law_cosines_gamma(r1, ab, r2)
	a2 = math.acos((x+1.3)/ab)
	alpha = a1 + a2

	return (alpha, beta)

# Uses law of cosines to find the angle (in radians) of the side opposite of c
def law_cosines_gamma(a, b, c):
	return math.acos((a**2 + b**2 - c**2)/(2*a*b))


class MoveitRunner():
	def __init__(self, group_names, node_name='move_kitting',
				 ns='', robot_description='robot_description'):

		mc.roscpp_initialize(sys.argv)
		rospy.init_node('move_kitting', anonymous=True)

		self.robot = mc.RobotCommander(ns+'/'+robot_description, ns)
		self.scene = mc.PlanningSceneInterface(ns)
		self.groups = {}
		for group_name in group_names:
			group = mc.MoveGroupCommander(group_name, 
					   robot_description=ns+'/'+robot_description, 
					   ns=ns)
			group.set_goal_tolerance(0.001)	# toggle this on and off
			self.groups[group_name] = group

if __name__ == '__main__':

	kitting_group_names = ['kitting_arm']
	moveit_runner_kitting = MoveitRunner(kitting_group_names, ns='/ariac/kitting')

	kitting_arm = moveit_runner_kitting.groups['kitting_arm']
	kitting_arm.set_end_effector_link("vacuum_gripper_link")

	orientation_k = kitting_arm.get_current_pose().pose.orientation
	(roll, pitch, yaw) = euler_from_quaternion([orientation_k.x, orientation_k.y, orientation_k.z, orientation_k.w])

	# Get user input
	valid = False
	x = input("Enter the desired x-value for the kitting robot: ")
	valid = False
	while not valid:
		y = input("Enter the desired y-value for the kitting robot: ")
		if y >= -4.8 and y <= 4.8:
			break
		print("y-value must in range (-4.8, 4.8)")
		
	z = input("Enter the desired z-value for the kitting robot: ")
	print("Moving to (%s, %s, %s)" % (x, y, z))

	# linear arm actuator
	cur_joint_pose = moveit_runner_kitting.groups['kitting_arm'].get_current_joint_values()
	cur_joint_pose[0] = y - 0.1616191

	# shoulder pan joint
	if x < -1.3:
		cur_joint_pose[1] = 3.14
	else:
		cur_joint_pose[1] = 0

	moveit_runner_kitting.groups['kitting_arm'].go(cur_joint_pose, wait=True)
	moveit_runner_kitting.groups['kitting_arm'].stop()

	# Finding alpha (shoulder lift angle) and beta (elbow joint angle)
	alpha, beta = find_alphabeta(x, z)
	





		


