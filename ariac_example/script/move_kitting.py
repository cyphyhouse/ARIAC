#!/usr/bin/env python

import rospy
import tf2_ros
import moveit_commander as mc
import math

import geometry_msgs.msg
from tf.transformations import euler_from_quaternion

import sys


def find_alphabeta(x, z):
	r1 = 0.61215	# range of motion of shoulder lift joint
	r2 = 0.57235	# range of motion of elbow joint
	#r1 = 0.573 # length of upper arm link (radius of its range of motion)
	#r2 = 0.400 # length of forearm link

	ab = euclidean_dist(-1.3, 1.1264, x, z) # dist from kitting base joint to desired (x,z) point (a + b in proof)
	beta = law_cosines_gamma(r1, r2, ab)

	# alpha = alpha' + alpha'' (check proof)
	a1 = law_cosines_gamma(r1, ab, r2)
	a2 = math.acos((abs(x+1.3))/ab)
	alpha = a1 + a2
	alpha = -alpha			# moving shoulder joint up is negative alpha direction
	beta = math.pi - beta	# we want complementary (beta is angle b/t two links, elbow joint is comp of this)
	# the above might need additional changes (e.g. abs val, etc) when trying to grab stuff on other side

	return (alpha, beta)

def euclidean_dist(x1, z1, x2, z2):
	return math.sqrt((x2-x1)**2 + (z2-z1)**2)

# Uses law of cosines to find the angle (in radians) of the side opposite of c
def law_cosines_gamma(a, b, c):
	return math.acos((a**2 + b**2 - c**2)/(2*a*b))

def bounds_checking(x, z):
	return True if euclidean_dist(x-0.1158, z+0.1, -1.3, 1.12725) <= 1.1845 else False


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

	def goto_pose(self, x, y, z):

		# TODO: incorporate roll pitch yaw in user input
		orientation_k = kitting_arm.get_current_pose().pose.orientation
		(roll, pitch, yaw) = euler_from_quaternion([orientation_k.x, orientation_k.y, orientation_k.z, orientation_k.w]) 

		# Bounds checking
		if y > 4.8 or y < -4.8:
			return False
		if not bounds_checking(x, z):
			return False

		conveyor_side = True if x >= -1.3 else False

		# Finding alpha (shoulder lift angle) and beta (elbow joint angle)
		if conveyor_side:
			alpha, beta = find_alphabeta(x-0.1158, z+0.1)	# adjust these values to account for wrist lengths
		else:
			alpha, beta = find_alphabeta(x+0.1154, z+0.1)

		print("alpha: ", alpha)
		print("beta: ", beta)

		cur_joint_pose = moveit_runner_kitting.groups['kitting_arm'].get_current_joint_values()

		# linear arm actuator
		if not conveyor_side:
			cur_joint_pose[0] = y + 0.1616191
		else:
			cur_joint_pose[0] = y - 0.1616191

		# shoulder pan joint
		if x < -1.3:
			cur_joint_pose[1] = 3.14
		else:
			cur_joint_pose[1] = 0

		# shoudler lift (alpha) and elbow (beta)
		cur_joint_pose[2] = alpha
		cur_joint_pose[3] = beta

		# to get flat ee: w1 = - shoulder lift - elbow - pi/2
		cur_joint_pose[4] = -1*cur_joint_pose[2] - cur_joint_pose[3] - math.pi/2

		# wrist 2 (always -pi/2 for now, until we incorporate roll, pitch, yaw)
		cur_joint_pose[5] = -math.pi/2

		# wrist 3
		cur_joint_pose[6] = 0

		print(cur_joint_pose)

		self.groups['kitting_arm'].go(cur_joint_pose, wait=True)
		self.groups['kitting_arm'].stop()		

		# TODO: eventually want to check with tf frames if move was successful or not
		return True

if __name__ == '__main__':

	kitting_group_names = ['kitting_arm']
	moveit_runner_kitting = MoveitRunner(kitting_group_names, ns='/ariac/kitting')

	kitting_arm = moveit_runner_kitting.groups['kitting_arm']
	kitting_arm.set_end_effector_link("vacuum_gripper_link")

	move_success = moveit_runner_kitting.goto_pose(-1.15, 0, 2)
	print("Move success: %s" % move_success)	


