#!/usr/bin/env python

import rospy
import tf2_ros
import moveit_commander as mc

import geometry_msgs.msg
from tf.transformations import euler_from_quaternion

import sys

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

	# Move linear arm actuator
	cur_joint_pose = moveit_runner_kitting.groups['kitting_arm'].get_current_joint_values()
	cur_joint_pose[0] = y - 0.1616191
	moveit_runner_kitting.groups['kitting_arm'].go(cur_joint_pose, wait=True)
	moveit_runner_kitting.groups['kitting_arm'].stop()
		


