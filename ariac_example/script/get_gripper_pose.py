#!/usr/bin/env python

import rospy
import moveit_commander as mc
import geometry_msgs.msg
from nist_gear.msg import Order, LogicalCameraImage
from std_srvs.srv import Trigger
from math import pi

import sys


class MoveitRunner():
	def __init__(self, group_names, node_name='ariac_test',
				 ns='', robot_description='robot_description'):

		mc.roscpp_initialize(sys.argv)
		rospy.init_node('ariac_test', anonymous=True)

		self.robot = mc.RobotCommander(ns+'/'+robot_description, ns)
		self.scene = mc.PlanningSceneInterface(ns)
		self.groups = {}
		for group_name in group_names:
			group = mc.MoveGroupCommander(group_name, 
					   robot_description=ns+'/'+robot_description, 
					   ns=ns)
			group.set_goal_tolerance(0.05)	# toggle this on and off
			self.groups[group_name] = group


def print_func():
	print("============= Printing robot state")
	print(moveit_runner_kitting.robot.get_current_state().attached_collision_objects)
	#print(moveit_runner_kitting.robot.get_current_state().joint_state.name)
	#print(moveit_runner_kitting.robot.get_current_state().joint_state.position)
	#print("============= Printing robot pose")
	#print(moveit_runner_kitting.groups['kitting_arm'].get_current_pose())


if __name__ == '__main__':

	kitting_group_names = ['kitting_arm']
	moveit_runner_kitting = MoveitRunner(kitting_group_names, ns='/ariac/kitting')
	
	print_func()

