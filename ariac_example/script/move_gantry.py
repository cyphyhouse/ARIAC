#!/usr/bin/env python

import rospy
import moveit_commander as mc
from math import pi

import sys

class MoveitRunner():
	def __init__(self, group_names, node_name='move_gantry',
				 ns='', robot_description='robot_description'):

		mc.roscpp_initialize(sys.argv)
		rospy.init_node(node_name, anonymous=True)

		self.robot = mc.RobotCommander(ns+'/'+robot_description, ns)
		self.scene = mc.PlanningSceneInterface(ns)
		self.groups = {}
		for group_name in group_names:
			group = mc.MoveGroupCommander(group_name, 
					   robot_description=ns+'/'+robot_description, 
					   ns=ns, wait_for_servers=30.0)
			group.set_goal_tolerance(0.001)
			self.groups[group_name] = group

		self.set_preset_location()

	def set_preset_location(self):
		locations = {}

		name = 'start'
		gantry = [-1.5, 0, 0]
		arm = [0.0, -pi/4, pi/2, -pi/4, pi/2, 0]
		locations[name] = (gantry, arm)

		name = 'pickup_standby'
		gantry = None
		arm = [-0.27, -0.57, 1.36, -0.81, 1.54, 0.83]
		locations[name] = (gantry, arm)

		name = 'station_standby'
		gantry = None
		arm = [0.0, -2.13, 1.9, 0.25, 1.55, 0.83]
		locations[name] = (gantry, arm)

		name = 'agv2_above'
		gantry = [-4.0, -3.0, pi]
		arm = [0.0, -2.13, 1.9, 0.25, 1.55, 0.83]

		name = 'as1'
		gantry = [-4.0, -3.0, pi/2]
		arm = [0.0, -2.13, 1.9, 0.25, 1.55, 0.83]
		locations[name] = (gantry, arm)

		name = 'agv2_to_as1_1'
		gantry = [-4.6, -0.6, 0]
		arm = None
		locations[name] = (gantry, arm)

		name = 'agv2_to_as1_2'
		gantry = [-4.6, -2.0, 0]
		arm = None
		locations[name] = (gantry, arm)

		name = 'as1_agv1'
		gantry = [-3.2, -4.0, 0]
		arm = None
		locations[name] = (gantry, arm)

		name = 'as1_agv2'
		gantry = [-3.2, -0.6, 0]
		arm = None
		locations[name] = (gantry, arm)

		self.locations = locations

	def goto_preset_location(self, location_name):
		group = self.groups['gantry_full']
		gantry, arm = self.locations[location_name]
		location_pose = group.get_current_joint_values()
		print(location_pose)

		if gantry:
			location_pose[:3] = gantry
		if arm:
			location_pose[3:] = arm

		MAX_ATTEMPTS = 5
		attempts = 0
		while not group.go(location_pose, wait=True):
			print(attempts)
			attempts += 1
			assert(attempts < MAX_ATTEMPTS)

		print(location_pose)


if __name__ == '__main__':
	
	# Define MoveIt Group
	gantry_group_names = ['gantry_full', 'gantry_arm', 'gantry_torso']

	# an instance of MoveitRunner for the gantry robot
	moveit_runner_gantry = MoveitRunner(gantry_group_names, ns='/ariac/gantry')

	moveit_runner_gantry.goto_preset_location('agv2_above')

