#!/usr/bin/env python

import rospy
import moveit_commander as mc
from math import pi
from nist_gear.msg import VacuumGripperState
from nist_gear.srv import VacuumGripperControl

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
		gantry = [-3.6, -2.45, pi]
		arm = [0.0, -2.13, 1.9, 0.25, 1.55, 0]
		locations[name] = (gantry, arm)

		name = 'pickup_agv2'
		gantry = [-3.6, -2.15, pi]
		arm = [0.0, -0.525, 0.525, 0, pi/2, 0]
		locations[name] = (gantry, arm)

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

	def move_part(self, gm):
		self.goto_preset_location('pickup_agv2')
		gm.activate_gripper()

		num_attempts = 0
		MAX_ATTEMPTS = 20
		while not gm.is_object_attached() and num_attempts < MAX_ATTEMPTS:
			cur_joint_pose = moveit_runner_gantry.groups['gantry_full'].get_current_joint_values()
			cur_joint_pose[4] += 0.005	# shoulder lift joint
			cur_joint_pose[5] -= 0.005  # elbow joint
			moveit_runner_gantry.groups['gantry_full'].go(cur_joint_pose, wait=True)
			moveit_runner_gantry.groups['gantry_full'].stop()
			print("attempt: ", num_attempts, ", pose:", cur_joint_pose)
			num_attempts += 1
			rospy.sleep(1)

class GripperManager():
	def __init__(self, ns):
		self.ns = ns
	
	def activate_gripper(self):
		rospy.wait_for_service(self.ns + 'control')
		rospy.ServiceProxy(self.ns + 'control', VacuumGripperControl)(True)

	def deactivate_gripper(self):
		rospy.wait_for_service(self.ns + 'control')
		rospy.ServiceProxy(self.ns + 'control', VacuumGripperControl)(False)

	def is_object_attached(self):
		status = rospy.wait_for_message(self.ns + 'state', VacuumGripperState)
		return status.attached

if __name__ == '__main__':
	
	# Define MoveIt Group
	gantry_group_names = ['gantry_full', 'gantry_arm', 'gantry_torso']

	# an instance of MoveitRunner for the gantry robot
	moveit_runner_gantry = MoveitRunner(gantry_group_names, ns='/ariac/gantry')

	# Gripper
	gm = GripperManager(ns='/ariac/gantry/arm/gripper/')

	#moveit_runner_gantry.goto_preset_location('agv2_above')

	# No longer using path planning when near battery
	move_successful = moveit_runner_gantry.move_part(gm)

	# Go to Assembly Station
	moveit_runner_gantry.goto_preset_location('as1')


	

