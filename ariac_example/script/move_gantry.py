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
	
	# dir specifies the direction gantry robot is facing when picking up item
	# dir == 0: facing -x, dir == 1: facing +x, dir == 2: facing -y, dir == 3: facing +y
	def goto_pose(self, x, y, z, dir):
		



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

	moveit_runner_gantry.goto_preset_location('agv2_above')

	moveit_runner_gantry.goto_preset_location('pickup_agv2')

	# No longer using path planning when near battery
	move_successful = moveit_runner_gantry.move_part(gm)

	# Go to Assembly Station
	moveit_runner_gantry.goto_preset_location('agv2_to_as1_wp1')
	moveit_runner_gantry.goto_preset_location('as1_drop_battery')

	gm.deactivate_gripper()


	

