#!/usr/bin/env python

import rospy
import tf2_ros
import moveit_commander as mc

import geometry_msgs.msg

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
	'''
	rospy.init_node('move_kitting')
	
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)

	frame = 'gantry_arm_vacuum_gripper_link'
	rate = rospy.Rate(1.0)
	while not rospy.is_shutdown():
		print('here')
		try:
			trans = tfBuffer.lookup_transform('world', frame, rospy.Time(), rospy.Duration(1.0))
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
			print(e)
			continue

	# Transform the pose from the specified frame to the world frame
	local_pose = geometry_msgs.msg.PoseStamped()
	local_pose.header.frame_id = frame
	local_pose.pose.position.x = 0.15
	local_pose.pose.position.y = 0.15

	world_pose = tfBuffer.transform(local_pose, 'world')
	print(world_pose)
	rate.sleep()
	'''

	kitting_group_names = ['kitting_arm']
	moveit_runner_kitting = MoveitRunner(kitting_group_names, ns='/ariac/kitting')

	kitting_arm = moveit_runner_kitting.groups['kitting_arm']
	kitting_arm.set_end_effector_link("vacuum_gripper_link")

	print(kitting_arm.get_current_pose())

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


