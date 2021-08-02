#!/usr/bin/env python

import rospy

if __name__ == '__main__':

	kitting_group_names = ['kitting_arm']
	#moveit_runner_kitting = MoveitRunner(kitting_group_names, ns='/ariac/kitting')

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

