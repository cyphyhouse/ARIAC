#!/usr/bin/env python

import rospy
import tf2_ros
import moveit_commander as mc
import geometry_msgs.msg
from nist_gear.msg import Order, LogicalCameraImage
from std_srvs.srv import Trigger
from math import pi

import sys
import yaml
import re


def start_competition():
	
	rospy.wait_for_service('/ariac/start_competition')
	rospy.ServiceProxy('/ariac/start_competition', Trigger)()

def callback(data):
	print('first')
	rospy.loginfo(data.kitting_shipments[0].products[0].pose.position.x)
	print('----------------------------')
	print(data.kitting_shipments[0].products[0].pose.position.x)
	return data.kitting_shipments[0].products[0].pose.position.x
#	rospy.loginfo("I heard %s", data)
#	print('------------------------------')
#	print(data.kitting_shipments)


def get_order():
	order = rospy.wait_for_message('/ariac/orders', Order)
	return order
	'''
	# this is really janky. try to use wait for msg or smth instead if possible
	rospy.init_node('order_sub')
	testorder = rospy.Subscriber('/ariac/orders', Order, callback)
	print("sub:", testorder)
	rospy.spin()
	return "hello"
   # order = rospy.wait_for_message('/ariac/orders', Order)
   # return testtxt
	'''

def get_parts_from_cameras():
	tf_buffer = tf2_ros.Buffer()
	tf_listener = tf2_ros.TransformListener(tf_buffer)

	# wait for all cameras to be broadcasting
	all_topics = rospy.get_published_topics()
	camera_topics = [t for t, _ in all_topics if '/ariac/logical_camera' in t]
	for topic in camera_topics:
		rospy.wait_for_message(topic, LogicalCameraImage)

	camera_frame_format = r"logical_camera_[0-9]+_(\w+)_[0-9]+_frame"
	all_frames = yaml.safe_load(tf_buffer.all_frames_as_yaml()).keys()
	part_frames = [f for f in all_frames if re.match(camera_frame_format, f)]

	objects = []
	for frame in part_frames:
		try:
			world_tf = tf_buffer.lookup_transform(
				'world',
				frame,
				rospy.Time(),
				rospy.Duration(0.1)
			)
			ee_tf = tf_buffer.lookup_transform(
				frame,
				'ee_link',
				rospy.Time(),
				rospy.Duration(0.1)
			)
		except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
			continue

		# remove stale transforms
		tf_time = rospy.Time(
			world_tf.header.stamp.secs,
			world_tf.header.stamp.nsecs
		)
		if rospy.Time.now() - tf_time > rospy.Duration(1.0):
			continue

		model = Model()
		model.type = re.match(camera_frame_format, frame).group(1)
		model.pose.position = world_tf.transform.translation
		model.pose.orientation = ee_tf.transform.rotation
		objects.append(model)
	return objects

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

		self.set_preset_location()
		if ns == '/ariac/kitting':
			#self.goto_preset_location('bin8_far_battery_0', 'kitting_robot')
			self.goto_preset_location('bin8_far_battery_1', 'kitting_robot')
			#self.goto_preset_location('bin8_far_battery_0', 'kitting_robot')
#		if ns == '/ariac/gantry':
#			self.goto_preset_location('bin8', 'gantry_robot') # TOGGLE 1: see goto_preset_loc func

	def set_preset_location(self):
		locations = {}

		name = 'home'
		kitting_arm = [0, 3.141594222190707, -1.128743290405139, 1.5106304587276407, 4.25, -1.5079761953269788, 0]
		gantry_torso = [0, 0, 0]
		gantry_arm = [0.0, -1.13, 1.88, -0.72, 1.55, 0.83]
		locations[name] = (kitting_arm, gantry_torso, gantry_arm)

		name = 'bin8'
		kitting_arm = [1.3458887656258813, -0.5601138939850792, -0.2804510290896989, 0, -0.8072468824120538, 1.5385783777411373, 0.8298981409931709]
		gantry_torso = [-2.48400006879773, -1.6336322021423504, 0, 3.4200004668605506]
		gantry_arm = [0.0, -pi/4, pi/2, -pi/4, pi/2, 0]
		locations[name] = (kitting_arm, gantry_torso, gantry_arm)

		name = 'standby'
		kitting_arm = [2.70, 3.141594222190707, -1.01, 1.88, 3.77, -1.55, 0]
		gantry_torso = [0, 0, 0]
		gantry_arm = [0.0, -pi/4, pi/2, -pi/4, pi/2, 0]
		locations[name] = (kitting_arm, gantry_torso, gantry_arm)

		name = 'agv4'
		kitting_arm = [1.50, 3.141594222190707, -1.01, 1.88, 3.77, -1.55, 0]
		gantry_torso = [0, 0, 0]
		gantry_arm = [0.0, -pi/4, pi/2, -pi/4, pi/2, 0]
		locations[name] = (kitting_arm, gantry_torso, gantry_arm)

		name = 'bin8_far_battery_0'
		kitting_arm = [-3.65, 0, -1.25, -0.47, -1, 1.5, 0.83]
		gantry_torso = [0, 0, 0]	# gantry torso and arm copied over from above
		locations[name] = (kitting_arm, gantry_torso, gantry_arm)

		name = 'bin8_far_battery_1'
		kitting_arm = [-3.65, 0, -3.17, -0.47, -1, 1.5, 0.83]
		gantry_torso = [0, 0, 0]
		gantry_arm = [0.0, -pi/4, pi/2, -pi/4, pi/2, 0]
		locations[name] = (kitting_arm, gantry_torso, gantry_arm)

		name = 'start'
		kitting_arm = [0, 0, -1.25, 1.74, 1, -1.58, 0]
		gantry_torso = [0, 0, 0]
		gantry_arm = [0.0, -pi/4, pi/2, -pi/4, pi/2, 0]
		locations[name] = (kitting_arm, gantry_torso, gantry_arm)

		# needs editing
		name = 'conveyor'
		kitting_arm = [0, 0, -0.47, 1.51, 0.68, 0, 0]
		gantry_torso = [0, 0, 0]
		gantry_arm = [0.0, -pi/4, pi/2, -pi/4, pi/2, 0]
		locations[name] = (kitting_arm, gantry_torso, gantry_arm)

		self.locations = locations

	def goto_preset_location(self, location_name, robot_type):
		group = None
		if robot_type == 'kitting_robot':
			group = self.groups['kitting_arm']
		elif robot_type == 'gantry_robot':
			group = self.groups['gantry_full']

		kitting_arm, gantry_torso, gantry_arm = self.locations[location_name]
		location_pose = group.get_current_joint_values()

		print(location_pose)
		if robot_type == 'kitting_robot':
			location_pose = kitting_arm
		print(location_pose)
		
		'''
		location_pose[3] = 1.5
		print(location_pose)
		tmp_loc = self.locations[location_name]
		print("tmp:", tmp_loc)
		'''

		MAX_ATTEMPTS = 5
		attempts = 0
		while not group.go(location_pose, wait=True):
			print(attempts)
			attempts += 1
			assert(attempts < MAX_ATTEMPTS)
		
		group.stop()
		group.clear_pose_targets()

		print(location_pose)

def print_func(print_kitting):
#	order = get_order()
#	print("order:", order)
	
	if print_kitting:
		#planning_frame = move_group.get_planning_frame()
		planning_frame = moveit_runner_kitting.groups['kitting_arm'].get_planning_frame()
		print("============= Planning frame: %s" % planning_frame)
		eef_link = moveit_runner_kitting.groups['kitting_arm'].get_end_effector_link()
		print("============= End effector link: %s" % eef_link)
		#group_names = robot.get_group_names()
		group_names = moveit_runner_kitting.robot.get_group_names()
		print("============= Available Planning Groups:", group_names)
		print("============= Printing robot state")
		print(moveit_runner_kitting.robot.get_current_state())
		print("============= Printing robot pose")
		print(moveit_runner_kitting.groups['kitting_arm'].get_current_pose())

'''
	else:
		#planning_frame = move_group.get_planning_frame()
		planning_frame = moveit_runner_gantry.groups['gantry_full'].get_planning_frame()
		print("============= Planning frame: %s" % planning_frame)
		eef_link = moveit_runner_gantry.groups['gantry_full'].get_end_effector_link()
		print("============= End effector link: %s" % eef_link)
		#group_names = robot.get_group_names()
		group_names = moveit_runner_gantry.robot.get_group_names()
		print("============= Available Planning Groups:", group_names)
		print("============= Printing robot state")
		print(moveit_runner_gantry.robot.get_current_state())
		print("============= Printing robot pose")
		print(moveit_runner_gantry.groups['gantry_full'].get_current_pose())
	print("")
'''
def move_joint():
	joint_goal = moveit_runner_kitting.groups['kitting_arm'].get_current_joint_values()
	print("before:", joint_goal)
	joint_goal[3] = -0.7
	moveit_runner_kitting.groups['kitting_arm'].go(joint_goal, wait=True)
	moveit_runner_kitting.groups['kitting_arm'].stop()
	print("after:", joint_goal)

def change_pose():
	pose_goal = geometry_msgs.msg.Pose()
	print("pose:", pose_goal)
	pose_goal.orientation.w = -0.6
	pose_goal.position.x = -4.0
	pose_goal.position.y = 0.0
	pose_goal.position.z = 1.0
	print("pose goal:", pose_goal)
	moveit_runner_kitting.groups['kitting_arm'].set_pose_target(pose_goal)
	plan = moveit_runner_kitting.groups['kitting_arm'].go(wait=True)
	moveit_runner_kitting.groups['kitting_arm'].stop()
	moveit_runner_kitting.groups['kitting_arm'].clear_pose_targets()


if __name__ == '__main__':
	print("starting script")

	kitting_group_names = ['kitting_arm']
	gantry_group_names = ['gantry_full', 'gantry_arm', 'gantry_torso']
	moveit_runner_kitting = MoveitRunner(kitting_group_names, ns='/ariac/kitting')
	#moveit_runner_gantry = MoveitRunner(gantry_group_names, ns='/ariac/gantry')

	start_competition()
	order = get_order()
	#print("order: %s" % order)

	all_known_parts = get_parts_from_cameras()
	print("all known parts:", all_known_parts)
	
#	start_competition()
	print_kitting = True
	print_func(print_kitting)
#	move_joint()
#	change_pose()

	
	
	print("script finished")
