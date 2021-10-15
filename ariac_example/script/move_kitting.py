#!/usr/bin/env python

import rospy
import tf2_ros
import moveit_commander as mc
import math
import yaml
from yaml.loader import SafeLoader

import tf2_listener

import geometry_msgs.msg
from tf.transformations import euler_from_quaternion
from nist_gear.msg import VacuumGripperState, Proximity, LogicalCameraImage
from nist_gear.srv import VacuumGripperControl, ConveyorBeltControl
from std_srvs.srv import Trigger
from std_msgs.msg import String

import sys

def start_competition():
    """ Start the competition through ROS service call """

    rospy.wait_for_service('/ariac/start_competition')
    rospy.ServiceProxy('/ariac/start_competition', Trigger)()

def competition_state():
	data = rospy.wait_for_message('/ariac/competition_state', String)
	print(data)
	return data

def move_agvs(agv, dest):
	rospy.wait_for_service('/ariac/' + agv + '/to_' + dest)
	rospy.ServiceProxy('/ariac/' + agv + '/to_' + dest, Trigger)()

def get_breakbeam_sensor_data():
	data = rospy.wait_for_message('/ariac/breakbeam_conveyor', Proximity)
	return data
def get_breakbeam_flat_sensor_data():
	data = rospy.wait_for_message('/ariac/breakbeam_conveyor_flat', Proximity)
	return data
def get_logical_camera_conveyor_data():
	data = rospy.wait_for_message('/ariac/logical_camera_conveyor', LogicalCameraImage)
	return data

def control_conveyor(power):
	if power < 0 or power > 100:
		print("Power must be in range (0,100)")
		return

	rospy.wait_for_service('/ariac/conveyor/control')
	conveyor_rosservice = rospy.ServiceProxy('/ariac/conveyor/control', ConveyorBeltControl)
	try:
		conveyor_rosservice(power)
	except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))

def find_alphabeta(x, z):
	r1 = 0.61215	# range of motion of shoulder lift joint
	r2 = 0.57235	# range of motion of elbow joint
	#r1 = 0.573 # length of upper arm link (radius of its range of motion)
	#r2 = 0.400 # length of forearm link

	ab = euclidean_dist(-1.3, 1.1264, x, z) # dist from kitting base joint to desired (x,z) point (a + b in proof)
	beta = law_cosines_gamma(r1, r2, ab)

	a1 = law_cosines_gamma(r1, ab, r2)
	a2 = math.acos((abs(x+1.3))/ab)
	# alpha = alpha' + alpha'' if goal z is above start z
	if z >= 1.1264:
		alpha = a1 + a2
	else:
		alpha = a1 - a2

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
			group.set_goal_tolerance(0.0001)	# toggle this on and off
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
		return x, y, z

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

class Follow_points():
	def __init__(self, moveit_runner_kitting, gm):
		self.moveit_runner_kitting = moveit_runner_kitting
		self.gm = gm
		self.state = 0
		self.agvs = [0, 0, 0, 0]	# 0 = unused, 1 = used
		with open("/home/dxwu2/araic_ws/src/ARIAC/nist_gear/config/user_config/sample_user_config.yaml", "r") as f:
			data = yaml.load(f)
			# breakbeam_conveyor_pose_xyz = data['sensors']['breakbeam_conveyor']['pose']['xyz']
		self.sensor_file = data

	def main_body(self):
		# Start state: Kitting robot may be out of position
		if self.state == 0:
			start_pose = [-1.15, 0, 2]
			self.moveit_runner_kitting.goto_pose(start_pose[0], start_pose[1], start_pose[2])
			
			# verify if there, then change state
			cx = kitting_arm.get_current_pose().pose.position.x
			cy = kitting_arm.get_current_pose().pose.position.y
			cz = kitting_arm.get_current_pose().pose.position.z
			if abs(cx - start_pose[0]) + abs(cy - start_pose[1]) + abs(cz - start_pose[2]) <= 0.01:
				self.state = 1
			return
		
		# State 1: Kitting robot in starting position
		if self.state == 1:
			if competition_state().data == "init":
				start_competition()
			else:
				control_conveyor(100)
			self.state = 2
			return
		
		# State 2: Wait for battery
		if self.state == 2:
			while not (get_breakbeam_sensor_data().object_detected and get_breakbeam_flat_sensor_data().object_detected):
				rospy.sleep(0.05)	# how to do this with pthread cond wait
			control_conveyor(0)
			self.state = 3
			return

		# State 3: Battery detected, moving arm toward battery
		if self.state == 3:
			self.gm.activate_gripper()

			# Use Logical Camera to get pose of battery(s)
			# data = get_logical_camera_conveyor_data()[0]

			# todo: add functionality to grab all batteries detected by camera (one at a time)

			for i in range(1,9):
				source_frame = 'logical_camera_conveyor_frame'
				target_frame = 'logical_camera_conveyor_assembly_battery_green_' + str(i) + '_frame'
				world_pose = tf2_listener.get_transformed_pose(source_frame, target_frame)
				if world_pose is not None:
					break

			if world_pose is None:
				self.state = 2
				return

			self.moveit_runner_kitting.goto_pose(world_pose.pose.position.x, world_pose.pose.position.y, world_pose.pose.position.z + 0.03)

			# battery_detect_pose = self.sensor_file['sensors']['breakbeam_conveyor']['pose']['xyz']
			# battery_detect_pose[2] -= 0.985	# detection length is 1 meter (a little less for height of battery)
			# self.moveit_runner_kitting.goto_pose(battery_detect_pose[0], battery_detect_pose[1], battery_detect_pose[2])

			# Verify robot moved there
			# cx = kitting_arm.get_current_pose().pose.position.x
			# cy = kitting_arm.get_current_pose().pose.position.y
			# cz = kitting_arm.get_current_pose().pose.position.z
			# print("cx:", cx - world_pose.pose.position.x)
			# print("cy:", cy - world_pose.pose.position.y)
			# print("cz:", cz - world_pose.pose.position.z + 0.03)
			# print(abs(cx - world_pose.pose.position.x) + abs(cy - world_pose.pose.position.y) + abs(cz - (world_pose.pose.position.z + 0.03)))
			# if abs(cx - world_pose.pose.position.x) + abs(cy - world_pose.pose.position.y) + abs(cz - world_pose.pose.position.z + 0.03) <= 0.01:
			# 	self.state = 4
			self.state = 4
			return
		
		# State 4: Lowering toward battery
		if self.state == 4:
			cx = kitting_arm.get_current_pose().pose.position.x
			cy = kitting_arm.get_current_pose().pose.position.y
			cz = kitting_arm.get_current_pose().pose.position.z
			self.moveit_runner_kitting.goto_pose(cx, cy, cz - 0.002)
			if self.gm.is_object_attached():
				self.state = 5
			return

		# State 5: Moving to AGV
		if self.state == 5:
			agv2_pose = [-2.265, 1.3676, 1.0]
			self.moveit_runner_kitting.goto_pose(-0.572989, 0, 2.0)	# waypoint to avoid crashing into conveyor
			self.moveit_runner_kitting.goto_pose(agv2_pose[0], agv2_pose[1], agv2_pose[2])
			
			# verify if there, then change state
			cx = kitting_arm.get_current_pose().pose.position.x
			cy = kitting_arm.get_current_pose().pose.position.y
			cz = kitting_arm.get_current_pose().pose.position.z
			if abs(cx - agv2_pose[0]) + abs(cy - agv2_pose[1]) + abs(cz - agv2_pose[2]) <= 0.01:
				self.state = 6
			return

		# State 6: Drop battery
		if self.state == 6:
			self.gm.deactivate_gripper()
			rospy.sleep(2.0)
			self.state = 7
			return

		# State 7: Transport to Gantry Robot
		if self.state == 7:
			# move_agvs('agv2', 'as1')
			self.state = 0
			return



	def loop_body(self):
		# bandaid fix: get robot arm out of the way so no interference with break beam sensor
		print("clearing arm")
		self.moveit_runner_kitting.goto_pose(-1.15, 0, 2)

		print("waiting for battery")
		while not (get_breakbeam_sensor_data().object_detected and get_breakbeam_flat_sensor_data().object_detected):
			rospy.sleep(0.05)	# how to do this with pthread cond wait

		# stop conveyor belt upon sensor detecting battery in range
		print("battery detected, stop belt")
		control_conveyor(0)

		print("moving arm to battery")
		self.gm.activate_gripper()
		cx,cy,cz = moveit_runner_kitting.goto_pose(-0.572989, 0, 0.935)
		print("lowering")
		while not self.gm.is_object_attached():
			print(cx, cy, cz)
			cx,cy,cz = self.moveit_runner_kitting.goto_pose(cx, cy, cz-0.002)
		
		print("going to agv")
		self.moveit_runner_kitting.goto_pose(-0.572989, 0, 2.0)	# waypoint to avoid crashing into conveyor
		self.moveit_runner_kitting.goto_pose(-2.265, 1.3676, 1.0)
		self.gm.deactivate_gripper()

		# resume conveyor
		print("next cycle")
		control_conveyor(100)

if __name__ == '__main__':

	kitting_group_names = ['kitting_arm']
	moveit_runner_kitting = MoveitRunner(kitting_group_names, ns='/ariac/kitting')

	kitting_arm = moveit_runner_kitting.groups['kitting_arm']
	kitting_arm.set_end_effector_link("vacuum_gripper_link")
	gm = GripperManager(ns='/ariac/kitting/arm/gripper/')

	# moveit_runner_kitting.goto_pose(-1.15, 0, 2)

	# control_conveyor(100)


	# source_frame = 'logical_camera_conveyor_frame'
	# target_frame = 'logical_camera_conveyor_assembly_battery_green_2_frame'
	# world_pose = tf2_listener.get_transformed_pose(source_frame, target_frame)
	# print(world_pose.pose.position)
	# exit()

	motionPlan = Follow_points(moveit_runner_kitting, gm)
	while True:
		motionPlan.main_body()
		print("state:", motionPlan.state)