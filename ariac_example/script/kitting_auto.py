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
import os

# for multithreading
import threading
import Queue

# Battery z-value grab heights on conveyor
BATTERY_HEIGHT = 0.03
SENSOR_HEIGHT = 0.048
REGULATOR_HEIGHT = 0.05

def start_competition():
    """ Start the competition through ROS service call """

    rospy.wait_for_service('/ariac/start_competition')
    rospy.ServiceProxy('/ariac/start_competition', Trigger)()

def competition_state():
	data = rospy.wait_for_message('/ariac/competition_state', String)
	print(data)
	return data

def get_order():
    """ Get the current order from the /ariac/orders topic"""

    order = rospy.wait_for_message('/ariac/orders', Order)
    return order

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

class Conveyor_Sensor_module():
	def __init__(self):
		self.conveyor_state = 0
		self.target = ""
	def main_body(self, q, t):
		# Conveyor State 0: Pre-competition (stopped)
		if self.conveyor_state == 0:
			if competition_state().data == "init":
				start_competition()	# conveyor begins to move
			self.conveyor_state = 1
			return False

		# Conveyor State 1: Waiting for a needed item (moving)
		if self.conveyor_state == 1:
			detected = False
			while True:
				models_detected = get_logical_camera_conveyor_data().models
				if len(models_detected) == 0:
					rospy.sleep(0.05)	# how to do this with pthread cond wait?
					continue
				for m in models_detected:
					# if m.type in self.items_needed and self.items_needed[m.type] > 0:
					if m.type in order and order[m.type] > 0:
						print("here")
						self.target = m.type
						detected = True
						break
				if detected:
					break
			control_conveyor(0)
			self.conveyor_state = 2	# conveyor paused
			# self.kitting_state = 2	# kitting arm moves down to grab item
			t.append(self.target)
			return False

		# Conveyor State 2: conveyor paused
		if self.conveyor_state == 2:
			# stay in this state until we get a signal from kitting FSM
			while True:
				msg = q.get()
				# operation finished, exit thread
				if msg == "done":
					return True
				elif msg == "run":
					break
				else:
					rospy.sleep(0.05)
			
			control_conveyor(100)
			self.conveyor_state = 1
			return False

AGV_TOP = -2.35								# x-value of top row on AGVs
AGV_LEFT = [4.5, 1.19, -1.51, -4.88]		# y-value of left cols on AGVs (1-4)
AGV_ROW_SPACE = 0.24	# x-value of dist between rows on AGV
AGV_COL_SPACE = 0.18	# y-value of dist between cols on AGV

class AGV_module():
	def __init__(self):
		self.used_agvs = [False, False, False, False]
		self.agv_num_items = [0, 0, 0, 0]
	
	# Returns x-y coordinates of next available spot on specified AGV
	def get_new_loc(self, agv_num):
		if agv_num == 1:
			return (AGV_TOP + AGV_ROW_SPACE*math.floor(self.agv_num_items[0]/3), AGV_LEFT[0] + AGV_COL_SPACE*(self.agv_num_items[0] % 3))
		elif agv_num == 2:
			return (AGV_TOP + AGV_ROW_SPACE*math.floor(self.agv_num_items[1]/3), AGV_LEFT[1] + AGV_COL_SPACE*(self.agv_num_items[1] % 3))
		elif agv_num == 3:
			return (AGV_TOP + AGV_ROW_SPACE*math.floor(self.agv_num_items[2]/3), AGV_LEFT[2] + AGV_COL_SPACE*(self.agv_num_items[2] % 3))
		else:
			return (AGV_TOP + AGV_ROW_SPACE*math.floor(self.agv_num_items[3]/3), AGV_LEFT[3] + AGV_COL_SPACE*(self.agv_num_items[3] % 3))
	
	def update_agv_info(self, agv_num):
		self.agv_num_items[agv_num-1] += 1

	def update_agv_done(self, agv_num):
		self.used_agvs[agv_num-1] = True



class Follow_points():
	def __init__(self, moveit_runner_kitting, gm):
		self.moveit_runner_kitting = moveit_runner_kitting
		self.gm = gm
		self.kitting_state = 0
		# self.conveyor_state = 0
		self.agvs = [0, 0, 0, 0]	# 0 = unused, 1 = used
		with open("/home/dxwu2/araic_ws/src/ARIAC/nist_gear/config/user_config/sample_user_config.yaml", "r") as f:
			data = yaml.load(f)
			# breakbeam_conveyor_pose_xyz = data['sensors']['breakbeam_conveyor']['pose']['xyz']
		self.sensor_file = data
		# self.items_needed = order
		self.target = ""

	def main_body(self, q, t, agvs):
		# Start state for kitting robot: Kitting robot may be out of position
		if self.kitting_state == 0:
			# completed order
			# if sum(self.items_needed.values()) == 0:
			if sum(order.values()) == 0:
				self.kitting_state = 5	# ready to move AGV
				return False

			start_pose = [-1.15, 0, 2]
			self.moveit_runner_kitting.goto_pose(start_pose[0], start_pose[1], start_pose[2])
			
			# verify if there, then change kitting state
			cx = kitting_arm.get_current_pose().pose.position.x
			cy = kitting_arm.get_current_pose().pose.position.y
			cz = kitting_arm.get_current_pose().pose.position.z
			if abs(cx - start_pose[0]) + abs(cy - start_pose[1]) + abs(cz - start_pose[2]) <= 0.01:
				self.kitting_state = 1 	# robot now in position, ready to grab items
			return False

		# Kitting State 1: Robot moves arm toward battery
		if self.kitting_state == 1:
			self.gm.activate_gripper()

			# Use Logical Camera to get pose of battery(s)
			# data = get_logical_camera_conveyor_data()[0]

			# todo: add functionality to grab all batteries detected by camera (one at a time)

			# wait for sensor/conveyor module to send target item
			while len(t) == 0:
				rospy.sleep(0.05)
			self.target = t.pop(0)
			print("in kitting mod:", self.target)
			
			# if self.target == "":
			# 	self.kitting_state = 1
			# 	return False

			# read from trial config file to change hardcoded value
			for i in range(1,11):
				source_frame = 'logical_camera_conveyor_frame'
				target_frame = 'logical_camera_conveyor_' + self.target + '_' + str(i) + '_frame'
				# target_frame = 'logical_camera_conveyor_' + self.target + '_' + str(i) + '_frame'
				# target_frame = 'logical_camera_conveyor_assembly_battery_green_' + str(i) + '_frame'
				world_pose = tf2_listener.get_transformed_pose(source_frame, target_frame)
				if world_pose is not None:
					break

			if world_pose is None:
				self.kitting_state = 0
				return False

			item_height = ""
			if "battery" in self.target:
				item_height = BATTERY_HEIGHT
			elif "sensor" in self.target:
				item_height = SENSOR_HEIGHT
			else:
				item_height = REGULATOR_HEIGHT
			self.moveit_runner_kitting.goto_pose(world_pose.pose.position.x, world_pose.pose.position.y, world_pose.pose.position.z + item_height)
			self.kitting_state = 2
			return False
		
		# Kitting State 2: Lowering toward battery
		if self.kitting_state == 2:
			cx = kitting_arm.get_current_pose().pose.position.x
			cy = kitting_arm.get_current_pose().pose.position.y
			cz = kitting_arm.get_current_pose().pose.position.z
			self.moveit_runner_kitting.goto_pose(cx, cy, cz - 0.002)
			if self.gm.is_object_attached():
				self.kitting_state = 3
			return False

		# State 3: Moving to AGV
		if self.kitting_state == 3:
			self.moveit_runner_kitting.goto_pose(-0.572989, 0, 2.0)	# waypoint to avoid crashing into conveyor
			q.put("run")	# resume conveyor belt

			(drop_x, drop_y) = agvs.get_new_loc(2)
			self.moveit_runner_kitting.goto_pose(drop_x, drop_y, 1.0)
			
			# verify if there, then change state
			cx = kitting_arm.get_current_pose().pose.position.x
			cy = kitting_arm.get_current_pose().pose.position.y
			cz = kitting_arm.get_current_pose().pose.position.z
			if abs(cx - drop_x) + abs(cy - drop_y) + abs(cz - 1.0) <= 0.01:
				self.kitting_state = 4
			return False

		# State 4: Drop battery
		if self.kitting_state == 4:
			self.gm.deactivate_gripper()
			# self.items_needed[self.target] -= 1
			order[self.target] -= 1
			print(order)

			agvs.update_agv_info(2)

			self.kitting_state = 0
			return False

		# State 5: Transport to Gantry Robot
		if self.kitting_state == 5:
			rospy.sleep(2.0)
			move_agvs('agv2', 'as1')
			q.put("done")

			agvs.update_agv_done(2)

			return True

def conveyor_loop(q, t):
	conveyorPlan = Conveyor_Sensor_module()
	lastState = 0
	while conveyorPlan.main_body(q, t) is False:
		curState = conveyorPlan.conveyor_state
		if curState != lastState:
			print("conveyor state:", conveyorPlan.conveyor_state)
		lastState = curState


def kitting_loop(q, t):
	motionPlan = Follow_points(moveit_runner_kitting, gm)
	agvs = AGV_module()
	lastState = 0
	while motionPlan.main_body(q, t, agvs) is False:
		curState = motionPlan.kitting_state
		if curState != lastState:
			print("kitting state:", motionPlan.kitting_state)
		lastState = curState

if __name__ == '__main__':

	kitting_group_names = ['kitting_arm']
	moveit_runner_kitting = MoveitRunner(kitting_group_names, ns='/ariac/kitting')

	kitting_arm = moveit_runner_kitting.groups['kitting_arm']
	kitting_arm.set_end_effector_link("vacuum_gripper_link")
	gm = GripperManager(ns='/ariac/kitting/arm/gripper/')

    # Read in data from JSON file (replace with actual code later)
    kitting_start_x = -1.3  # eventually, will want to set kitting.urdf.xacro file with this info
    kitting_start_y = 0
    kitting_start_z = 0.9

    # Retrieve robot arm parameters -- assuming this isn't part of user input
    upper_arm_length = 0.612
    forearm_length = 0.5723
    max_radius = upper_arm_length + forearm_length

    # Boot up simulation (Gazebo) with the auto-generated controller
    
