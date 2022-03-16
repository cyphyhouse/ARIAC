#!/usr/bin/env python

import rospy
import tf2_ros
import moveit_commander as mc
import math
import yaml
from yaml.loader import SafeLoader

import tf2_listener
import json

import geometry_msgs.msg
from tf.transformations import euler_from_quaternion
from nist_gear.msg import VacuumGripperState, Proximity, LogicalCameraImage
from nist_gear.srv import VacuumGripperControl, ConveyorBeltControl
from std_srvs.srv import Trigger
from std_msgs.msg import String

# from scipy.spatial import distance

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
	return data

def get_order():
	""" Get the current order from the /ariac/orders topic"""

	order = rospy.wait_for_message('/ariac/orders', Order)
	return order

def move_agvs(agvObject, dest):
	if agvObject.name == 'agv1' or agvObject.name == 'agv2':
		if dest == 'near':
			dest = 'as1'
		else:
			dest = 'as2'
	elif agvObject.name == 'agv3' or agvObject.name == 'agv4':
		if dest == 'near':
			dest = 'as3'
		else:
			dest = 'as4'
	else:
		print("Trying to move invalid AGV name. AGV name must be either agv1, agv2, agv3, or agv4.")
		exit()
	rospy.wait_for_service('/ariac/' + agvObject.name + '/to_' + dest)
	rospy.ServiceProxy('/ariac/' + agvObject.name + '/to_' + dest, Trigger)()

	# update AGV current position
	a = agvObject.destinations[0]
	b = agvObject.destinations[1]
	if dest == 'near':
		agvObject.cur_pos = a if new_euclidean_dist(a, agvObject.cur_pos) < new_euclidean_dist(b, agvObject.cur_pos) else b
	else:
		agvObject.cur_pos = a if new_euclidean_dist(a, agvObject.cur_pos) > new_euclidean_dist(b, agvObject.cur_pos) else b


def get_breakbeam_sensor_data():
	data = rospy.wait_for_message('/ariac/breakbeam_conveyor', Proximity)
	return data
def get_breakbeam_flat_sensor_data():
	data = rospy.wait_for_message('/ariac/breakbeam_conveyor_flat', Proximity)
	return data
def get_logical_camera_conveyor_data():
	data = rospy.wait_for_message('/ariac/logical_camera_conveyor', LogicalCameraImage)
	return data
def get_logical_camera_agv_as_data(agv_num, as_num):
	rostopic_str = '/ariac/logical_camera_agv' + str(agv_num) + '_as' + str(as_num)
	data = rospy.wait_for_message(rostopic_str, LogicalCameraImage)
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

def clear_briefcase():
	# for now, just clears all briefcases
	rospy.wait_for_service('/ariac/briefcase_1/clear_briefcase')
	clear_briefcase_rosservice = rospy.ServiceProxy('/ariac/briefcase_1/clear_briefcase', Trigger)()
	rospy.wait_for_service('/ariac/briefcase_2/clear_briefcase')
	clear_briefcase_rosservice = rospy.ServiceProxy('/ariac/briefcase_1/clear_briefcase', Trigger)()
	rospy.wait_for_service('/ariac/briefcase_3/clear_briefcase')
	clear_briefcase_rosservice = rospy.ServiceProxy('/ariac/briefcase_1/clear_briefcase', Trigger)()
	rospy.wait_for_service('/ariac/briefcase_4/clear_briefcase')
	clear_briefcase_rosservice = rospy.ServiceProxy('/ariac/briefcase_1/clear_briefcase', Trigger)()


# finds shoulder lift and elbow angle joints to achieve desired (coord1, coord2) pose
# Inputs : desired pose - desired pose of end effector, either (x,z) or (y,z)
#          robot_world_pose - shoulder link pose of kitting/gantry robot
def find_alphabeta(desired_pose, robot_world_pose):
	r1 = 0.61215	# range of motion of shoulder lift joint
	r2 = 0.57235	# range of motion of elbow joint
	#r1 = 0.573 # length of upper arm link (radius of its range of motion)
	#r2 = 0.400 # length of forearm link

	# CHANGE JSON: -1.3, 1.1264
	ab = new_euclidean_dist(robot_world_pose, desired_pose)
	# ab = euclidean_dist(-1.3, 1.1264, x, z) # dist from kitting base joint to desired (x,z) point (a + b in proof)
	# ab = distance.euclidean((-1.3, 1.1264), (x,z))   # ab is dist from kitting base joint to desired (x,z) point (a + b in proof)
	beta = law_cosines_gamma(r1, r2, ab)

	a1 = law_cosines_gamma(r1, ab, r2)
	a2 = math.acos((abs(desired_pose[0]+1.3))/ab)
	# alpha = alpha' + alpha'' if goal z is above start z
	if desired_pose[1] >= 1.1264:
		alpha = a1 + a2
	else:
		alpha = a1 - a2

	alpha = -alpha			# moving shoulder joint up is negative alpha direction
	beta = math.pi - beta	# we want complementary (beta is angle b/t two links, elbow joint is comp of this)
	# the above might need additional changes (e.g. abs val, etc) when trying to grab stuff on other side

	return (alpha, beta)

def find_alphabeta_gantry(desired_pose, robot_world_pose):
	r1 = 0.61215
	r2 = 0.57235
	
	ab = new_euclidean_dist(robot_world_pose, desired_pose)
	# current version always gets outward angle: alpha = alpha_0 + 2*alpha_prime
	# if want inward angle, use: alpha = alpha_0 and invert beta
	alpha_0 = math.acos((abs(desired_pose[1] - robot_world_pose[1]))/ab)
	alpha_prime = law_cosines_gamma(r1, ab, r2)
	alpha = alpha_0 + alpha_prime

	alpha = -alpha   # moving shoulder joint up is 
	beta = math.pi - law_cosines_gamma(r1, r2, ab)

	return (alpha, beta)
	

def euclidean_dist(x1, z1, x2, z2):
	return math.sqrt((x2-x1)**2 + (z2-z1)**2)

def euclidean(a, b):
	if len(a) != len(b):
		assert("Inputs of different dimensions")

	tmp_sum = 0
	for i in range(len(a)):
		tmp_sum += (b[i] - a[i])**2

	return tmp_sum**0.5

def new_euclidean_dist(a, b):
	if len(a) != len(b):
		raise Exception('Invalid input for euclidean distance!')
	
	squared_sum = 0
	for i in range(len(a)):
		squared_sum += (abs(b[i]-a[i]))**2

	return squared_sum**0.5

# Input - two ranges (e.g. a = [3,5], b = [4,8])
# Output - boolean
def overlap(a, b):
	# swap if out of order
	if a[0] > b[0]:
		tmp = a
		a = b
		b = tmp

	return a[1] > b[0]

def reachable(a, b):
	# Normal cases: y-coordinate of point is within linear rail range
	if b[1] < a[1] + rail_length and b[1] > a[1] - rail_length:
		return new_euclidean_dist(a[0::2], b[0::2]) <= max_radius
	# Edge cases: y-coordinate of point is outside linear rail range
	else:
		print("edge case!")
		# TODO: check 'hemispheres' outside linear rail range
		return False

# Uses law of cosines to find the angle (in radians) of the side opposite of c
def law_cosines_gamma(a, b, c):
	return math.acos((a**2 + b**2 - c**2)/(2*a*b))

def bounds_checking(x, z, robotObject):
	a = [x,z]
	b = robotObject.pose[0::2]
	return True if new_euclidean_dist(a, b) <= robotObject.max_r else False

class KittingRobot:
	def __init__(self, name, pose, orient, orient_range, max_r, id_count):
		self.type = 'kitting'
		self.name = name
		self.pose = pose           # center pose - [x,y,z]
		self.orient = orient   # direction in which rail runs: 0=x, 1=y
		self.orient_range = orient_range   # range in its oriented direction (aka rail length)
		self.max_r = max_r
		self.shape = Cylinder(pose, orient, orient_range, max_r)
		self.reachable_agvs = []   # list of AGVObjects 
		self.id = id_count
		
	# Returns boolean indicating whether there exists intersection with a Gantry Robot
	def intersect_w_gantry(self, GantryObject):
		pass
	
	def intersect_w_conveyor(self, ConveyorObject):
		# ASSUMPTION FOR NOW: kitting and conveyor lie in same direction
		# check if oriented direction overlaps
		if not overlap(self.orient_range, ConveyorObject.orient_range):
			return false

		# check if other two directions overlap (for now, if line lies inside cylinder -> if point inside circle)
		return euclidean(self.shape.circle_center, ConveyorObject.shape.point_2d) < self.shape.r

	def intersect_w_agv(self, AGVObject):
		# ASSUMPTION: only kitting can intersect with starting point
		# check start point's vertical line intersects with kitting cylinder
		return self.shape.intersect_w_line(AGVObject.start_shape) # TODO: implement intersect_w_line func

class GantryRobot:
	def __init__(self, name, pose, x_rail_range, y_rail_range, arm_r, id_count):
		self.type = 'gantry'
		self.name = name
		self.pose = pose
		self.x_rail_range = x_rail_range   # [x_min, x_max]
		self.y_rail_range = y_rail_range   # [y_min, y_max]
		self.arm_r = arm_r
		self.torso_to_arm = 0.637
		self.shoulder_ee_last_dim = 0.16   # if rot = 0, pi: x-dim; if rot = pi/2, -pi/2: y-dim
		self.id = id_count

	# TODO
	# ASSUMPTION: since AGV -> gantry for now, gantry can only intersect w one of dest vert. lines
	def intersect_w_agv(self, AGVObject):
		return True
		
class AGVRobot:
	def __init__(self, name, pose, dst, id_count):
		# ASSUMPTION: z-range hardcoded for now (twice in this function)
		self.type = 'agv'
		self.name = name
		self.cur_pos = pose
		self.cur_state = 'start'   # start, near, or far
		self.start_shape = Line(pose, 2, [0.81, 2])   # starting pose's vertical line
		self.destinations = []
		self.shape = []   # list of vertical lines
		for i in dst:
			self.destinations.append(i)
			self.shape.append(Line(i, 2, [0.81,2]))
		self.id = id_count

		# For usage in AGV_module -> nvm
		self.ready = False   # signal denotes ready to move?
		self.used = False
		self.num_items = 0
		self.carrying_items = {}   # item : quantity
		
class ConveyorRobot:
	def __init__(self, name, pose, orient, orient_range, id_count):
		self.type = 'conveyor'
		self.name = name
		self.pose = pose   # center pose [x,y,z]
		self.orient = orient   # 0 = runs along x-direction, 1 = runs along y-direction
		self.orient_range = orient_range
		self.height = 0.90	
		# self.dim = dim   # [x length, y length, z height]
		# compute line for this (see intersect_kitting_conveyor for formatting)
		self.shape = Line([pose[0],pose[1],self.height], orient, orient_range)
		self.id = id_count

class Line:
	def __init__(self, pose, orient, orient_range):
		self.orient = orient
		self.orient_range = orient_range
		self.point_2d = self.get_point(pose)
	
	def get_point(self, x):
		point = []
		for dim in range(3):
			if dim != self.orient:
				point.append(x[dim])
		return point

class Cylinder:
	def __init__(self, pose, orient, orient_range, max_r):
		self.orient = orient
		self.orient_range = orient_range
		self.circle_center = self.get_point(pose)
		self.r = max_r

	def get_point(self, x):
		point = []
		for dim in range(3):
			if dim != self.orient:
				point.append(x[dim])
		return point

	# TODO: incomplete. Possibly simply to boxes
	def intersect_w_line(self, line):
		# if both are oriented the same way, check:
		# 1) line's point is inside cylinder's circle
		# 2) if line and cylinder's ranges intersect
		if self.orient == line.orient:
			return True
		# if not oriented same way, 
		else:
			return True

class Graph:
	# edges - connectivity list (2D array) of robotObjects -> [[a,b,c],[],[a,d]] -> robot_id=0 is connected to robotObjects a,b,c
	def __init__(self, edges):
		self.num_v = len(edges)   # number of vertices, ordered 0,1,2,...,v-1
		self.e = edges
	
	# Runs DFS starting on specified vertex/robotObject (rep. by robot_id)
	# Input: start_v - robot_id of robotObject we want to run DFS starting at
	# ASSUMPTION: conveyor -> kitting -> AGV -> conveyor
	def dfs(self, start_v):
		seen = set()
		seen.add(start_v)
		self.dfs_helper(start_v, self.e, seen)
		return seen
	
	# Input: cur - robot_id of robotObject we are currently at in DFS
	# Output: none
	# Side effects: updates seen set
	def dfs_helper(self, cur, edge_list, seen):
		for r in edge_list[cur]:
			if r not in seen:
				seen.add(r)
				self.dfs_helper(r, edge_list, seen)
		return

# Input: robotObjects - a 2D list of robot objects (ordered KittingObject, GantryObject, AGVObject, ConveyorObject)
# ASSUMPTION: for now, assuming conveyor -> kitting -> AGV -> gantry
def build_graph(robotObjects):
	e = []   # index corresponds to robot_id
	for i in range(num_robots):
		e.append([])

	# detect conveyor -> kitting edges
	for c_obj in robotObjects[3]:
		tmp_e = []
		for k_obj in robotObjects[0]:
			if k_obj.intersect_w_conveyor(c_obj):
				tmp_e.append(k_obj.id)
		e[c_obj.id] = tmp_e

	# detect kitting -> AGV edges
	for k_obj in robotObjects[0]:
		tmp_e = []
		for agv_obj in robotObjects[2]:
			if k_obj.intersect_w_agv(agv_obj):
				tmp_e.append(agv_obj.id)
				k_obj.reachable_agvs.append(agv_obj)
		e[k_obj.id] = tmp_e

	# detect AGV -> gantry edges
	for agv_obj in robotObjects[2]:
		tmp_e = []
		for g_obj in robotObjects[1]:
			if g_obj.intersect_w_agv(agv_obj):
				tmp_e.append(g_obj.id)
		e[agv_obj.id] = tmp_e

	return Graph(e)

# Input : robotObjects - a 2D list of robot objects (ordered KittingObject, GantryObject, AGVObject, ConveyorObject)
def connectivity(robotObjects):
	g = build_graph(robotObjects)
	connected = set()
	for c_obj in robotObjects[3]:
		s = g.dfs(c_obj.id)

		# Merge sets
		for i in s:
			connected.add(i)

	return len(connected) == num_robots

# Check if JSON input is valid
def json_kitting_check(data):
	if len(data['pose']) != 3:
		raise Exception("Kitting Robot ", data['name'], " has incorrectly formatted pose.")
	if data['rail_dim'] != 'x' and data['rail_dim'] != 'y':
		raise Exception("Kitting Robot ", data['name'], " has incorrectly formatted rail_dim.")
	if len(data['rail_range']) != 2 or data['rail_range'][0] > data['rail_range'][1]:
		raise Exception("Kitting Robot ", data['name'], " has incorrectly formatted rail_range")
def json_gantry_check(data):
	if len(data['pose']) != 3:
		raise Exception("Gantry Robot ", data['name'], " has incorrectly formatted pose.")
	if len(data['x_rail_range']) != 2 or data['x_rail_range'][0] > data['x_rail_range'][1]:
		raise Exception("Gantry Robot ", data['name'], " has incorrectly formatted x_rail_range.")
	if len(data['y_rail_range']) != 2 or data['y_rail_range'][0] > data['y_rail_range'][1]:
		raise Exception("Gantry Robot ", data['name'], " has incorrectly formatted _rail_range.")
def json_agv_check(data):
	if len(data['pose']) != 3:
		raise Exception("AGV Robot ", data['name'], " has incorrectly formatted pose.")     
def json_conveyor_check(data):
	if len(data['pose']) != 3:
		raise Exception("Conveyor Robot ", data['name'], " has incorrectly formatted pose.")
	if data['orient'] != 'x' and data['orient'] != 'y':
		raise Exception("Conveyor Robot ", data['name'], " has invalid orient.")
	if len(data['orient_range']) != 2:
		raise Exception("Conveyor Robot ", data['name'], " has invalid length")

# JSON parser function
# Input: JSON file
# Output: robotObjects - 2D array of RobotObjects (ordered Kitting, Gantry, AGV, Conveyor)
def parse_json(file):
	f = open(file)
	data = json.load(f)

	robotObjects = [[],[],[],[]]
	ur10_upper_arm_len = 0.612
	ur10_forearm_len = 0.5723
	id_count = 0
	for i in data['robots']:
		if i['type'] == 'kitting':
			json_kitting_check(i)
			kitting_arm_range = ur10_upper_arm_len + ur10_forearm_len
			robotObjects[0].append(KittingRobot(i['name'], i['pose'], i['rail_dim'], i['rail_range'], kitting_arm_range, id_count))
			id_count += 1
		elif i['type'] == 'gantry':
			json_gantry_check(i)
			gantry_arm_range = ur10_upper_arm_len + ur10_forearm_len
			robotObjects[1].append(GantryRobot(i['name'], i['pose'], i['x_rail_range'], i['y_rail_range'], gantry_arm_range, id_count))
			id_count += 1
		elif i['type'] == 'agv':
			json_agv_check(i)
			robotObjects[2].append(AGVRobot(i['name'], i['pose'], i['destinations'], id_count))
			id_count += 1
		elif i['type'] == 'conveyor':
			json_conveyor_check(i)
			robotObjects[3].append(ConveyorRobot(i['name'], i['pose'], i['orient'], i['orient_range'], id_count))
			id_count += 1
		else:
			raise Exception("Error: Robot type ", i['type'], " does not exist")
	
	global num_robots
	num_robots = id_count

	return robotObjects

# Debug function
def print_robot_list(robotObjects):
	for i in robotObjects:
		for j in i:
			print("id = %d, type = %s, name = %s" % (j.id, j.type, j.name))

def pick_place(moveit_runner_kitting, moveit_runner_gantry, kitting_gm, gantry_gm):
	while True:
		src = input("\nEnter pick (source) pose in form (x, y, z): ")
		if len(src) == 3:
			break
		print("Input formatted incorrectly. Please try again")
	
	while True:
		dst = input("\nEnter place (dest) pose in form (x, y, z): ")
		if len(dst) == 3:
			break
		print("Input formatted incorrectly. Please try again")

	# Check if pick-and-place operation is possible under the given constraints
	if not reachable(kitting_base, src):
		print("Pick (source) location is out of kitting robot's range.")
		exit()
	
	if not reachable(kitting_base, dst):
		print("Place (dest) location is out of kitting robot's range.")
		exit()

	# Boot up simulation (Gazebo) with the auto-generated controller
	moveit_runner_kitting.goto_pose(src[0], src[1], src[2])
	kitting_gm.activate_gripper()
	moveit_runner_kitting.goto_pose(dst[0], dst[1], dst[2])
	kitting_gm.deactivate_gripper()

def get_item_height(target):
	if "battery" in target:
		return BATTERY_HEIGHT
	elif "sensor" in target:
		return SENSOR_HEIGHT
	else:
		return REGULATOR_HEIGHT


class MoveitRunner():
	def __init__(self, group_names, robotObject, node_name='move_kitting',
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
		self.robotObject = robotObject

	def goto_pose(self, x, y, z):

		# TODO: incorporate roll pitch yaw in user input
		orientation_k = kitting_arm.get_current_pose().pose.orientation
		(roll, pitch, yaw) = euler_from_quaternion([orientation_k.x, orientation_k.y, orientation_k.z, orientation_k.w]) 

		# Bounds checking
		if self.robotObject.orient == 'y':
			if y > self.robotObject.orient_range[1] or y < self.robotObject.orient_range[0]:
				print("goto_pose error: outside y-range bounds")
				return False
			if not bounds_checking(x, z, self.robotObject):
				print("goto_pose error: outside x-z range bounds")
				return False
		else:
			if x > self.robotObject.orient_range[1] or x < self.robotObject.orient_range[0]:
				print("goto_pose error: outside x-range bounds")
				return False
			if not bounds_checking(y, z, self.robotObject):
				print("goto_pose error: outside y-z range bounds")
				return False

		# conveyor_side = True if x >= -1.3 else False
		conveyor_x = robotObjects[3][0].pose[0]
		conveyor_side = True if x >= conveyor_x-0.2 else False	# 0.2 is approximate of conveyor width

		# Finding alpha (shoulder lift angle) and beta (elbow joint angle)
		if conveyor_side:
			alpha, beta = find_alphabeta([x-0.1158, z+0.1], self.robotObject.pose[0::2])	# adjust these values to account for wrist lengths
		else:
			alpha, beta = find_alphabeta([x+0.1154, z+0.1], self.robotObject.pose[0::2])

		cur_joint_pose = moveit_runner_kitting.groups['kitting_arm'].get_current_joint_values()

		print('debug in goto_pose')
		print(cur_joint_pose)

		# linear arm actuator
		if not conveyor_side:
			cur_joint_pose[0] = y + 0.1616191
		else:
			cur_joint_pose[0] = y - 0.1616191

		# shoulder pan joint
		if x < conveyor_x-0.2:
			cur_joint_pose[1] = 3.14
		else:
			cur_joint_pose[1] = 0

		# shoulder lift (alpha) and elbow (beta)
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

	# torso_rotation: angle (in radians) of torso rotation in CCW orientation
	def gantry_goto_pose(self, pose):
		x, y, z, torso_rotation = pose
		# Bounds checking
		if x > self.robotObject.x_rail_range[1] or x < self.robotObject.x_rail_range[0]:
			print("gantry_goto_pose error: outside x-range bounds")
			return False
		if y > self.robotObject.y_rail_range[1]+0.4 or y < self.robotObject.y_rail_range[0]-0.4:
			print("gantry_goto_pose error: outside y-range bounds")
			return False
		# mult robot
		gantryRobot = robotObjects[1][0]
		if z > gantryRobot.pose[2] + gantryRobot.arm_r or z < gantryRobot.pose[2] - gantryRobot.arm_r:
			print("gantry_goto_pose error: outside z-range bounds")
		
		cur_arm_pose = moveit_runner_gantry.groups['gantry_arm'].get_current_joint_values()
		cur_torso_pose = moveit_runner_gantry.groups['gantry_torso'].get_current_joint_values()
		cur_gantry_pose = moveit_runner_gantry.groups['gantry_full'].get_current_joint_values()

		# torso rails
		cur_gantry_pose[0] = x + gantryRobot.torso_to_arm * math.sin(torso_rotation) + 2   # +2 for difference between joint_value and x-coord
		cur_gantry_pose[1] = (-1 * y) + gantryRobot.torso_to_arm * math.cos(torso_rotation)
		# account for third-dimension (x for rot = 0, pi; y for rot = pi/2, -pi/2)
		# cur_gantry_pose[0] = x + gantryRobot.shoulder_ee_last_dim*math.sin(torso_rotation)   # assuming pan_joint = 0
		if torso_rotation == 0:
			cur_gantry_pose[0] += gantryRobot.shoulder_ee_last_dim
		elif torso_rotation == math.pi or torso_rotation == -math.pi:
			cur_gantry_pose[0] -= gantryRobot.shoulder_ee_last_dim
		elif torso_rotation == math.pi/2:
			cur_gantry_pose[1] -= gantryRobot.shoulder_ee_last_dim
		elif torso_rotation == -math.pi/2:
			cur_gantry_pose[1] += gantryRobot.shoulder_ee_last_dim
		else:
			print("gantry_goto_pose error: Arbitrary torso rotation not yet implemented")
			exit()


		# small adjustment for torso rails to give arm enough room to operate
		if torso_rotation == 0:
			cur_gantry_pose[1] += 0.0
		elif torso_rotation == math.pi or torso_rotation == -math.pi:
			cur_gantry_pose[1] -= 0.0
		elif torso_rotation == math.pi/2:
			cur_gantry_pose[0] += 0.0
		elif torso_rotation == -math.pi/2:
			cur_gantry_pose[0] -= 0.0
		else:
			print("gantry_goto_pose error: Arbitrary torso rotation not yet implemented")
			exit()


		# Torso rotation
		cur_gantry_pose[2] = torso_rotation

		# for now, always keeping pan joint at 0
		cur_gantry_pose[3] = 0

		# shoulder lift (4) and elbow (5) joint
		w1_to_ee = [-0.115, -0.115, -0.1]   # distance from wrist 1 to ee (at torso rotation = 0)

		if torso_rotation == 0:
			alpha, beta = find_alphabeta_gantry([y-w1_to_ee[1], z-w1_to_ee[2]], [y, 2])
			# alpha, beta = find_alphabeta_gantry([y+0.1158, z+0.1], [y, 2])	  # adjustments for wrist1 -> ee difference
		elif torso_rotation == math.pi or torso_rotation == -math.pi:
			alpha, beta = find_alphabeta_gantry([y-(w1_to_ee[1] * -1), z-w1_to_ee[2]], [y, 2])
		elif torso_rotation == -math.pi/2:
			alpha, beta = find_alphabeta_gantry([x-w1_to_ee[0], z-w1_to_ee[2]], [x, 2])
		elif torso_rotation == math.pi/2:
			alpha, beta = find_alphabeta_gantry([x-(w1_to_ee[0] * -1), z-w1_to_ee[2]], [x, 2])
		else:
			print("gantry goto_pose error: Arbitrary torso rotation functionality not yet implemented")
			return False


		# affects x-z plane
		# elif torso_rotation == -math.pi/2 or torso_rotation == math.pi/2:
		# 	alpha, beta = find_alphabeta_gantry([x+0.1148, z+0.1], [x, 2])
		# else:
		# 	print("gantry goto_pose error: Arbitrary torso rotation functionality not yet implemented")
		# 	return False
		cur_gantry_pose[4] = alpha
		cur_gantry_pose[5] = beta

		# wrist 1 joint, to get flat ee: w1 = -(shoulder_lift + elbow)
		cur_gantry_pose[6] = -1*(cur_gantry_pose[4] + cur_gantry_pose[5])

		# wrist 2 joint - always pi/2 for now to keep it flat
		cur_gantry_pose[7] = math.pi/2

		# wrist 3 joint
		cur_gantry_pose[8] = 0

		print('going to:', cur_gantry_pose)
		self.groups['gantry_full'].go(cur_gantry_pose, wait=True)
		self.groups['gantry_full'].stop()

		return True
	
	def goto_agv1(self):
		cur_joint_pose = moveit_runner_kitting.groups['kitting_arm'].get_current_joint_values()

		cur_joint_pose[0] = 4.8
		cur_joint_pose[1] = math.pi
		cur_joint_pose[2] = -0.64
		cur_joint_pose[3] = 1.90
		cur_joint_pose[4] = -1*cur_joint_pose[2] - cur_joint_pose[3] - math.pi/2
		cur_joint_pose[5] = -math.pi/2
		cur_joint_pose[6] = 0


		self.groups['kitting_arm'].go(cur_joint_pose, wait=True)
		self.groups['kitting_arm'].stop()

	def goto_agv4(self):
		cur_joint_pose = moveit_runner_kitting.groups['kitting_arm'].get_current_joint_values()

		cur_joint_pose[0] = -4.8
		cur_joint_pose[1] = math.pi
		cur_joint_pose[2] = -0.64
		cur_joint_pose[3] = 1.90
		cur_joint_pose[4] = -1*cur_joint_pose[2] - cur_joint_pose[3] - math.pi/2
		cur_joint_pose[5] = -math.pi/2
		cur_joint_pose[6] = 0


		self.groups['kitting_arm'].go(cur_joint_pose, wait=True)
		self.groups['kitting_arm'].stop()

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

	global robotObjects
	robotObjects = parse_json(sys.argv[1])   # [kitting, gantry, agv, conveyor]

	kitting_group_names = ['kitting_arm']
	moveit_runner_kitting = MoveitRunner(kitting_group_names, robotObjects[0][0], ns='/ariac/kitting')
	kitting_arm = moveit_runner_kitting.groups['kitting_arm']
	kitting_arm.set_end_effector_link("vacuum_gripper_link")
	kitting_gm = GripperManager(ns='/ariac/kitting/arm/gripper/')

	gantry_group_names = ['gantry_full', 'gantry_arm', 'gantry_torso']
	moveit_runner_gantry = MoveitRunner(gantry_group_names, robotObjects[1][0], ns='/ariac/gantry')
	gantry_arm = moveit_runner_gantry.groups['gantry_arm']
	gantry_arm.set_end_effector_link("gantry_arm_vacuum_gripper_link")
	gantry_torso = moveit_runner_gantry.groups['gantry_torso']
	gantry_gm = GripperManager(ns='/ariac/gantry/arm/gripper/')

	# testing gantry goto pose
	moveit_runner_gantry.gantry_goto_pose([-5.469, -4.895, 0.82, -math.pi])
	exit()



	
