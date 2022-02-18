#!/usr/bin/env python3
import argparse
import json


def euclidean(a, b):
    if len(a) != len(b):
        assert ("Inputs of different dimensions")

    tmp_sum = 0
    for i in range(len(a)):
        tmp_sum += (b[i] - a[i]) ** 2

    return tmp_sum ** 0.5


def overlap(a, b):
    # swap if out of order
    if a[0] > b[0]:
        tmp = a
        a = b
        b = tmp

    return a[1] > b[0]


class KittingRobot:
    def __init__(self, name, pose, orient, orient_range, max_r, id_count):
        self.type = 'kitting'
        self.name = name
        self.pose = pose  # center pose - [x,y,z]
        self.orient = orient  # direction in which rail runs: 0=x, 1=y
        self.orient_range = orient_range  # range in its oriented direction (aka rail length)
        self.max_r = max_r
        self.shape = Cylinder(pose, orient, orient_range, max_r)
        self.id = id_count

    # Returns boolean indicating whether there exists intersection with a Gantry Robot
    def intersect_w_gantry(self, GantryObject):
        pass

    def intersect_w_conveyor(self, ConveyorObject):
        # ASSUMPTION FOR NOW: kitting and conveyor lie in same direction
        # check if oriented direction overlaps
        if not overlap(self.orient_range, ConveyorObject.orient_range):
            return False

        # check if other two directions overlap (for now, if line lies inside cylinder -> if point inside circle)
        return euclidean(self.shape.circle_center, ConveyorObject.shape.point_2d) < self.shape.r

    def intersect_w_agv(self, AGVObject):
        # ASSUMPTION: only kitting can intersect with starting point
        # check start point's vertical line intersects with kitting cylinder
        return self.shape.intersect_w_line(AGVObject.start_shape)  # TODO: implement intersect_w_line func


class GantryRobot:
    def __init__(self, name, pose, x_rail_range, y_rail_range, arm_r, id_count):
        self.type = 'gantry'
        self.name = name
        self.pose = pose
        self.x_rail_range = x_rail_range  # [x_min, x_max]
        self.y_rail_range = y_rail_range  # [y_min, y_max]
        self.arm_r = arm_r
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
        self.pose = pose
        self.start_shape = Line(pose, 2, [0.81, 2])  # starting pose's vertical line
        self.shape = []  # list of vertical lines
        for i in dst:
            self.shape.append(Line(i, 2, [0.81, 2]))
        self.id = id_count

        # For usage in AGV_module
        self.used = False
        self.num_items = 0


class ConveyorRobot:
    def __init__(self, name, pose, orient, orient_range, id_count):
        self.type = 'conveyor'
        self.name = name
        self.pose = pose  # center pose [x,y,z]
        self.orient = orient  # 0 = runs along x-direction, 1 = runs along y-direction
        self.orient_range = orient_range
        self.height = 0.90
        # self.dim = dim   # [x length, y length, z height]
        # compute line for this (see intersect_kitting_conveyor for formatting)
        self.shape = Line([pose[0], pose[1], self.height], orient, orient_range)
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
        self.num_v = len(edges)  # number of vertices, ordered 0,1,2,...,v-1
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


def build_graph(robotObjects):
    e = []  # index corresponds to robot_id
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
        e[k_obj.id] = tmp_e

    # detect AGV -> gantry edges
    for agv_obj in robotObjects[2]:
        tmp_e = []
        for g_obj in robotObjects[1]:
            if g_obj.intersect_w_agv(agv_obj):
                tmp_e.append(g_obj.id)
        e[agv_obj.id] = tmp_e

    return Graph(e)


def connectivity(robotObjects):
    g = build_graph(robotObjects)
    connected = set()
    for c_obj in robotObjects[3]:
        s = g.dfs(c_obj.id)

        # Merge sets
        for i in s:
            connected.add(i)

    return len(connected) == num_robots


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


def parse_json(f):
    data = json.load(f)

    robotObjects = [[], [], [], []]
    ur10_upper_arm_len = 0.612
    ur10_forearm_len = 0.5723
    id_count = 0
    for i in data['robots']:
        if i['type'] == 'kitting':
            json_kitting_check(i)
            kitting_arm_range = ur10_upper_arm_len + ur10_forearm_len
            robotObjects[0].append(
                KittingRobot(i['name'], i['pose'], i['rail_dim'], i['rail_range'], kitting_arm_range, id_count))
            id_count += 1
        elif i['type'] == 'gantry':
            json_gantry_check(i)
            gantry_arm_range = ur10_upper_arm_len + ur10_forearm_len
            robotObjects[1].append(
                GantryRobot(i['name'], i['pose'], i['x_rail_range'], i['y_rail_range'], gantry_arm_range, id_count))
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


def print_robot_list(robotObjects):
    # Debug function
    for i in robotObjects:
        for j in i:
            print("id = %d, type = %s, name = %s" % (j.id, j.type, j.name))


def main(argv):
    robot_objects = parse_json(argv.json_file)

    print_robot_list(robot_objects)

    print('connected? ', connectivity(robot_objects))
    if not connectivity(robot_objects):
        print('Missing a link in Conveyor -> Kitting -> AGV -> Gantry pathway.'
              'A connected controller cannot be generated.')
        return


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Static analysis on a floor plan json file")
    parser.add_argument('json_file', type=argparse.FileType('r'))
    main(parser.parse_args())
