#!/usr/bin/env python

import numpy as np
import math

def euler_to_quaternion(r):
    (yaw, pitch, roll) = (r[0], r[1], r[2])
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

def quaternion_to_euler(q):
    (x, y, z, w) = (q[0], q[1], q[2], q[3])
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]


if __name__ == '__main__':
    option = int(input("Enter 0 for euler -> quat, 1 for quat -> euler: "))

    if option == 0:
        roll = input("Enter your roll: ")
        pitch = input("Enter your pitch: ")
        yaw = input("Enter your yaw: ")
        (qx, qy, qz, qw) = euler_to_quaternion((yaw, pitch, roll))
        print((qx, qy, qz, qw))
    else:
        x = input("Enter x: ")
        y = input("Enter y: ")
        z = input("Enter z: ")
        w = input("Enter w: ")
        (yaw, pitch, roll) = quaternion_to_euler((x, y, z, w))
        print((roll, pitch, yaw))

