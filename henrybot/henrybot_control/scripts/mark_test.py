#!/usr/bin/env python3
import sys
from time import sleep

import rospy
from client_arm import ArmClient
import numpy as np

from mrsdl_joint_models import *

x0 = 0.0
y0 = 0.300
z0 = 0.0
dx = 0.10
dy = 0.10
dz = 0.15

pos_list = []
duration_list = []

#for i in range(3):
#    for j in range(3):
#        joints,err = xyz_to_joints(VERT_GRIP, ur5e_params, [ x0+i*dx, y0+j*dy, z0 ])
#        pos_list.append(joints)
#        joints,err = xyz_to_joints(VERT_GRIP, ur5e_params, [ x0+i*dx, y0+j*dy, z0+dz ])
#        pos_list.append(joints)
#duration_list = [ 5.0 + 5*i for i in range(len(pos_list)) ]
#duration_list[0] = 8.0


if __name__ == "__main__":
    rospy.init_node('test_move')
    arm = ArmClient()
    sleep(3)

    ##off_corner# joints,err = xyz_to_joints(VERT_GRIP, ur5e_params, [ x0+0.31, y0+0.13, z0-0.16 ])
    joints,err = xyz_to_joints(VERT_GRIP, ur5e_params, [ x0-0.10, y0, z0 ])
    pos_list = []
    pos_list.append(joints)
    duration_list = [8.0]
    arm.move(pos_list, duration_list)

    joints,err = xyz_to_joints(VERT_GRIP, ur5e_params, [ x0+0, y0, z0 ])
    pos_list = []
    pos_list.append(joints)
    duration_list = [1.0]
    arm.move(pos_list, duration_list)

    joints,err = xyz_to_joints(VERT_GRIP, ur5e_params, [ x0+0.10, y0, z0 ])
    pos_list = []
    pos_list.append(joints)
    duration_list = [1.0]
    arm.move(pos_list, duration_list)


