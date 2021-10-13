#!/usr/bin/env python3
import sys

import rospy
import geometry_msgs.msg as geometry_msgs
from client_arm import ArmClient
import numpy as np 


def pos_to_rad(tp_pos):
    return np.array(tp_pos)*np.pi/180.

def add_pos(pos, list,times):
    list.append(pos)
    times.append(times[-1]+5)
    return list, times

if __name__ == "__main__":


    rospy.init_node('UR5e')
    ##
    client = ArmClient()
    ##
    trajectory_type = client.choose_controller("forward_joint_traj_controller")
    # trajectory_type = client.choose_controller("pos_joint_traj_controller")
    ##
    position_list = []
    #duration_list = []

    position_list.append([-1.2, -1.57, -0.5, 0, 0, 0])
    position_list.append([-1.62316, -1.528908, -2.131047, -1.052434, 1.51844, -1.677261])
    position_list.append([-1.680752, -2.26893, -1.11701, -1.27409,1.6057, -0.139626])
    position_list.append([-1.06465, -2.26893, -1.11701, -1.39626, 1.48353, 0.523599])
    position_list.append([-1.2, -1.57, -0.5, 0, 0, 0])
    # position_list.append([-2.131047, -1.528908, -1.62316, -1.052434, 1.51844, -1.677261])
    # position_list.append([-1.11701, -2.26893, -1.680752, -1.27409,1.6057, -0.139626])
    # position_list.append([-1.11701, -2.26893, -1.06465, -1.39626, 1.48353, 0.523599])
    # position_list.append([-0.5, -1.57, -1.2, 0, 0, 0])

    duration_list = [1.0, 4.0, 7.0, 9.0, 13.0]
# -93.1 -87.6 -122.1 -60.3 87 -96.1
# -96.3 -130 -64 -73 92 -8
# -61 -132 -64 -80 85 30 
    client.send_joint_trajectory(position_list, duration_list)