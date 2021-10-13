#!/usr/bin/env python3
import sys

import rospy
import geometry_msgs.msg as geometry_msgs
from client_arm import ArmClient



if __name__ == "__main__":


    rospy.init_node('UR5e')
    ##
    client = ArmClient()
    ##
    trajectory_type = client.choose_controller("forward_joint_traj_controller")
    ##

    position_list = [[0, -1.57, -1.57, 0, 0, 0]]
    position_list.append([0.2, -1.57, -1.57, 0, 0, 0])
    position_list.append([-0.5, -1.57, -1.2, 0, 0, 0])
    duration_list = [3.0, 7.0, 10.0]

    client.send_joint_trajectory(position_list, duration_list)