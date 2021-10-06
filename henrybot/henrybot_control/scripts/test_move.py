#!/usr/bin/env python3
import sys
from time import sleep

import rospy
import geometry_msgs.msg as geometry_msgs
from client_arm import ArmClient
from client_gripper import RobotiqGripper
from tf.transformations import quaternion_from_euler
import numpy as np

def angle_deg(rx,ry,rz):
    return quaternion_from_euler(rx*np.pi/180,ry*np.pi/180,rz*np.pi/180)

def tp_to_ros(x,y,z,rx,ry,rz):
    print(rx,ry,rz)
    q = angle_deg(rx,ry,rz)
    print(q)
    return geometry_msgs.Pose(geometry_msgs.Vector3(x/1000, y/1000, z/1000 + 0.4), geometry_msgs.Quaternion(q[0],q[1],q[2],q[3]))

# orig_pose_list = [
#         tp_to_ros(400,-100,0,0,0,0),
#         tp_to_ros(400,-100,200,0,0,0),
#         tp_to_ros(400,300,200,0,0,0),
#         tp_to_ros(400,300,0,0,0,0),
#         tp_to_ros(400,-100,0,0,0,0),
#         ]
# orig_duration_list = [5.0, 7.0, 10.0, 13.0, 16.0]

pose_list = [
    geometry_msgs.Pose(geometry_msgs.Vector3(550/1000, -575/1000, -250/1000 + 0.4), geometry_msgs.Quaternion(0.769365, 0.1440758, -0.6051185, -0.1454344)),
    geometry_msgs.Pose(geometry_msgs.Vector3(550/1000, -575/1000, -250/1000 + 0.4), geometry_msgs.Quaternion(0.0800099, 0.7360909, -0.672083, 0.0085462)),
    geometry_msgs.Pose(geometry_msgs.Vector3(550/1000, -575/1000, -250/1000 + 0.4), geometry_msgs.Quaternion(0.769365, 0.1440758, -0.6051185, -0.1454344))
#        tp_to_ros(480,-400,-366,-90,-90,0),
        ]

duration_list = [4.0,8.0,12.0]

if __name__ == "__main__":

    rospy.init_node('test_move')
    
    arm = ArmClient()
    #gripper = RobotiqGripper("192.168.50.82")

    ##
    #gripper.move_and_wait_for_pos(100, 255, 255)
    arm.move(pose_list, duration_list)
    #gripper.move_and_wait_for_pos(255, 255, 255)
