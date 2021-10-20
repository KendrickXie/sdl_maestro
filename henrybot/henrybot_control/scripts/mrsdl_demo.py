#!/usr/bin/env python3

import sys
import numpy as np

from time import sleep

import rospy
import geometry_msgs.msg as geometry_msgs
from client_base import BaseClient
from client_arm import ArmClient
from client_gripper import RobotiqGripper
from tf.transformations import quaternion_from_euler
import numpy as np



from mrsdl_database import *

def print_xyz(xyz):
    print(f'{xyz.x:6.3f} {xyz.y:6.3f} {xyz.z:6.3f}')

def print_pose(p):
    x = p.position.x
    y = p.position.y
    z = p.position.z
    qx = p.orientation.x
    qy = p.orientation.y
    qz = p.orientation.z
    qw = p.orientation.w
    print(f'{x:6.3f} {y:6.3f} {z:6.3f}    {qx:6.3f} {qy:6.3f} {qz:6.3f} {qw:6.3f}')

# ============================================================================
#                                           CREATE PROTOCOLS FOR DEMO
# ============================================================================
#demo_protocol_1 = [ "go_to_mixing_station", "gripper_to_front", "open_gripper", "get_clean_beaker", "close_gripper", "beaker_to_fluids", "add_next_reagent", "add_next_reagent", "add_next_reagent","go_to_electrochem_station" ]
#test_absolute = [ "gripper_to_home", "gripper_to_front", "gripper_to_home", "gripper_to_front" ]
#test_relative = [ "gripper_to_front", "get_clean_beaker", "get_clean_beaker", "get_clean_beaker" ]

#demo_protocol = demo_protocol_1
#demo_protocol = test_absolute
#demo_protocol = test_relative

demo_protocol = test_mx


# ============================================================================
#                                           PROTOCOL EXECUTION ENGINE
# ============================================================================
rospy.init_node('clyde_crashcup')
base    = BaseClient()
arm     = ArmClient()
gripper = RobotiqGripper("192.168.50.82")
sleep(2)
gripper.move(255,255,255)

xyz_now = geometry_msgs.Vector3(0,0,0)                 # initialize absolution position accumulator
def clyde_crashcup(protocol):
    # playback an experimental protocol
    for t in protocol:
        tsk = mini_tasks[t]

        if (type(tsk) is MiRTask):
            task_type = 'MiRTask'
            bx = tsk.position[0]
            by = tsk.position[1]
            bth = tsk.position[2]
            base.move(bx,by,bth)
            sleep(10.0)

        if (type(tsk) is ArmTask):
            task_type = 'ArmTask'
            if ('joint' in tsk.action_server):
                #print('Joint based trajectory tasks not implemented yet\n')
                for p in tsk.position_list:
                    print(f'{p[0]:8.2f} {p[1]:8.2f} {p[2]:8.2f} {p[3]:8.2f} {p[4]:8.2f} {p[5]:8.2f} ')
                arm.move(tsk.position_list,tsk.duration_list,"forward_joint_traj_controller")
            elif ('cartesian' in tsk.action_server):
                if tsk.absolute_pos:
                    absolute_position_list = tsk.position_list
                else:
                    absolute_position_list = []
                    for rel_pos in tsk.position_list:
                        abs_pos = geometry_msgs.Vector3(rel_pos.position.x + xyz_now.x, rel_pos.position.y + xyz_now.y, rel_pos.position.z + xyz_now.z)
                        absolute_position_list.append(geometry_msgs.Pose(abs_pos,rel_pos.orientation))
                # update to position after trajectory complete
                xyz_now = absolute_position_list[-1].position

                # execute absolute cartesian trajectory
                #print(absolute_position_list)
                for p in absolute_position_list:
                    print_pose(p)
                arm.move(tsk.position_list,tsk.duration_list)

                #print(xyz_now)
                print_xyz(xyz_now)
       
        if (type(tsk) is GripperTask):
            task_type = 'GripperTask'
            gripper.move(tsk.gripper_state,255,255)

        if (type(tsk) is QueryTask):
            task_type = 'QueryTask'
            # figure out what to do here for two type of query
            # 1. query to reagent list for next reagent, or done
            # 2. query to AI guiding the experiment for next experiment, or DONE

        print(t, task_type)

def cli_clyde():
    # args
    # help
    # list <fragment>
    # interactive
    # exit
    # info <mini-task>
    
    
    parser = ArgumentParser()
    parser.add_argument("-m", default=16.0, type=str)
    parser.add_argument("-x", default=23.0, type=float)
    parser.add_argument("-y", default=23.0, type=float)
    parser.add_argument("-z", default=23.0, type=float)
    parser.add_argument("-d", default="BACK", type=str)
    args = parser.parse_args()

    # rospy.init_node("arm_python")

    # model = VERT_GRIP
    # if "VERT" in args.m:
    #     model = VERT_GRIP
    # pos_X = args.x
    # pos_Y = args.y
    # pos_Z = args.z
    # dock_dir = DOCK_BACK
    # if "BACK" in args.d:
    #     dock_dir = DOCK_BACK
    # elif "FRONT" in args.d:
    #     dock_dir = DOCK_FRONT
    # elif "LEFT" in args.d:
    #     dock_dir = DOCK_LEFT
    # elif "RIGHT" in args.d:
    #     dock_dif = DOCK_RIGHT

    # joints,err = xyz_to_joints(model, ur5e_params, [pos_X,pos_Y,pos_Z], dock_angle=dock_dir)

    # arm = ArmClient()
    # arm.move([joints],[8.0],"forward_joint_trajectory_controller")

# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass





# ============================================================================
#                                           TEST DRIVE
# ============================================================================

#clyde_crashcup(['gripper_to_minus_y','gripper_to_minus_x'])
#clyde_crashcup(test_this)

#clyde_crashcup(['gripper_to_home','open_gripper','gripper_to_table_center','grab_block','gripper_to_front','open_gripper'])

#clyde_crashcup([ 'go_to_mixing_offramp', 'go_to_mixing_nearby', 'go_to_mixing_runway', 'go_to_mixing_station' ])

#clyde_crashcup(['close_gripper', 'ttt_home', 'ttt_00', 'ttt_11', 'ttt_01', 'ttt_10'])
#clyde_crashcup(['close_gripper', 'ttt_home', 'ttt_origin'])
#clyde_crashcup(ttt_tour)
clyde_crashcup(['ttt_home', 'open_gripper', 'ttt_00', 'grab_block', 'ttt_home', 'ttt_11', 'open_gripper', 'ttt_home', 'close_gripper'])


