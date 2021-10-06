#!/usr/bin/env python3

import sys
import numpy as np

from time import sleep

import rospy
import geometry_msgs.msg as geometry_msgs
from client_arm import ArmClient
from client_gripper import RobotiqGripper
from tf.transformations import quaternion_from_euler
import numpy as np



from mrsdl_database import *


# ============================================================================
#                                           CREATE PROTOCOLS FOR DEMO
# ============================================================================
#demo_protocol_1 = [ "go_to_mixing_station", "gripper_to_front", "open_gripper", "get_clean_beaker", "close_gripper", "beaker_to_fluids", "add_next_reagent", "add_next_reagent", "add_next_reagent","go_to_electrochem_station" ];
#test_absolute = [ "gripper_to_home", "gripper_to_front", "gripper_to_home", "gripper_to_front" ];
#test_relative = [ "gripper_to_front", "get_clean_beaker", "get_clean_beaker", "get_clean_beaker" ];

#demo_protocol = demo_protocol_1;
#demo_protocol = test_absolute;
#demo_protocol = test_relative;

demo_protocol = test_mx


# ============================================================================
#                                           PROTOCOL EXECUTION ENGINE
# ============================================================================
rospy.init_node('clyde_crashcup')
# FIX # base    = BaseClient();
arm     = ArmClient();
# FIX # gripper = GripperClient();

xyz_now = geometry_msgs.Vector3(0,0,0)                            # initialize absolution position accumulator
def clyde_crashcup(protocol):
    # playback an experimental protocol
    for t in protocol:
        tsk = mini_tasks[t];

        if (type(tsk) is MiRTask):
            task_type = 'MiRTask';
            # FIX # base.move(tsk.position) 

        if (type(tsk) is ArmTask):
            task_type = 'ArmTask';
            if ('joint' in tsk.action_server):
                print('Joint based trajectory tasks not implemented yet\n')
                # FIX # arm.move(tsk.position_list,tsk.duration_list)
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
                print(absolute_position_list)
                arm.move(tsk.position_list,tsk.duration_list);

                print(xyz_now)
       
        if (type(tsk) is GripperTask):
            task_type = 'GripperTask';
            # FIX # gripper.move(tsk.gripper_state)

        if (type(tsk) is QueryTask):
            task_type = 'QueryTask';
            # figure out what to do here for two type of query
            # 1. query to reagent list for next reagent, or done
            # 2. query to AI guiding the experiment for next experiment, or DONE

        print(t, task_type)


# ============================================================================
#                                           TEST DRIVE
# ============================================================================

clyde_crashcup(['gripper_to_minus_y','gripper_to_minus_x'])




