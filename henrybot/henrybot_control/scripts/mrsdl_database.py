#!/usr/bin/env python3

import numpy as np
from dataclasses import dataclass
import geometry_msgs.msg as geometry_msgs
from mrsdl_joint_models import *

# ============================================================================
#                                           CREATE DATABASE OF MINI-TASKS
# ============================================================================
@dataclass
class ArmTask:
    action_server: str
    position_list: []      # either a 6DOF position or [ [x,y,z], [q0,q1,q2,q3] ]
    duration_list: []      # target time in seconds at each position in list
    absolute_pos: bool
    absolute_orient: bool = True

@dataclass
class MiRTask:
    position: []    # Pose2D:   x, y, theta

@dataclass
class QueryTask:
    query_type: str

@dataclass
class GripperTask:
    gripper_state: int     # 0 = closed, 255 = open


# initialize database
mini_tasks = dict()

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> GRIPPER
mini_tasks["open_gripper"]  = GripperTask(0)
mini_tasks["close_gripper"] = GripperTask(255)
mini_tasks["grab_block"]    = GripperTask(186)
mini_tasks["grab_beaker"]   = GripperTask(150)
mini_tasks["open_mic"]      = GripperTask(150)

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> BASE
mini_tasks["go_to_charger_stage"]   = MiRTask([ 5.2, 2.2, -90])
mini_tasks["go_to_charger"]         = MiRTask([ 5.2, 1.72, -89.1])
mini_tasks["go_to_bottom_up"]       = MiRTask([ 5.2, 3.0,  0])
mini_tasks["go_to_center_up"]       = MiRTask([ 6.6, 3.0, 0])
mini_tasks["go_to_top_up"]          = MiRTask([ 8.5, 3.0, 0])
mini_tasks["go_to_bottom_down"]     = MiRTask([ 5.2, 3.0, 180])
mini_tasks["go_to_center_down"]     = MiRTask([ 7.0, 3.0, 180])
mini_tasks["go_to_top_down"]        = MiRTask([ 8.5, 3.0, 180])
# mini_tasks["go_to_mixing_station"]       = MiRTask([ 15.3, 24.5, 90])
# mini_tasks["go_to_mixing_nearby"]        = MiRTask([ 15.3, 22.5,  0])
# mini_tasks["go_to_mixing_runway"]        = MiRTask([ 15.3, 22.5, 90])
# mini_tasks["go_to_mixing_offramp"]       = MiRTask([ 15.3, 26.0, 90])
# mini_tasks["go_around_a180"]             = MiRTask([ 15.3, 26.0, 180])
# mini_tasks["go_around_b180"]             = MiRTask([ 12.3, 26.0, 180])
# mini_tasks["go_around_b-90"]             = MiRTask([ 12.3, 26.0, -90])
# mini_tasks["go_around_b-90"]             = MiRTask([ 12.3, 22.5, -90])
# mini_tasks["go_around_c000"]             = MiRTask([ 12.3, 22.5,   0])
# mini_tasks["go_around_d000"]             = MiRTask([ 15.3, 22.5,   0])
# mini_tasks["go_to_electrochem_station"]  = MiRTask([16,11,0])
# mini_tasks["go_to_electrochem_nearby"]   = MiRTask([16,11,0])

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ARM ABSOLUTE
task_name = "gripper_to_home"          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],True)
mini_tasks[task_name].action_server = "joint_based_trajectory"
mini_tasks[task_name].duration_list = [8.0]
mini_tasks[task_name].position_list = [
    xyz_to_joints(VERT_GRIP, ur5e_params, [0,0.5,0.3], dock_angle=DOCK_BACK)[0]
]

task_name = "gripper_to_front"          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],True)
mini_tasks[task_name].action_server = "joint_based_trajectory"
mini_tasks[task_name].duration_list = [5.0]
mini_tasks[task_name].position_list = [
    xyz_to_joints(VERT_GRIP, ur5e_params, [0,0.5,0.0],  dock_angle=DOCK_BACK)[0]
]

task_name = "gripper_to_table_center"          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],True)
mini_tasks[task_name].action_server = "joint_based_trajectory"
mini_tasks[task_name].duration_list = [5.0]
mini_tasks[task_name].position_list = [
    xyz_to_joints(VERT_GRIP, ur5e_params, [0,0.350,-0.190],  dock_angle=DOCK_BACK)[0]
]


# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ARM RELATIVE
## task_name = "get_clean_beaker"          # ---------------------------------
## mini_tasks[task_name] = ArmTask("",[],[],False)
## mini_tasks[task_name].action_server = "pose_based_cartesian_trajectory"
## mini_tasks[task_name].position_list = [
##     geometry_msgs.Pose(
##         geometry_msgs.Vector3(0, 0,   0.1), geometry_msgs.Quaternion(0, 0, 0, 1)
##     ),
##     geometry_msgs.Pose(
##         geometry_msgs.Vector3(0, 0.4, 0  ), geometry_msgs.Quaternion(0, 0, 0, 1)
##     )
## ]
## mini_tasks[task_name].duration_list = [2.0, 3.0]
## 
## task_name = "beaker_to_fluids"          # ---------------------------------
## mini_tasks[task_name] = ArmTask("",[],[],False)
## mini_tasks[task_name].action_server = "pose_based_cartesian_trajectory"
## mini_tasks[task_name].position_list = [
##     geometry_msgs.Pose(
##         geometry_msgs.Vector3(0, 0,   0.1), geometry_msgs.Quaternion(0, 0, 0, 1)
##     )
## ]
## mini_tasks[task_name].duration_list = [2.0]
## 
## task_name = "add_next_reagent"          # ---------------------------------
## mini_tasks[task_name] = ArmTask("",[],[],False)
## mini_tasks[task_name].action_server = "pose_based_cartesian_trajectory"
## mini_tasks[task_name].position_list = [
##     geometry_msgs.Pose(
##         geometry_msgs.Vector3(0,  0.2, 0), geometry_msgs.Quaternion(0, 0, 0, 1)
##     ),
##     geometry_msgs.Pose(
##         geometry_msgs.Vector3(0,  0,   0.1), geometry_msgs.Quaternion(0, 0, 0, 1)
##     ),
##     geometry_msgs.Pose(
##         geometry_msgs.Vector3(0,  0,  -0.1), geometry_msgs.Quaternion(0, 0, 0, 1)
##     ),
##     geometry_msgs.Pose(
##         geometry_msgs.Vector3(0, -0.2, 0), geometry_msgs.Quaternion(0, 0, 0, 1)
##     )
## ]
## mini_tasks[task_name].duration_list = [2.0, 3.0, 3.0, 2.0]

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> NOT READY
## task_name = "put_beaker_on_table"       # ---------------------------------
## mini_tasks[task_name] = ArmTask("",[],[],False)
## 
## task_name = "put_wp_in_beaker"          # ---------------------------------
## mini_tasks[task_name] = ArmTask("",[],[],False)
## 
## task_name = "put_aux_in_beaker"         # ---------------------------------
## mini_tasks[task_name] = ArmTask("",[],[],False)
## 
## task_name = "make_measurements"         # ---------------------------------
## mini_tasks[task_name] = ArmTask("",[],[],False)
## 
## task_name = "rinse_aux"                 # ---------------------------------
## mini_tasks[task_name] = ArmTask("",[],[],False)
## 
## task_name = "dispose_used_wp"           # ---------------------------------
## mini_tasks[task_name] = ArmTask("",[],[],False)
## 
## task_name = "dispose_used_beaker"       # ---------------------------------
## mini_tasks[task_name] = ArmTask("",[],[],False)
## 
## task_name = "grab_mic"                  # ---------------------------------
## mini_tasks[task_name] = ArmTask("",[],[],False)
## 
## task_name = "lift_mic"                  # ---------------------------------
## mini_tasks[task_name] = ArmTask("",[],[],False)
## 
## task_name = "drop_mic"                  # ---------------------------------
## mini_tasks[task_name] = ArmTask("",[],[],False)

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> QUERY
task_name = "more_reagents_query"       # ---------------------------------
mini_tasks[task_name] = QueryTask("")

task_name = "more_tests_query"          # ---------------------------------
mini_tasks[task_name] = QueryTask("")



# ============================================================================
#                                           CARDINAL POSITIONS
# ============================================================================

## # https://www.andre-gaschler.com/rotationconverter/
## #     enter Rx,Ry,Rz in radians into "axis with angle magnitude" on left side
## #     pull quaternion values [x,y,z,w] from right side of page
## #
## #          X      Y     Z       Rx     Ry     Rz        raw_Qx      raw_Qy      raw_Qz      raw_Qw                 Qx      Qy      Qz      Qw
## # +y       0    365  -333     4.72   0.02  -0.02     [ 0.704368,   0.0029846, -0.0029846, -0.7098224 ]          0.707   0.000   0.000  -0.707
## # -x    -365      0  -333     2.40   2.42  -2.43     [ 0.4969667,  0.5011081, -0.5031788, -0.4987242 ]          0.500   0.500  -0.500  -0.500
## # -y       0   -365  -333     0.01  -2.22   2.23     [ 0.003178,  -0.7055102,  0.7086882, -0.0025282 ]          0.000  -0.707   0.707   0.000
## # +x     365      0  -333     2.43  -2.41   2.43     [ 0.5001256, -0.4960093,  0.5001256, -0.5037098 ]          0.500  -0.500   0.500  -0.500
## nom_plus_y  = geometry_msgs.Pose(geometry_msgs.Vector3(    0,  365, -100 ), geometry_msgs.Quaternion(-0.707,  0.000,  0.000,  0.707 ))
## nom_minus_x = geometry_msgs.Pose(geometry_msgs.Vector3( -365,    0, -100 ), geometry_msgs.Quaternion(-0.500, -0.500,  0.500,  0.500 ))
## nom_minus_y = geometry_msgs.Pose(geometry_msgs.Vector3(    0, -365, -100 ), geometry_msgs.Quaternion( 0.000,  0.707, -0.707,  0.000 ))
## nom_plus_x  = geometry_msgs.Pose(geometry_msgs.Vector3(  365,    0, -100 ), geometry_msgs.Quaternion(-0.500,  0.500, -0.500,  0.500 ))
## 
## 
## task_name = "gripper_to_plus_y"          # ---------------------------------
## mini_tasks[task_name] = ArmTask("",[],[],True)
## mini_tasks[task_name].action_server = "pose_based_cartesian_trajectory"
## mini_tasks[task_name].position_list = [ nom_plus_y ]
## mini_tasks[task_name].duration_list = [10.0]
## 
## task_name = "gripper_to_minus_x"          # ---------------------------------
## mini_tasks[task_name] = ArmTask("",[],[],True)
## mini_tasks[task_name].action_server = "pose_based_cartesian_trajectory"
## mini_tasks[task_name].position_list = [ nom_minus_x ]
## mini_tasks[task_name].duration_list = [10.0]
## 
## task_name = "gripper_to_minus_y"          # ---------------------------------
## mini_tasks[task_name] = ArmTask("",[],[],True)
## mini_tasks[task_name].action_server = "pose_based_cartesian_trajectory"
## mini_tasks[task_name].position_list = [ nom_minus_y ]
## mini_tasks[task_name].duration_list = [10.0]
## 
## task_name = "gripper_to_plus_x"          # ---------------------------------
## mini_tasks[task_name] = ArmTask("",[],[],True)
## mini_tasks[task_name].action_server = "pose_based_cartesian_trajectory"
## mini_tasks[task_name].position_list = [ nom_plus_x ]
## mini_tasks[task_name].duration_list = [10.0]
## 
## task_name = "gripper_p1"          # ---------------------------------
## mini_tasks[task_name] = ArmTask("",[],[],True)
## mini_tasks[task_name].action_server = "pose_based_cartesian_trajectory"
## mini_tasks[task_name].position_list = [
##     geometry_msgs.Pose(geometry_msgs.Vector3(-.100,.240,.665),geometry_msgs.Quaternion(-.707,0,0,.707 )), 
## ]
## mini_tasks[task_name].duration_list = [8.0]
## 
## task_name = "gripper_fwd"          # ---------------------------------
## mini_tasks[task_name] = ArmTask("",[],[],False)
## mini_tasks[task_name].action_server = "pose_based_cartesian_trajectory"
## mini_tasks[task_name].position_list = [
##     geometry_msgs.Pose(
##         geometry_msgs.Vector3(0, 0,   0.001), geometry_msgs.Quaternion(-.707,0,0,.707)
##     ),
## ]
## mini_tasks[task_name].duration_list = [8.0]
## 
## 
## #               base   shoulder   elbow   wrist_1  wrist_2  wrist_3
## #             -------- -------- -------- -------- -------- --------
## # vicinity 1     220      -27     -137      -14      -39        0
## def deg2rad(lst):
##     radlst = []
##     for j in lst:
##         radlst.append(j*np.pi/180.)
##     return radlst
## 
## task_name = "joint_v1"          # ---------------------------------
## mini_tasks[task_name] = ArmTask("",[],[],True)
## mini_tasks[task_name].action_server = "joint_based_trajectory"
## mini_tasks[task_name].position_list = [
##     deg2rad([220,-27,-137,-14,-39,0]),
##     deg2rad([225,-27,-137,-14,-39,0]),
##     deg2rad([220,-27,-137,-14,-39,0]),
##     deg2rad([220,-22,-137,-14,-39,0]),
##     deg2rad([220,-27,-137,-14,-39,0]),
##     deg2rad([220,-27,-132,-14,-39,0]),
##     deg2rad([220,-27,-137,-14,-39,0]),
##     deg2rad([220,-27,-137,-19,-39,0]),
##     deg2rad([220,-27,-137,-14,-39,0]),
##     deg2rad([220,-27,-137,-14,-34,0]),
##     deg2rad([220,-27,-137,-14,-39,0]),
##     deg2rad([220,-27,-137,-14,-34,5]),
## ]
## mini_tasks[task_name].duration_list = [8.0,12.0,16.0,20.0,24.0,28.0,32.0,36.0,40.0,44.0,48.0,52.0,56.0]
## 
## task_name = "joint_v1_sp"          # ---------------------------------
## mini_tasks[task_name] = ArmTask("",[],[],True)
## mini_tasks[task_name].action_server = "joint_based_trajectory"
## mini_tasks[task_name].position_list = [
##     deg2rad([220,-27,-137,-14,-39,0]),
##     deg2rad([210,-27,-137,-14,-39,0]),
## ]
## mini_tasks[task_name].duration_list = [6.0,4.0]
## 
## task_name = "joint_v1_sl"          # ---------------------------------
## mini_tasks[task_name] = ArmTask("",[],[],True)
## mini_tasks[task_name].action_server = "joint_based_trajectory"
## mini_tasks[task_name].position_list = [
##     deg2rad([200,-17,-137,-14,-39,0]),
##     deg2rad([200,-27,-137,-14,-39,0]),
## ]
## mini_tasks[task_name].duration_list = [6.0,4.0]
## 
## task_name = "joint_v1_el"          # ---------------------------------
## mini_tasks[task_name] = ArmTask("",[],[],True)
## mini_tasks[task_name].action_server = "joint_based_trajectory"
## mini_tasks[task_name].position_list = [
##     deg2rad([190,-27,-122,-14,-39,0]),
##     deg2rad([190,-27,-132,-14,-39,0]),
## ]
## mini_tasks[task_name].duration_list = [2.0,4.0]
## 
## task_name = "joint_v1_w1"          # ---------------------------------
## mini_tasks[task_name] = ArmTask("",[],[],True)
## mini_tasks[task_name].action_server = "joint_based_trajectory"
## mini_tasks[task_name].position_list = [
##     deg2rad([180,-27,-137,-4,-39,0]),
##     deg2rad([180,-27,-137,-14,-39,0]),
## ]
## mini_tasks[task_name].duration_list = [2.0,4.0]
## 
## task_name = "joint_v1_w2"          # ---------------------------------
## mini_tasks[task_name] = ArmTask("",[],[],True)
## mini_tasks[task_name].action_server = "joint_based_trajectory"
## mini_tasks[task_name].position_list = [
##     deg2rad([170,-27,-137,-14,-29,0]),
##     deg2rad([170,-27,-137,-14,-39,0]),
## ]
## mini_tasks[task_name].duration_list = [2.0,4.0]
## 
## task_name = "joint_v1_w3"          # ---------------------------------
## mini_tasks[task_name] = ArmTask("",[],[],True)
## mini_tasks[task_name].action_server = "joint_based_trajectory"
## mini_tasks[task_name].position_list = [
##     deg2rad([160,-27,-137,-14,-39,10]),
##     deg2rad([160,-27,-137,-14,-39,0]),
## ]
## mini_tasks[task_name].duration_list = [2.0,4.0]
## 
## task_name = "joint_v1_home"          # ---------------------------------
## mini_tasks[task_name] = ArmTask("",[],[],True)
## mini_tasks[task_name].action_server = "joint_based_trajectory"
## mini_tasks[task_name].position_list = [
##     deg2rad([220,-27,-137,-14,-34,0]),
## ]
## mini_tasks[task_name].duration_list = [8.0]


# ============================================================================
#                                           EXAMPLE PROTOCOLS
# ============================================================================
demo_protocol_1 = [ "go_to_mixing_station", "gripper_to_front", "open_gripper", "get_clean_beaker", "close_gripper", "beaker_to_fluids", "add_next_reagent", "add_next_reagent", "add_next_reagent","go_to_electrochem_station" ]
test_absolute = [ "gripper_to_home", "gripper_to_front", "gripper_to_home", "gripper_to_front" ]
test_relative = [ "gripper_to_front", "get_clean_beaker", "get_clean_beaker", "beaker_to_fluids" ]
circle = ["go_to_bottom_down","go_to_bottom_up","go_to_center_up","go_to_top_up","go_to_top_down","go_to_center_down","go_to_bottom_down"]

test_py = ["gripper_to_plus_y"]
test_mx = ["gripper_to_minus_x"]
test_my = ["gripper_to_minus_y"]
test_px = ["gripper_to_plus_x"]

#test_this = ["gripper_p1","gripper_fwd"]
#test_this = ["joint_v1"]
test_this = [ "joint_v1_sp", "joint_v1_sl", "joint_v1_el", "joint_v1_w1", "joint_v1_w2", "joint_v1_w3" ]
#test_this = [ "joint_v1_w3" ]





# ========================== tic-tac-toe calibration
da = DOCK_LEFT
origin_TP = [ -0.387-0.140, -0.164 ]
twotwo_TP = [ -0.664-0.140, +0.184 ]
origin = [ origin_TP[1]-0.010, -origin_TP[0]+0.020]
twotwo = [ twotwo_TP[1]-0.010, -twotwo_TP[0]+0.020]
z_block = -0.315
# da = DOCK_BACK
# origin = [-0.15, 0.30]
# twotwo = [ 0.05, 0.50]
# z_block = 0
print(f'ORIGIN: {origin[0]:5.3f} {origin[1]:5.3f}    TWOTWO: {twotwo[0]:5.3f} {twotwo[1]:5.3f}')

def ttt(i,j):
    # x =  Y_TD
    # y = -X_TD
    dx = (twotwo[0]-origin[0])/2.
    dy = (twotwo[1]-origin[1])/2.
    x =  origin[0]+(i+0.5)*dx
    y =  origin[1]+(j+0.5)*dy
    print(f'{i} {j} {dx:5.3f} {dy:5.3f} {x:5.3f} {y:5.3f} {z_block:5.3f}')
    return [x,y,z_block]


ttt_duration = 2.0
task_name = "ttt_origin"          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],True)
mini_tasks[task_name].action_server = "joint_based_trajectory"
mini_tasks[task_name].position_list = [
    xyz_to_joints(VERT_GRIP, ur5e_params, [origin[0],origin[1],z_block],  dock_angle=da)[0]
]
mini_tasks[task_name].duration_list = [ttt_duration]
task_name = "ttt_twotwo"          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],True)
mini_tasks[task_name].action_server = "joint_based_trajectory"
mini_tasks[task_name].position_list = [
    xyz_to_joints(VERT_GRIP, ur5e_params, [twotwo[0],twotwo[1],z_block],  dock_angle=da)[0]
]
mini_tasks[task_name].duration_list = [ttt_duration]
task_name = "ttt_home"          # ---------------------------------
ctr = ttt(0.5,0.5)
ctr[2] = 0.000
mini_tasks[task_name] = ArmTask("",[],[],True)
mini_tasks[task_name].action_server = "joint_based_trajectory"
mini_tasks[task_name].position_list = [
    xyz_to_joints(VERT_GRIP, ur5e_params, ctr,  dock_angle=da)[0]
]
mini_tasks[task_name].duration_list = [ttt_duration]
task_name = "ttt_00"          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],True)
mini_tasks[task_name].action_server = "joint_based_trajectory"
mini_tasks[task_name].position_list = [
    xyz_to_joints(VERT_GRIP, ur5e_params, ttt(0,0),  dock_angle=da)[0]
]
mini_tasks[task_name].duration_list = [ttt_duration]
task_name = "ttt_01"          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],True)
mini_tasks[task_name].action_server = "joint_based_trajectory"
mini_tasks[task_name].position_list = [
    xyz_to_joints(VERT_GRIP, ur5e_params, ttt(0,1),  dock_angle=da)[0]
]
mini_tasks[task_name].duration_list = [ttt_duration]
task_name = "ttt_10"          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],True)
mini_tasks[task_name].action_server = "joint_based_trajectory"
mini_tasks[task_name].position_list = [
    xyz_to_joints(VERT_GRIP, ur5e_params, ttt(1,0),  dock_angle=da)[0]
]
mini_tasks[task_name].duration_list = [ttt_duration]
task_name = "ttt_11"          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],True)
mini_tasks[task_name].action_server = "joint_based_trajectory"
mini_tasks[task_name].position_list = [
    xyz_to_joints(VERT_GRIP, ur5e_params, ttt(1,1),  dock_angle=da)[0]
]
mini_tasks[task_name].duration_list = [ttt_duration]
# task_name = "ttt_02"          # ---------------------------------
# mini_tasks[task_name] = ArmTask("",[],[],True)
# mini_tasks[task_name].action_server = "joint_based_trajectory"
# mini_tasks[task_name].position_list = [
#     xyz_to_joints(VERT_GRIP, ur5e_params, ttt(0,2),  dock_angle=da)[0]
# ]
# mini_tasks[task_name].duration_list = [ttt_duration]
# task_name = "ttt_12"          # ---------------------------------
# mini_tasks[task_name] = ArmTask("",[],[],True)
# mini_tasks[task_name].action_server = "joint_based_trajectory"
# mini_tasks[task_name].position_list = [
#     xyz_to_joints(VERT_GRIP, ur5e_params, ttt(1,2),  dock_angle=da)[0]
# ]
# mini_tasks[task_name].duration_list = [ttt_duration]
# task_name = "ttt_20"          # ---------------------------------
# mini_tasks[task_name] = ArmTask("",[],[],True)
# mini_tasks[task_name].action_server = "joint_based_trajectory"
# mini_tasks[task_name].position_list = [
#     xyz_to_joints(VERT_GRIP, ur5e_params, ttt(2,0),  dock_angle=da)[0]
# ]
# mini_tasks[task_name].duration_list = [ttt_duration]
# task_name = "ttt_21"          # ---------------------------------
# mini_tasks[task_name] = ArmTask("",[],[],True)
# mini_tasks[task_name].action_server = "joint_based_trajectory"
# mini_tasks[task_name].position_list = [
#     xyz_to_joints(VERT_GRIP, ur5e_params, ttt(2,1),  dock_angle=da)[0]
# ]
# mini_tasks[task_name].duration_list = [ttt_duration]
# task_name = "ttt_22"          # ---------------------------------
# mini_tasks[task_name] = ArmTask("",[],[],True)
# mini_tasks[task_name].action_server = "joint_based_trajectory"
# mini_tasks[task_name].position_list = [
#     xyz_to_joints(VERT_GRIP, ur5e_params, ttt(2,2),  dock_angle=da)[0]
# ]
# mini_tasks[task_name].duration_list = [ttt_duration]
def ttt_math(ttt_pos,delta_cm):
    result = ttt(ttt_pos[0],ttt_pos[1])
    for i in range(len(result)):
        result[i] += delta_cm[i]
    return result
task_name = "above_beaker"          # --------------------------------- FOR DROPPING BLOCKS
mini_tasks[task_name] = ArmTask("",[],[],True)
mini_tasks[task_name].action_server = "joint_based_trajectory"
mini_tasks[task_name].duration_list = [2.0]
mini_tasks[task_name].position_list = [
    xyz_to_joints(VERT_GRIP, ur5e_params, ttt_math([-1,0.5],[0,0,0.200]), dock_angle=DOCK_LEFT)[0]
]
task_name = "over_beaker"          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],True)
mini_tasks[task_name].action_server = "joint_based_trajectory"
mini_tasks[task_name].duration_list = [2.0]
mini_tasks[task_name].position_list = [
    xyz_to_joints(HORZ_XNEG_GRIP, ur5e_params, ttt_math([-1,0.5],[0.120,0,0.200]), dock_angle=DOCK_LEFT)[0]
]
task_name = "near_beaker"          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],True)
mini_tasks[task_name].action_server = "joint_based_trajectory"
mini_tasks[task_name].duration_list = [2.0]
mini_tasks[task_name].position_list = [
    xyz_to_joints(HORZ_XNEG_GRIP, ur5e_params, ttt_math([-1,0.5],[0.120,0,0.050]), dock_angle=DOCK_LEFT)[0]
]
task_name = "at_beaker"          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],True)
mini_tasks[task_name].action_server = "joint_based_trajectory"
mini_tasks[task_name].duration_list = [1.0]
mini_tasks[task_name].position_list = [
    xyz_to_joints(HORZ_XNEG_GRIP, ur5e_params, ttt_math([-1,0.5],[-0.080,0,0.050]), dock_angle=DOCK_LEFT)[0]
]
task_name = "over_ttt"          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],True)
mini_tasks[task_name].action_server = "joint_based_trajectory"
mini_tasks[task_name].duration_list = [4.0]
mini_tasks[task_name].position_list = [
    xyz_to_joints(HORZ_XNEG_GRIP, ur5e_params, ttt_math([-1,0.5],[0.100,0,0.200]), dock_angle=DOCK_RIGHT)[0]
]
task_name = "near_ttt"          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],True)
mini_tasks[task_name].action_server = "joint_based_trajectory"
mini_tasks[task_name].duration_list = [2.0]
mini_tasks[task_name].position_list = [
    xyz_to_joints(HORZ_XNEG_GRIP, ur5e_params, ttt_math([-1,0.5],[0.100,0,0.050]), dock_angle=DOCK_RIGHT)[0]
]
task_name = "at_ttt"          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],True)
mini_tasks[task_name].action_server = "joint_based_trajectory"
mini_tasks[task_name].duration_list = [1.0]
mini_tasks[task_name].position_list = [
    xyz_to_joints(HORZ_XNEG_GRIP, ur5e_params, ttt_math([-1,0.5],[-0.100,0,0.050]), dock_angle=DOCK_RIGHT)[0]
]
task_name = "over_tray"          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],True)
mini_tasks[task_name].action_server = "joint_based_trajectory"
mini_tasks[task_name].duration_list = [4.0]
mini_tasks[task_name].position_list = [
    xyz_to_joints(HORZ_XNEG_GRIP, ur5e_params, np.array([0,0.300,0.000])+np.array([0.100,0.100,0.200]), dock_angle=DOCK_BACK)[0]
]
task_name = "near_tray"          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],True)
mini_tasks[task_name].action_server = "joint_based_trajectory"
mini_tasks[task_name].duration_list = [2.0]
mini_tasks[task_name].position_list = [
    xyz_to_joints(HORZ_XNEG_GRIP, ur5e_params, np.array([0,0.300,0.000])+np.array([0.050,0.100,-0.130]), dock_angle=DOCK_BACK)[0]
]
task_name = "at_tray"          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],True)
mini_tasks[task_name].action_server = "joint_based_trajectory"
mini_tasks[task_name].duration_list = [1.0]
mini_tasks[task_name].position_list = [
    xyz_to_joints(HORZ_XNEG_GRIP, ur5e_params, np.array([0,0.300,0.000])+np.array([-0.100,0.100,-0.150]), dock_angle=DOCK_BACK)[0]
]
task_name = "at_mic"          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],True)
mini_tasks[task_name].action_server = "joint_based_trajectory"
mini_tasks[task_name].duration_list = [1.0]
mini_tasks[task_name].position_list = [
    xyz_to_joints(VERT_GRIP, ur5e_params, [0,0.520,-0.180], orientation=90, dock_angle=DOCK_BACK)[0]
]
task_name = "lift_mic"          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],True)
mini_tasks[task_name].action_server = "joint_based_trajectory"
mini_tasks[task_name].duration_list = [1.0]
mini_tasks[task_name].position_list = [
    xyz_to_joints(VERT_GRIP, ur5e_params, [0,0.520,-0.100], orientation=90, dock_angle=DOCK_BACK)[0]
]
task_name = "extend_mic"          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],True)
mini_tasks[task_name].action_server = "joint_based_trajectory"
mini_tasks[task_name].duration_list = [1.0]
mini_tasks[task_name].position_list = [
    xyz_to_joints(VERT_GRIP, ur5e_params, [0,0.900,-0.100], orientation=90, dock_angle=DOCK_BACK)[0]
]

# ttt_tour = [
#     'ttt_home', 'ttt_00',
#     'ttt_home', 'ttt_01',
#     'ttt_home', 'ttt_02',
#     'ttt_home', 'ttt_12',
#     'ttt_home', 'ttt_11',
#     'ttt_home', 'ttt_10',
#     'ttt_home', 'ttt_20',
#     'ttt_home', 'ttt_21',
#     'ttt_home', 'ttt_22',
# ]
ttt_tour = [
    'ttt_home', 'ttt_00',
    'ttt_home', 'ttt_01',
    'ttt_home', 'ttt_11',
    'ttt_home', 'ttt_10',
]
