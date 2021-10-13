#!/usr/bin/env python3

import numpy as np
from dataclasses import dataclass
import geometry_msgs.msg as geometry_msgs

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
    target_position: []    # Pose2D:   x, y, theta

@dataclass
class QueryTask:
    query_type: str

@dataclass
class GripperTask:
    gripper_state: int     # 0 = closed, 255 = open


# initialize database
mini_tasks = dict();

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> GRIPPER
task_name = "open_gripper";      # ----------------------------------
mini_tasks[task_name] = GripperTask(255);

task_name = "close_gripper";      # ----------------------------------
mini_tasks[task_name] = GripperTask(0);

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> BASE
task_name = "go_to_mixing_station";      # ---------------------------------
mini_tasks[task_name] = MiRTask([17,18,0]);

task_name = "go_to_electrochem_station";  # ---------------------------------
mini_tasks[task_name] = MiRTask([16,11,0]);

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ARM ABSOLUTE
task_name = "gripper_to_front";          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],True);
mini_tasks[task_name].action_server = "pose_based_cartesian_trajectory";
mini_tasks[task_name].position_list = [
    geometry_msgs.Pose(
        geometry_msgs.Vector3(0, 0.2, 0.3), geometry_msgs.Quaternion(0, 0, 0, 1)
    ),
]
mini_tasks[task_name].duration_list = [5.0]


task_name = "gripper_to_home";          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],True);
mini_tasks[task_name].action_server = "pose_based_cartesian_trajectory";
mini_tasks[task_name].position_list = [
    geometry_msgs.Pose(
        geometry_msgs.Vector3(0.0, -0.3, 0.4), geometry_msgs.Quaternion(0, 0, 0, 1)
    )
]
mini_tasks[task_name].duration_list = [5.0]

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ARM RELATIVE
task_name = "get_clean_beaker";          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],False);
mini_tasks[task_name].action_server = "pose_based_cartesian_trajectory";
mini_tasks[task_name].position_list = [
    geometry_msgs.Pose(
        geometry_msgs.Vector3(0, 0,   0.1), geometry_msgs.Quaternion(0, 0, 0, 1)
    ),
    geometry_msgs.Pose(
        geometry_msgs.Vector3(0, 0.4, 0  ), geometry_msgs.Quaternion(0, 0, 0, 1)
    )
];
mini_tasks[task_name].duration_list = [2.0, 3.0];

task_name = "beaker_to_fluids";          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],False);
mini_tasks[task_name].action_server = "pose_based_cartesian_trajectory";
mini_tasks[task_name].position_list = [
    geometry_msgs.Pose(
        geometry_msgs.Vector3(0, 0,   0.1), geometry_msgs.Quaternion(0, 0, 0, 1)
    )
];
mini_tasks[task_name].duration_list = [2.0];

task_name = "add_next_reagent";          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],False);
mini_tasks[task_name].action_server = "pose_based_cartesian_trajectory";
mini_tasks[task_name].position_list = [
    geometry_msgs.Pose(
        geometry_msgs.Vector3(0,  0.2, 0), geometry_msgs.Quaternion(0, 0, 0, 1)
    ),
    geometry_msgs.Pose(
        geometry_msgs.Vector3(0,  0,   0.1), geometry_msgs.Quaternion(0, 0, 0, 1)
    ),
    geometry_msgs.Pose(
        geometry_msgs.Vector3(0,  0,  -0.1), geometry_msgs.Quaternion(0, 0, 0, 1)
    ),
    geometry_msgs.Pose(
        geometry_msgs.Vector3(0, -0.2, 0), geometry_msgs.Quaternion(0, 0, 0, 1)
    )
];
mini_tasks[task_name].duration_list = [2.0, 3.0, 3.0, 2.0];

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> NOT READY
task_name = "put_beaker_on_table";       # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],False);

task_name = "put_wp_in_beaker";          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],False);

task_name = "put_aux_in_beaker";         # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],False);

task_name = "make_measurements";         # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],False);

task_name = "rinse_aux";                 # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],False);

task_name = "dispose_used_wp";           # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],False);

task_name = "dispose_used_beaker";       # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],False);

task_name = "grab_mic";                  # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],False);

task_name = "lift_mic";                  # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],False);

task_name = "drop_mic";                  # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],False);

# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> QUERY
task_name = "more_reagents_query";       # ---------------------------------
mini_tasks[task_name] = QueryTask("");

task_name = "more_tests_query";          # ---------------------------------
mini_tasks[task_name] = QueryTask("");



# ============================================================================
#                                           CARDINAL POSITIONS
# ============================================================================

# https://www.andre-gaschler.com/rotationconverter/
#     enter Rx,Ry,Rz in radians into "axis with angle magnitude" on left side
#     pull quaternion values [x,y,z,w] from right side of page
#
#          X      Y     Z       Rx     Ry     Rz        raw_Qx      raw_Qy      raw_Qz      raw_Qw                 Qx      Qy      Qz      Qw
# +y       0    365  -333     4.72   0.02  -0.02     [ 0.704368,   0.0029846, -0.0029846, -0.7098224 ]          0.707   0.000   0.000  -0.707
# -x    -365      0  -333     2.40   2.42  -2.43     [ 0.4969667,  0.5011081, -0.5031788, -0.4987242 ]          0.500   0.500  -0.500  -0.500
# -y       0   -365  -333     0.01  -2.22   2.23     [ 0.003178,  -0.7055102,  0.7086882, -0.0025282 ]          0.000  -0.707   0.707   0.000
# +x     365      0  -333     2.43  -2.41   2.43     [ 0.5001256, -0.4960093,  0.5001256, -0.5037098 ]          0.500  -0.500   0.500  -0.500
nom_plus_y  = geometry_msgs.Pose(geometry_msgs.Vector3(    0,  365, -100 ), geometry_msgs.Quaternion(-0.707,  0.000,  0.000,  0.707 ))
nom_minus_x = geometry_msgs.Pose(geometry_msgs.Vector3( -365,    0, -100 ), geometry_msgs.Quaternion(-0.500, -0.500,  0.500,  0.500 ))
nom_minus_y = geometry_msgs.Pose(geometry_msgs.Vector3(    0, -365, -100 ), geometry_msgs.Quaternion( 0.000,  0.707, -0.707,  0.000 ))
nom_plus_x  = geometry_msgs.Pose(geometry_msgs.Vector3(  365,    0, -100 ), geometry_msgs.Quaternion(-0.500,  0.500, -0.500,  0.500 ))


task_name = "gripper_to_plus_y";          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],True);
mini_tasks[task_name].action_server = "pose_based_cartesian_trajectory";
mini_tasks[task_name].position_list = [ nom_plus_y ]
mini_tasks[task_name].duration_list = [10.0]

task_name = "gripper_to_minus_x";          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],True);
mini_tasks[task_name].action_server = "pose_based_cartesian_trajectory";
mini_tasks[task_name].position_list = [ nom_minus_x ]
mini_tasks[task_name].duration_list = [10.0]

task_name = "gripper_to_minus_y";          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],True);
mini_tasks[task_name].action_server = "pose_based_cartesian_trajectory";
mini_tasks[task_name].position_list = [ nom_minus_y ]
mini_tasks[task_name].duration_list = [10.0]

task_name = "gripper_to_plus_x";          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],True);
mini_tasks[task_name].action_server = "pose_based_cartesian_trajectory";
mini_tasks[task_name].position_list = [ nom_plus_x ]
mini_tasks[task_name].duration_list = [10.0]

task_name = "gripper_p1";          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],True);
mini_tasks[task_name].action_server = "pose_based_cartesian_trajectory";
mini_tasks[task_name].position_list = [
    geometry_msgs.Pose(geometry_msgs.Vector3(-.100,.240,.665),geometry_msgs.Quaternion(-.707,0,0,.707 )), 
]
mini_tasks[task_name].duration_list = [8.0]

task_name = "gripper_fwd";          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],False);
mini_tasks[task_name].action_server = "pose_based_cartesian_trajectory";
mini_tasks[task_name].position_list = [
    geometry_msgs.Pose(
        geometry_msgs.Vector3(0, 0,   0.001), geometry_msgs.Quaternion(-.707,0,0,.707)
    ),
];
mini_tasks[task_name].duration_list = [8.0]


#               base   shoulder   elbow   wrist_1  wrist_2  wrist_3
#             -------- -------- -------- -------- -------- --------
# vicinity 1     220      -27     -137      -14      -39        0
def deg2rad(lst):
    radlst = []
    for j in lst:
        radlst.append(j*np.pi/180.)
    return radlst

task_name = "joint_v1";          # ---------------------------------
mini_tasks[task_name] = ArmTask("",[],[],True);
mini_tasks[task_name].action_server = "joint_based_trajectory";
mini_tasks[task_name].position_list = [
    deg2rad([220,-27,-137,-14,-39,0]),
]
mini_tasks[task_name].duration_list = [8.0]


# ============================================================================
#                                           EXAMPLE PROTOCOLS
# ============================================================================
demo_protocol_1 = [ "go_to_mixing_station", "gripper_to_front", "open_gripper", "get_clean_beaker", "close_gripper", "beaker_to_fluids", "add_next_reagent", "add_next_reagent", "add_next_reagent","go_to_electrochem_station" ];
test_absolute = [ "gripper_to_home", "gripper_to_front", "gripper_to_home", "gripper_to_front" ];
test_relative = [ "gripper_to_front", "get_clean_beaker", "get_clean_beaker", "beaker_to_fluids" ];

test_py = ["gripper_to_plus_y"]
test_mx = ["gripper_to_minus_x"]
test_my = ["gripper_to_minus_y"]
test_px = ["gripper_to_plus_x"]

#test_this = ["gripper_p1","gripper_fwd"]
test_this = ["joint_v1"]

