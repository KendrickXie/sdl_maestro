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
