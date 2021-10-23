#!/usr/bin/env python3

import numpy as np
import math
from dataclasses import dataclass

from time import sleep 
import rospy
from client_arm import ArmClient
from  argparse import ArgumentParser

# ========================== calibration
origin = [ -0.518, -0.244 ]
twotwo = [ -0.801, +0.101 ]

def ttt(i,j):
    # x =  Y_TD
    # y = -X_TD
    dx = (twotwo[1]-origin[1])/2.
    dy = -(twotwo[0]-origin[0])/2.
    x = origin[1]+(i+0.5)*dx
    y = -origin[1]+(j+0.5)*dy

z_block = -0.301


# ==========================
HORZ_GRIP = 0              # model_type
HORZ_YPOS_GRIP = 2
HORZ_XPOS_GRIP = 3
HORZ_XNEG_GRIP = 4
VERT_GRIP = 1

DOCK_FRONT = -2*np.pi/2    # dock_angle
DOCK_LEFT  = -1*np.pi/2
DOCK_BACK  =  0*np.pi/2
DOCK_RIGHT =  1*np.pi/2

# model_params = [ upper_len, lower_len, wrist2_len, wrist3_len, arm_offset ]
ur5e_params = [ 0.425, 0.3922, 0.12, 0.35, 0.15 ]       # linkage lengths in meters

# ================
def radians_to_degrees(rad_list):
    deg_list = []
    for a in rad_list:
        deg_list.append(a*180./np.pi)
    return deg_list

# ========================== shoulder and elbow joints of the reduced problem
#
def shoulder_and_elbow(r_prime, z_prime, model_params):
    err = 0

    upper_arm = model_params[0]     # length of arm segments
    lower_arm = model_params[1]

    hyp2 = r_prime**2 + z_prime**2    # hypotenuse of the triangle
    zeta = math.atan2(z_prime,r_prime)   # angle of the hypotenuse

    gamma = math.acos( (upper_arm**2 + lower_arm**2 - hyp2) / (2*upper_arm*lower_arm) )
    beta = math.acos( (hyp2 + upper_arm**2 - lower_arm**2) / (2*math.sqrt(hyp2)*upper_arm) )

    shoulder_lift = -(zeta + beta)
    elbow_angle = np.pi - gamma

    #print(f'r_prime = {r_prime:7.3f}  z_prime = {z_prime:7.3f}  upper_arm = {upper_arm:7.3f}  lower_arm = {lower_arm:7.3f}  hyp = {math.sqrt(hyp2):7.3f}' )
    #print(f'zeta = {zeta*180/np.pi:4.0f}  gamma = {gamma*180/np.pi:4.0f}  beta = {beta*180/np.pi:4.0f}  shoulder_lift = {shoulder_lift*180/np.pi:4.0f}  elbow_angle = {elbow_angle*180/np.pi:4.0f} ')

    return shoulder_lift, elbow_angle, err


# ========================== joints from xyz
#
def xyz_to_joints(model_type, model_params, position, orientation = 0, dock_angle = DOCK_BACK):
    err = -1
    joints = []
    upper_len = model_params[0]
    lower_len = model_params[1]
    wrist2_len = model_params[2]
    wrist3_len = model_params[3]
    arm_offset = model_params[4]
    x = position[0]
    y = position[1]
    z = position[2]

    if (model_type == VERT_GRIP): # ----------------------------- VERT model
        #print('Using VERT model')
        # handle effector offset and base rotation -- set up the reduced problem
        R = math.sqrt(x**2 + y**2)
        #print(f'y = {y:7.3f} and R = {R:7.3f}')
        delta_theta = math.asin(arm_offset/R)
        theta = math.atan2(y,x) - delta_theta
        r_v = math.sqrt(R**2-arm_offset**2) - wrist2_len   #x - wrist2_len
        z_v = z + wrist3_len
        base = theta  + dock_angle

        # the hard part -- compute the shoulder and elbow joint angles
        shoulder, elbow, err = shoulder_and_elbow(r_v, z_v, model_params)

        # button it up -- compute remaing joints based on constrained model
        wrist_1 = -np.pi/2 - (shoulder + elbow)
        wrist_2 = -np.pi/2
        wrist_3 = theta - np.pi/2 + orientation*np.pi/180.0
    
        joints = [ base, shoulder, elbow, wrist_1, wrist_2, wrist_3 ]
        err = 0

    elif model_type == HORZ_GRIP: # --------------------------- HORZ model
        # handle effector offset and base rotation -- set up the reduced problem
        R = math.sqrt(x**2 + y**2)
        delta_theta = math.asin(arm_offset/R)
        theta = math.atan2(y,x) - delta_theta
        r_h = math.sqrt(R**2-arm_offset**2) - wrist3_len
        z_h = z + wrist2_len
        base = theta + dock_angle

        # the hard part -- compute the shoulder and elbow joint angles
        shoulder, elbow, err = shoulder_and_elbow(r_h, z_h, model_params)

        # button it up -- compute remaing joints based on constrained model
        wrist_1 = 0 - (shoulder + elbow)
        wrist_2 = dock_angle + np.pi/2
        wrist_3 = np.pi
    
        joints = [ base, shoulder, elbow, wrist_1, wrist_2, wrist_3 ]
        err = 0
    
    elif model_type == HORZ_YPOS_GRIP: # --------------------------- HORZ model
        # handle effector offset and base rotation -- set up the reduced problem
        R = math.sqrt(x**2 + (y-wrist3_len)**2)
        delta_theta = math.asin(arm_offset/R)
        theta = math.atan2(y-wrist3_len,x) - delta_theta
        r_h = math.sqrt(R**2-arm_offset**2)
        z_h = z + wrist2_len
        base = theta + dock_angle

        # the hard part -- compute the shoulder and elbow joint angles
        shoulder, elbow, err = shoulder_and_elbow(r_h, z_h, model_params)

        # button it up -- compute remaing joints based on constrained model
        wrist_1 = 0 - (shoulder + elbow)
        wrist_2 = theta
        wrist_3 = np.pi
    
        joints = [ base, shoulder, elbow, wrist_1, wrist_2, wrist_3 ]
        err = 0
    
    elif model_type == HORZ_XPOS_GRIP: # --------------------------- HORZ model
        # handle effector offset and base rotation -- set up the reduced problem
        R = math.sqrt((x-wrist3_len)**2 + y**2)
        delta_theta = math.asin(arm_offset/R)
        theta = math.atan2(y,x-wrist3_len) - delta_theta
        r_h = math.sqrt(R**2-arm_offset**2)
        z_h = z + wrist2_len
        base = theta + dock_angle

        # the hard part -- compute the shoulder and elbow joint angles
        shoulder, elbow, err = shoulder_and_elbow(r_h, z_h, model_params)

        # button it up -- compute remaing joints based on constrained model
        wrist_1 = 0 - (shoulder + elbow)
        wrist_2 = theta + np.pi/2
        wrist_3 = np.pi
    
        joints = [ base, shoulder, elbow, wrist_1, wrist_2, wrist_3 ]
        err = 0
    
    elif model_type == HORZ_XNEG_GRIP: # --------------------------- HORZ model
        # handle effector offset and base rotation -- set up the reduced problem
        R = math.sqrt((x+wrist3_len)**2 + y**2)
        delta_theta = math.asin(arm_offset/R)
        theta = math.atan2(y,x+wrist3_len) - delta_theta
        r_h = math.sqrt(R**2-arm_offset**2)
        z_h = z + wrist2_len
        base = theta + dock_angle

        # the hard part -- compute the shoulder and elbow joint angles
        shoulder, elbow, err = shoulder_and_elbow(r_h, z_h, model_params)

        # button it up -- compute remaing joints based on constrained model
        wrist_1 = 0 - (shoulder + elbow)
        wrist_2 = theta - np.pi/2
        wrist_3 = np.pi
    
        joints = [ base, shoulder, elbow, wrist_1, wrist_2, wrist_3 ]
        err = 0
    
    else: # --------------------------------------------------- BOOM
        err = -2
        print('VERT ({VERT_GRIP}) or HORZ ({HORZ_GRIP}) are only model_types accepted: {model_type}.')

    #print(f'Joints (rad): [ {joints[0]:7.3f} {joints[1]:7.3f} {joints[2]:7.3f} {joints[3]:7.3f} {joints[4]:7.3f} {joints[5]:7.3f} ]')
    j_degs = radians_to_degrees(joints)
    #print(f'Joints (deg): [ {j_degs[0]:7.2f} {j_degs[1]:7.2f} {j_degs[2]:7.2f} {j_degs[3]:7.2f} {j_degs[4]:7.2f} {j_degs[5]:7.2f} ]')
    return joints, err


#xyz_to_joints(VERT_GRIP, ur5e_params, [0.0, -0.685, 0.0])



def main():

    parser = ArgumentParser()
    parser.add_argument("-m", default=16.0, type=str)
    parser.add_argument("-x", default=23.0, type=float)
    parser.add_argument("-y", default=23.0, type=float)
    parser.add_argument("-z", default=23.0, type=float)
    parser.add_argument("-o", default=0,    type=float)
    parser.add_argument("-d", default="BACK", type=str)
    args = parser.parse_args()

    rospy.init_node("arm_python")

    model = VERT_GRIP
    if "VERT" in args.m:
        model = VERT_GRIP
    elif "YPOS" in args.m:
        model = HORZ_YPOS_GRIP
    elif "XPOS" in args.m:
        model = HORZ_XPOS_GRIP
    elif "XNEG" in args.m:
        model = HORZ_XNEG_GRIP
    elif "HORZ" in args.m:
        model = HORZ_GRIP
    pos_X = args.x
    pos_Y = args.y
    pos_Z = args.z
    dock_dir = DOCK_BACK
    if "BACK" in args.d:
        dock_dir = DOCK_BACK
    elif "FRONT" in args.d:
        dock_dir = DOCK_FRONT
    elif "LEFT" in args.d:
        dock_dir = DOCK_LEFT
    elif "RIGHT" in args.d:
        dock_dif = DOCK_RIGHT

    joints,err = xyz_to_joints(model, ur5e_params, [pos_X,pos_Y,pos_Z], orientation=args.o, dock_angle=dock_dir)

    arm = ArmClient()
    arm.move([joints],[8.0],"forward_joint_trajectory_controller")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


