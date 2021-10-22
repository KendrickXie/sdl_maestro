#!/usr/bin/env python3
import sys
from time import sleep

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
import geometry_msgs.msg as geometry_msgs
from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
)

# If your robot description is created with a tf_prefix, those would have to be adapted
JOINT_NAMES = [
    "shoulder_pan_joint", #1
    "shoulder_lift_joint", #2 
    "elbow_joint", #3
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


JOINT_TRAJECTORY_CONTROLLERS = [
    "scaled_pos_joint_traj_controller",
    "scaled_vel_joint_traj_controller",
    "pos_joint_traj_controller",
    "vel_joint_traj_controller",
    "forward_joint_traj_controller",
]

CARTESIAN_TRAJECTORY_CONTROLLERS = [
    "pose_based_cartesian_traj_controller",
    "joint_based_cartesian_traj_controller",
    "forward_cartesian_traj_controller",
]

CONFLICTING_CONTROLLERS = ["joint_group_vel_controller", "twist_controller"]


class ArmClient:
    """Small trajectory client to test a joint trajectory"""

    def __init__(self):

        timeout = rospy.Duration(5)
        self.switch_srv = rospy.ServiceProxy(
            "controller_manager/switch_controller", SwitchController
        )
        self.load_srv = rospy.ServiceProxy("controller_manager/load_controller", LoadController)
        try:
            self.switch_srv.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr("Could not reach controller switch service. Msg: {}".format(err))
            sys.exit(-1)

#FORWARD_JOINT_ONLY#        self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[0]
        self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[4]
        self.switch_controller(self.joint_trajectory_controller)
        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[0]

    def send_joint_trajectory(self, position_list, duration_list):
        """Creates a trajectory and sends it using the selected action server"""

        # make sure the correct controller is loaded and activated
#FORWARD_JOINT_ONLY#        self.switch_controller(self.joint_trajectory_controller)
        trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_joint_trajectory".format(self.joint_trajectory_controller),
            FollowJointTrajectoryAction,
        )

        # Create and fill trajectory goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES

        for i, position in enumerate(position_list):
            point = JointTrajectoryPoint()
            point.positions = position
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)

        sleep(0.5)
        rospy.loginfo("Executing trajectory using the {}".format(self.joint_trajectory_controller))

        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

        result = trajectory_client.get_result()
        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))

#FORWARD_JOINT_ONLY#     def send_cartesian_trajectory(self, pose_list, duration_list):
#FORWARD_JOINT_ONLY#         """Creates a Cartesian trajectory and sends it using the selected action server"""
#FORWARD_JOINT_ONLY#         self.switch_controller(self.cartesian_trajectory_controller)
#FORWARD_JOINT_ONLY# 
#FORWARD_JOINT_ONLY#         # make sure the correct controller is loaded and activated
#FORWARD_JOINT_ONLY#         goal = FollowCartesianTrajectoryGoal()
#FORWARD_JOINT_ONLY#         trajectory_client = actionlib.SimpleActionClient(
#FORWARD_JOINT_ONLY#             "{}/follow_cartesian_trajectory".format(self.cartesian_trajectory_controller),
#FORWARD_JOINT_ONLY#             FollowCartesianTrajectoryAction,
#FORWARD_JOINT_ONLY#         )
#FORWARD_JOINT_ONLY# 
#FORWARD_JOINT_ONLY# 
#FORWARD_JOINT_ONLY#         for i, pose in enumerate(pose_list):
#FORWARD_JOINT_ONLY#             point = CartesianTrajectoryPoint()
#FORWARD_JOINT_ONLY#             point.pose = pose
#FORWARD_JOINT_ONLY#             point.time_from_start = rospy.Duration(duration_list[i])
#FORWARD_JOINT_ONLY#             goal.trajectory.points.append(point)
#FORWARD_JOINT_ONLY# 
#FORWARD_JOINT_ONLY#         sleep(2)
#FORWARD_JOINT_ONLY#         rospy.loginfo(
#FORWARD_JOINT_ONLY#             "Executing trajectory using the {}".format(self.cartesian_trajectory_controller)
#FORWARD_JOINT_ONLY#         )
#FORWARD_JOINT_ONLY#         trajectory_client.send_goal(goal)
#FORWARD_JOINT_ONLY#         trajectory_client.wait_for_result()
#FORWARD_JOINT_ONLY# 
#FORWARD_JOINT_ONLY#         result = trajectory_client.get_result()
#FORWARD_JOINT_ONLY# 
#FORWARD_JOINT_ONLY#         rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))

    def choose_controller(self, controller):
        if controller in JOINT_TRAJECTORY_CONTROLLERS:
            self.joint_trajectory_controller = controller
            return "joint_based"

        if controller in CARTESIAN_TRAJECTORY_CONTROLLERS:
            self.cartesian_trajectory_controller = controller
            return "cartesian"

    def switch_controller(self, target_controller):
        """Activates the desired controller and stops all others from the predefined list above"""
        other_controllers = (
            JOINT_TRAJECTORY_CONTROLLERS
            + CARTESIAN_TRAJECTORY_CONTROLLERS
            + CONFLICTING_CONTROLLERS
        )

        other_controllers.remove(target_controller)

        srv = LoadControllerRequest()
        srv.name = target_controller
        self.load_srv(srv)

        srv = SwitchControllerRequest()
        srv.stop_controllers = other_controllers
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_srv(srv)

    def move(self, pos_list, duration_list, controller="pose_based_cartesian_traj_controller"):
#FORWARD_JOINT_ONLY#        trajectory_type = self.choose_controller(controller)

#FORWARD_JOINT_ONLY#        if trajectory_type == "joint_based":
            self.send_joint_trajectory(pos_list, duration_list)
#FORWARD_JOINT_ONLY#        elif trajectory_type == "cartesian":
#FORWARD_JOINT_ONLY#            self.send_cartesian_trajectory(pos_list, duration_list)
#FORWARD_JOINT_ONLY#        else:
#FORWARD_JOINT_ONLY#            raise ValueError(
#FORWARD_JOINT_ONLY#                "I only understand types 'joint_based' and 'cartesian', but got '{}'".format(trajectory_type)
#FORWARD_JOINT_ONLY#    )



if __name__ == "__main__":


    rospy.init_node('UR5e')
    test_controller = "pose_based_cartesian_traj_controller"
    ##
    client = ArmClient()
    ##
    trajectory_type = client.choose_controller(test_controller)
    ##

    position_list = [[0, -1.57, -1.57, 0, 0, 0]]
    position_list.append([0.2, -1.57, -1.57, 0, 0, 0])
    position_list.append([-0.5, -1.57, -1.2, 0, 0, 0])
    duration_list = [3.0, 7.0, 10.0]


    pose_list = [
        geometry_msgs.Pose(
            geometry_msgs.Vector3(0.4, -0.1, 0.4), geometry_msgs.Quaternion(0, 0, 0, 1)
        ),
        geometry_msgs.Pose(
            geometry_msgs.Vector3(0.4, -0.1, 0.6), geometry_msgs.Quaternion(0, 0, 0, 1)
        ),
        geometry_msgs.Pose(
            geometry_msgs.Vector3(0.4, 0.3, 0.6), geometry_msgs.Quaternion(0, 0, 0, 1)
        ),
        geometry_msgs.Pose(
            geometry_msgs.Vector3(0.4, 0.3, 0.4), geometry_msgs.Quaternion(0, 0, 0, 1)
        ),
        geometry_msgs.Pose(
            geometry_msgs.Vector3(0.4, -0.1, 0.4), geometry_msgs.Quaternion(0, 0, 0, 1)
        ),
    ]
    duration_list = [5.0, 7.0, 10.0, 13.0, 16.0]


    # if trajectory_type == "joint_based":
    #     client.send_joint_trajectory(position_list, duration_list)
    # elif trajectory_type == "cartesian":
    #     client.send_cartesian_trajectory(pose_list, duration_list)
    # else:
    #     raise ValueError(
    #         "I only understand types 'joint_based' and 'cartesian', but got '{}'".format(
    #             trajectory_type
    #         )
    #     )
