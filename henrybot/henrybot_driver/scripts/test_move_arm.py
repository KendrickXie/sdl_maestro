import sys

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
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# All of those controllers can be used to execute joint-based trajectories.
# The scaled versions should be preferred over the non-scaled versions.
JOINT_TRAJECTORY_CONTROLLERS = [
    "scaled_pos_joint_traj_controller",
    "scaled_vel_joint_traj_controller",
    "pos_joint_traj_controller",
    "vel_joint_traj_controller",
    "forward_joint_traj_controller",
]

# All of those controllers can be used to execute Cartesian trajectories.
# The scaled versions should be preferred over the non-scaled versions.
CARTESIAN_TRAJECTORY_CONTROLLERS = [
    "pose_based_cartesian_traj_controller",
    "joint_based_cartesian_traj_controller",
    "forward_cartesian_traj_controller",
]

# We'll have to make sure that none of these controllers are running, as they will
# be conflicting with the joint trajectory controllers
CONFLICTING_CONTROLLERS = ["joint_group_vel_controller", "twist_controller"]


class ArmClient:

    def __init__(self):
        rospy.init_node("arm_client")
        
        timeout = rospy.Duration(10)

        self.switch_srv = rospy.ServiceProxy(
            "controller_manager/switch_controller", SwitchController
        )
        self.load_srv = rospy.ServiceProxy("controller_manager/load_controller", LoadController)
        try:
            self.switch_srv.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr("Could not reach controller switch service. Msg: {}".format(err))
            sys.exit(-1)

        self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[0]
        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[0]

    def send_joint_trajectory(self,position_list,duration_list):
        """Creates a trajectory and sends it using the selected action server"""

        # make sure the correct controller is loaded and activated
        self.switch_controller(self.joint_trajectory_controller)
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

        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

        result = trajectory_client.get_result()
        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))

    def send_cartesian_trajectory(self, pose_list, duration_list):
        """Creates a Cartesian trajectory and sends it using the selected action server"""
        self.switch_controller(self.cartesian_trajectory_controller)

        # make sure the correct controller is loaded and activated
        goal = FollowCartesianTrajectoryGoal()
        trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_cartesian_trajectory".format(self.cartesian_trajectory_controller),
            FollowCartesianTrajectoryAction,
        )

        for i, pose in enumerate(pose_list):
            point = CartesianTrajectoryPoint()
            point.pose = pose
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)

        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

        result = trajectory_client.get_result()

        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))


    def choose_controller(self):
        """Ask the user to select the desired controller from the available list."""
        print("Available trajectory controllers:")
        for (index, name) in enumerate(JOINT_TRAJECTORY_CONTROLLERS):
            print("{} (joint-based): {}".format(index, name))
        for (index, name) in enumerate(CARTESIAN_TRAJECTORY_CONTROLLERS):
            print("{} (Cartesian): {}".format(index + len(JOINT_TRAJECTORY_CONTROLLERS), name))
        choice = -1
        while choice < 0:
            input_str = input(
                "Please choose a controller by entering its number (Enter '0' if "
                "you are unsure / don't care): "
            )
            try:
                choice = int(input_str)
                if choice < 0 or choice >= len(JOINT_TRAJECTORY_CONTROLLERS) + len(
                    CARTESIAN_TRAJECTORY_CONTROLLERS
                ):
                    print(
                        "{} not inside the list of options. "
                        "Please enter a valid index from the list above.".format(choice)
                    )
                    choice = -1
            except ValueError:
                print("Input is not a valid number. Please try again.")
        if choice < len(JOINT_TRAJECTORY_CONTROLLERS):
            self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[choice]
            return "joint_based"

        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[
            choice - len(JOINT_TRAJECTORY_CONTROLLERS)
        ]
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


if __name__ == "__main__":
    client = ArmClient()

    trajectory_type = client.choose_controller()

    if trajectory_type == "joint_based":
        # The following list are arbitrary positions
        # Change to your own needs if desired
        position_list = [[0, -1.57, -1.57, 0, 0, 0]]
        position_list.append([0.2, -1.57, -1.57, 0, 0, 0])
        position_list.append([-0.5, -1.57, -1.2, 0, 0, 0])
        duration_list = [3.0, 7.0, 10.0]
        ##lists should be the same lenght
        client.send_joint_trajectory(position_list,duration_list)
    elif trajectory_type == "cartesian":
        # The following list are arbitrary positions
        # Change to your own needs if desired
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
        duration_list = [3.0, 4.0, 5.0, 6.0, 7.0]
        client.send_cartesian_trajectory(pose_list,duration_list)